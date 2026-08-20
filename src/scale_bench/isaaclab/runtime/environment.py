"""ScaleBench manager-based environment runtime."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import torch
from isaaclab.envs import ManagerBasedEnv
from isaaclab.envs.common import VecEnvObs

from scale_bench.isaaclab.builders.environment import ScaleBenchEnvCfg
from scale_bench.isaaclab.mdp.events import ResetTaskLayout, resolve_env_ids
from scale_bench.isaaclab.runtime.io_descriptors import build_io_descriptors, validate_io_descriptors
from scale_bench.tasks.common.layout import TaskLayout


class ScaleBenchEnv(ManagerBasedEnv):
    """Own the simulation lifecycle and expose runtime-derived metadata."""

    def __init__(self, cfg: ScaleBenchEnvCfg) -> None:
        self._task_layout_reset: ResetTaskLayout
        super().__init__(cfg)

        term = self.event_manager.get_term_cfg("task_layout").func
        self._task_layout_reset = term
        validate_io_descriptors(self, self.get_IO_descriptors)

    def load_managers(self) -> None:
        """Load native managers and run startup events for this subclass."""

        super().load_managers()
        if "startup" in self.event_manager.available_modes:
            self.event_manager.apply(mode="startup")

    @property
    def get_IO_descriptors(self) -> dict[str, Any]:
        """Return descriptors derived from initialized managers and sensors."""

        return build_io_descriptors(self, super().get_IO_descriptors)

    def step(self, action: torch.Tensor) -> tuple[VecEnvObs, dict]:
        """Validate and execute one action under the public environment contract."""

        expected_shape = (self.num_envs, self.action_manager.total_action_dim)
        if not isinstance(action, torch.Tensor):
            raise TypeError("action must be a torch.Tensor")
        if action.shape != expected_shape:
            raise ValueError(
                f"action shape must be {expected_shape}, got {tuple(action.shape)}"
            )
        return super().step(action)

    def reset(
        self,
        seed: int | None = None,
        env_ids: Sequence[int] | torch.Tensor | None = None,
        options: dict[str, Any] | None = None,
        task_layouts: Sequence[TaskLayout] | None = None,
    ) -> tuple[VecEnvObs, dict]:
        resolved_env_ids = (
            None if env_ids is None else resolve_env_ids(env_ids, self.num_envs)
        )
        env_id_tensor = (
            None
            if resolved_env_ids is None
            else torch.tensor(
                resolved_env_ids,
                device=self.device,
                dtype=torch.int32,
            )
        )
        if task_layouts is not None:
            self._task_layout_reset.assign_layouts(resolved_env_ids, task_layouts)
        observation, info = super().reset(
            seed=seed,
            env_ids=env_id_tensor,
            options=options,
        )
        if self._task_layout_reset is not None:
            episode_info = self._task_layout_reset.episode_info(
                resolved_env_ids
            )
            info["episode"] = episode_info
            if self.recorder_manager.active_terms:
                for env_id, layout_seed in zip(
                    episode_info["env_ids"],
                    episode_info["layout_seeds"],
                    strict=True,
                ):
                    self.recorder_manager.get_episode(env_id).seed = layout_seed
        return observation, info

    def complete_episodes(
        self,
        *,
        success: Sequence[bool] | torch.Tensor,
        env_ids: Sequence[int] | torch.Tensor | None = None,
        demo_ids: Sequence[int] | None = None,
    ) -> None:
        """Mark and export completed episodes before their next reset."""

        if not self.recorder_manager.active_terms:
            raise RuntimeError("episode recording is not enabled")
        resolved_env_ids = resolve_env_ids(env_ids, self.num_envs)
        success_tensor = _resolve_success_values(
            success,
            count=len(resolved_env_ids),
            device=self.device,
        )
        env_id_tensor = torch.tensor(
            resolved_env_ids,
            device=self.device,
            dtype=torch.int32,
        )
        self.recorder_manager.set_success_to_episodes(
            env_id_tensor,
            success_tensor,
        )
        self.recorder_manager.export_episodes(env_id_tensor, demo_ids=demo_ids)

    def close(self) -> None:
        """Close dataset handles deterministically, then release the simulation."""

        try:
            if not self._is_closed and hasattr(self, "recorder_manager"):
                self.recorder_manager.close()
        finally:
            super().close()


def _resolve_success_values(
    success: Sequence[bool] | torch.Tensor,
    *,
    count: int,
    device: str,
) -> torch.Tensor:
    if isinstance(success, torch.Tensor):
        if success.dtype != torch.bool:
            raise TypeError("success must contain boolean values")
        if success.ndim != 1:
            raise ValueError("success must be one-dimensional")
        values = success.to(device=device)
    else:
        raw_values = tuple(success)
        if any(type(value) is not bool for value in raw_values):
            raise TypeError("success must contain boolean values")
        values = torch.tensor(raw_values, device=device, dtype=torch.bool)
    if values.shape != (count,):
        raise ValueError(
            f"success must contain one value per environment ({count}), "
            f"got shape {tuple(values.shape)}"
        )
    return values


__all__ = ["ScaleBenchEnv"]
