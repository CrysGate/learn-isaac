"""ScaleBench manager-based environment runtime."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import torch
from isaaclab.envs import ManagerBasedEnv
from isaaclab.envs.common import VecEnvObs

from scale_bench.isaaclab.builders.environment import ScaleBenchEnvCfg
from scale_bench.isaaclab.mdp.events import ResetTaskLayout
from scale_bench.isaaclab.runtime.io_descriptors import (
    build_io_descriptors,
    validate_io_descriptors,
)


class ScaleBenchEnv(ManagerBasedEnv):
    """Own the simulation lifecycle and expose runtime-derived metadata."""

    def __init__(self, cfg: ScaleBenchEnvCfg) -> None:
        self._task_layout_reset: ResetTaskLayout | None = None
        super().__init__(cfg)

        if cfg.events.task_layout is not None:
            term = self.event_manager.get_term_cfg("task_layout").func
            if not isinstance(term, ResetTaskLayout):
                raise RuntimeError("task_layout event did not initialize correctly")
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
        if action.dtype != torch.float32:
            raise TypeError(f"action dtype must be torch.float32, got {action.dtype}")
        if action.device != torch.device(self.device):
            raise ValueError(
                f"action must be on environment device {self.device}, "
                f"got {action.device}"
            )
        if not torch.isfinite(action).all():
            raise ValueError("action must contain only finite values")
        return super().step(action)

    def reset(
        self,
        seed: int | None = None,
        env_ids: Sequence[int] | torch.Tensor | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[VecEnvObs, dict]:
        resolved_env_ids = None
        if env_ids is not None:
            resolved_env_ids = torch.as_tensor(
                env_ids,
                device=self.device,
                dtype=torch.int32,
            )
            if resolved_env_ids.ndim != 1:
                raise ValueError("env_ids must be a one-dimensional sequence")
        observation, info = super().reset(
            seed=seed,
            env_ids=resolved_env_ids,
            options=options,
        )
        if self._task_layout_reset is not None:
            info["episode"] = self._task_layout_reset.episode_info(
                resolved_env_ids
            )
        return observation, info


__all__ = ["ScaleBenchEnv"]
