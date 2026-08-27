"""ScaleBench manager-based environment runtime."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import torch
from isaaclab.envs import ManagerBasedEnv
from isaaclab.envs.common import VecEnvObs

from scale_bench.isaaclab.builders.environment import ScaleBenchEnvCfg
from scale_bench.isaaclab.mdp.events import resolve_env_ids
from scale_bench.isaaclab.runtime.io_descriptors import build_io_descriptors, validate_io_descriptors

from scale_bench.runtime.episodes import EpisodeTermination
from scale_bench.runtime.recording import StepSemantics
from scale_bench.tasks.common.layout import TaskLayout
from scale_bench.tasks.common.task import Task


class ScaleBenchEnv(ManagerBasedEnv):
    """Own the simulation lifecycle and expose runtime-derived metadata."""

    def __init__(self, cfg: ScaleBenchEnvCfg) -> None:
        self._task: Task = cfg.task
        self._step_semantics: tuple[StepSemantics | None, ...] = (
            (None,) * cfg.scene.num_envs
        )
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

    @property
    def recording_enabled(self) -> bool:
        """Whether this environment has active episode recorder terms."""

        return bool(self.recorder_manager.active_terms)

    @property
    def task(self) -> Task:
        """Return the task whose evaluator semantics drive this environment."""

        return self._task

    def step(self, action: torch.Tensor) -> tuple[VecEnvObs, dict]:
        """Execute one action under the public environment contract."""

        try:
            return super().step(action)
        finally:
            self._step_semantics = (None,) * self.num_envs

    @property
    def step_semantics(self) -> tuple[StepSemantics | None, ...]:
        """Return staged events; None marks slots with no semantic event."""

        return self._step_semantics

    def set_step_semantics(
        self,
        events: Mapping[int, StepSemantics],
    ) -> None:
        """Stage semantic values for exactly the next environment step."""

        resolved_env_ids = resolve_env_ids(tuple(events), self.num_envs)
        staged: list[StepSemantics | None] = [None] * self.num_envs
        for env_id in resolved_env_ids:
            event = events[env_id]
            staged[env_id] = event
        self._step_semantics = tuple(staged)

    def hold_action(self) -> torch.Tensor:
        """Build absolute joint targets that hold every robot at its current pose."""

        targets = []
        for term_name in self.action_manager.active_terms:
            term = self.action_manager.get_term(term_name)
            asset = self.scene[term.cfg.asset_name]
            joint_ids, _ = asset.find_joints(
                term.cfg.joint_names,
                preserve_order=term.cfg.preserve_order,
            )
            targets.append(asset.data.joint_pos.torch[:, joint_ids])
        hold = torch.cat(targets, dim=-1)
        return hold

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
        episode_info = self._task_layout_reset.episode_info(resolved_env_ids)
        info["episode"] = episode_info
        if self.recorder_manager.active_terms:
            for env_id, layout_seed in zip(
                episode_info["env_ids"],
                episode_info["layout_seeds"],
                strict=True,
            ):
                self.recorder_manager.get_episode(env_id).seed = layout_seed
        return observation, info

    def export_episodes(
        self,
        *,
        success: tuple[bool, ...],
        env_ids: Sequence[int] | torch.Tensor | None = None,
        demo_ids: Sequence[str | int] | None = None,
        terminations: Sequence[EpisodeTermination] | None = None,
        step_counts: Sequence[int] | torch.Tensor | None = None,
    ) -> None:
        """Add completion metadata and export recorded episodes before reset."""

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
        if terminations is not None:
            resolved_terminations = tuple(terminations)
            self.recorder_manager.add_to_episodes(
                "termination/reason",
                _encode_episode_text(
                    tuple(
                        termination.reason.value
                        for termination in resolved_terminations
                    ),
                    device=self.device,
                ),
                env_id_tensor,
            )
            self.recorder_manager.add_to_episodes(
                "termination/retryable",
                torch.tensor(
                    tuple(
                        termination.retryable
                        for termination in resolved_terminations
                    ),
                    dtype=torch.bool,
                    device=self.device,
                ),
                env_id_tensor,
            )
            self.recorder_manager.add_to_episodes(
                "termination/message",
                _encode_episode_text(
                    tuple(
                        termination.message for termination in resolved_terminations
                    ),
                    device=self.device,
                ),
                env_id_tensor,
            )
        if step_counts is not None:
            resolved_step_counts = torch.as_tensor(
                step_counts,
                dtype=torch.long,
                device=self.device,
            )
            self.recorder_manager.add_to_episodes(
                "termination/step_count",
                resolved_step_counts,
                env_id_tensor,
            )
        self.recorder_manager.set_success_to_episodes(
            env_id_tensor,
            success_tensor,
        )
        self.recorder_manager.export_episodes(env_id_tensor, demo_ids=demo_ids)

    def discard_episode_buffers(
        self,
        env_ids: Sequence[int] | torch.Tensor,
    ) -> None:
        """Clear unexported recorder data for inactive environments."""

        resolved_env_ids = resolve_env_ids(env_ids, self.num_envs)
        env_id_tensor = torch.tensor(
            resolved_env_ids,
            device=self.device,
            dtype=torch.int32,
        )
        self.recorder_manager.reset(env_id_tensor)

    def close(self) -> None:
        """Close dataset handles deterministically, then release the simulation."""

        try:
            if not self._is_closed and hasattr(self, "recorder_manager"):
                self.recorder_manager.close()
        finally:
            super().close()


def _resolve_success_values(
    success: tuple[bool, ...],
    *,
    count: int,
    device: str,
) -> torch.Tensor:
    """Validate Driver booleans and move them to the recorder device."""

    if any(type(value) is not bool for value in success):
        raise TypeError("success must contain boolean values")
    values = torch.tensor(success, device=device, dtype=torch.bool)
    if values.shape != (count,):
        raise ValueError(
            f"success must contain one value per selected environment ({count}), "
            f"got shape {tuple(values.shape)}"
        )
    return values


def _encode_episode_text(
    values: Sequence[str | None],
    *,
    device: str,
) -> torch.Tensor:
    """Encode per-episode UTF-8 text as zero-padded uint8 rows."""

    raw_values = tuple(
        b"" if value is None else value.encode("utf-8") for value in values
    )
    width = max((len(value) for value in raw_values), default=0)
    encoded = torch.zeros(
        (len(raw_values), max(1, width)),
        dtype=torch.uint8,
        device=device,
    )
    for index, raw in enumerate(raw_values):
        if raw:
            encoded[index, : len(raw)] = torch.tensor(
                tuple(raw),
                dtype=torch.uint8,
                device=device,
            )
    return encoded


__all__ = ["ScaleBenchEnv"]
