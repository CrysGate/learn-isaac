"""Common fixed-batch episode stepping and finalization."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Protocol

import torch
from torch import Tensor

from scale_bench.tasks.common.layout import TaskLayout
from scale_bench.tasks.common.task import BatchedEvaluatorObservation, Task

from .evaluator import TaskEpisodeEvaluator
from .episodes import (
    EpisodeResult,
    EpisodeState,
    EpisodeTermination,
    TerminationReason,
)
from .recording import StepSemantics


class DriverEnvironment(Protocol):
    """The simulator operations used by :class:`EpisodeDriver`."""

    num_envs: int
    device: str

    @property
    def task(self) -> Task: ...

    @property
    def recording_enabled(self) -> bool: ...

    def reset(
        self,
        *,
        env_ids: Sequence[int] | Tensor | None = None,
        task_layouts: Sequence[TaskLayout] | None = None,
    ) -> tuple[object, dict]: ...

    def step(self, action: Tensor) -> tuple[object, dict]: ...

    def export_episodes(
        self,
        *,
        success: tuple[bool, ...],
        env_ids: Sequence[int] | Tensor | None = None,
        demo_ids: Sequence[str | int] | None = None,
        terminations: Sequence[EpisodeTermination] | None = None,
        step_counts: Sequence[int] | Tensor | None = None,
    ) -> None: ...

    def discard_episode_buffers(
        self,
        env_ids: Sequence[int] | Tensor,
    ) -> None: ...

    def set_step_semantics(
        self,
        events: Mapping[int, StepSemantics],
    ) -> None: ...


@dataclass(frozen=True, slots=True)
class DriverSnapshot:
    """Observable driver state returned after reset or one batched step."""

    observation: object
    active_mask: Tensor
    completed: Mapping[str, EpisodeResult]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "completed",
            MappingProxyType(dict(self.completed)),
        )


class EpisodeDriver:
    """Own one reset-to-finish lifecycle for a fixed set of env slots."""

    def __init__(self, env: DriverEnvironment) -> None:
        self._env = env
        self._evaluator = TaskEpisodeEvaluator(
            env.task,
            num_envs=env.num_envs,
            device=env.device,
        )
        self._states: dict[int, EpisodeState] = {}
        self._results: dict[str, EpisodeResult] = {}
        self._active_mask = torch.zeros(
            env.num_envs,
            dtype=torch.bool,
            device=env.device,
        )
        self._step_counts = torch.zeros(
            env.num_envs,
            dtype=torch.long,
            device=env.device,
        )
        self._max_steps = torch.zeros(
            env.num_envs,
            dtype=torch.long,
            device=env.device,
        )
        self._observation: object | None = None

    @property
    def is_active(self) -> bool:
        return bool(self._active_mask.any().item())

    @property
    def results(self) -> Mapping[str, EpisodeResult]:
        return MappingProxyType(dict(self._results))

    def start(self, states: Sequence[EpisodeState]) -> DriverSnapshot:
        """Reset the assigned slots and start one fixed episode per slot."""

        resolved_states = tuple(sorted(states, key=lambda state: state.env_id))
        env_ids = tuple(state.env_id for state in resolved_states)

        self._states = {state.env_id: state for state in resolved_states}
        self._results = {}
        env_id_tensor = torch.tensor(
            env_ids,
            dtype=torch.long,
            device=self._env.device,
        )
        self._active_mask.zero_()
        self._active_mask[env_id_tensor] = True
        self._step_counts[env_id_tensor] = 0
        self._max_steps[env_id_tensor] = torch.tensor(
            tuple(state.spec.max_steps for state in resolved_states),
            dtype=torch.long,
            device=self._env.device,
        )
        for state in resolved_states:
            state.step_count = 0

        observation, _ = self._env.reset(
            env_ids=env_ids,
            task_layouts=tuple(state.spec.layout for state in resolved_states),
        )
        self._observation = observation

        self._evaluator.reset(env_id_tensor)
        return self._snapshot({})

    def step(
        self,
        action: Tensor,
        *,
        success_verification_mask: Tensor,
        terminations: Mapping[int, EpisodeTermination] | None = None,
        semantic_events: Mapping[int, StepSemantics] | None = None,
    ) -> DriverSnapshot:
        """Step once and count success only in explicitly eligible slots.

        Policy rollouts make every active slot eligible. Expert rollouts keep
        slots ineligible until their final success-verification hold begins.
        """

        completed = self._finalize(dict(terminations or {}))
        if not self.is_active:
            return self._snapshot(completed)

        stepped_mask = self._active_mask.clone()
        self._env.set_step_semantics(semantic_events or {})
        try:
            observation, _ = self._env.step(action)
        except Exception:
            self.abort()
            raise
        self._observation = observation
        self._step_counts += stepped_mask.to(dtype=torch.long)

        inactive_env_ids = torch.nonzero(
            ~self._active_mask,
            as_tuple=False,
        ).flatten()
        if inactive_env_ids.numel() and self._env.recording_enabled:
            self._env.discard_episode_buffers(inactive_env_ids)

        success = self._evaluator.update(
            _evaluator_observation(observation),
            success_verification_mask,
        )
        goal_mask = self._active_mask & success
        horizon_mask = (
            self._active_mask
            & ~goal_mask
            & (self._step_counts >= self._max_steps)
        )
        finished_env_ids = torch.nonzero(
            goal_mask | horizon_mask,
            as_tuple=False,
        ).flatten().detach().cpu().tolist()
        automatic_terminations = {}
        for env_id in finished_env_ids:
            if goal_mask[env_id].item():
                automatic_terminations[env_id] = EpisodeTermination(
                    TerminationReason.GOAL_REACHED
                )
            else:
                automatic_terminations[env_id] = EpisodeTermination(
                    TerminationReason.HORIZON_REACHED
                )
        completed.update(self._finalize(automatic_terminations))
        return self._snapshot(completed)

    def abort(self) -> None:
        """Discard all active recorder transactions after an exception/cancel."""

        active_env_ids = torch.nonzero(
            self._active_mask,
            as_tuple=False,
        ).flatten()
        if active_env_ids.numel() and self._env.recording_enabled:
            self._env.discard_episode_buffers(active_env_ids)
        if active_env_ids.numel():
            self._active_mask[active_env_ids] = False

    def _finalize(
        self,
        terminations: Mapping[int, EpisodeTermination],
    ) -> dict[str, EpisodeResult]:
        if not terminations:
            return {}
        env_ids = tuple(sorted(terminations))
        env_id_tensor = torch.tensor(
            env_ids,
            dtype=torch.long,
            device=self._env.device,
        )
        evaluations = self._evaluator.finalize(
            env_id_tensor,
            _evaluator_observation(self._observation),
        )
        if env_ids and self._env.recording_enabled:
            self._env.export_episodes(
                env_ids=env_ids,
                success=tuple(evaluations[env_id].success for env_id in env_ids),
                demo_ids=tuple(
                    self._states[env_id].spec.episode_id
                    for env_id in env_ids
                ),
                terminations=tuple(terminations[env_id] for env_id in env_ids),
                step_counts=tuple(
                    int(self._step_counts[env_id].item()) for env_id in env_ids
                ),
            )

        completed = {}
        for env_id in env_ids:
            state = self._states[env_id]
            state.step_count = int(self._step_counts[env_id].item())
            result = EpisodeResult(
                spec=state.spec,
                evaluation=evaluations[env_id],
                termination=terminations[env_id],
                steps=state.step_count,
            )
            completed[state.spec.episode_id] = result
            self._results[state.spec.episode_id] = result
        self._active_mask[env_id_tensor] = False
        return completed

    def _snapshot(self, completed: Mapping[str, EpisodeResult]) -> DriverSnapshot:
        return DriverSnapshot(
            observation=self._observation,
            active_mask=self._active_mask.clone(),
            completed=completed,
        )


def _evaluator_observation(
    observation: object,
) -> BatchedEvaluatorObservation:
    evaluator_observation = observation.get("evaluator")
    return evaluator_observation


__all__ = [
    "DriverEnvironment",
    "DriverSnapshot",
    "EpisodeDriver",
]
