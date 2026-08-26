"""Replay recorded actions from an exact initial state and re-evaluate them."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol

import torch
from torch import Tensor

from scale_bench.tasks.common.evaluation import EvaluationResult
from scale_bench.tasks.common.layout import TaskLayout
from scale_bench.tasks.common.task import Task

from .evaluator import TaskEpisodeEvaluator


class ReplayEnvironment(Protocol):
    """Exact-state operations needed by :class:`EpisodeReplayRunner`."""

    num_envs: int
    device: str

    @property
    def task(self) -> Task: ...

    def hold_action(self) -> Tensor: ...

    def reset_to(
        self,
        state: Mapping[str, object],
        env_ids: Tensor,
        *,
        seed: int | None = None,
        is_relative: bool = False,
    ) -> tuple[object, dict]: ...
    """ManagerBasedEnv.reset_to()"""

    def step(self, action: Tensor) -> tuple[object, dict]: ...


@dataclass(frozen=True, slots=True)
class RecordedEpisode:
    """One native Isaac Lab episode plus its deterministic identity."""

    episode_name: str
    seed: int
    layout: TaskLayout
    initial_state: Mapping[str, object]
    actions: Tensor # shape (steps, action_dim)
    recorded_steps: int
    recorded_success: bool

    def __post_init__(self) -> None:
        if not self.episode_name:
            raise ValueError("episode_name must not be empty")
        if self.seed < 0:
            raise ValueError("seed must be non-negative")
        if self.layout.seed != self.seed:
            raise ValueError("layout seed must match the recorded episode seed")
        if not self.initial_state:
            raise ValueError("initial_state must not be empty")
        if self.actions.ndim != 2:
            raise ValueError("actions must have shape (steps, action_dim)")
        if self.recorded_steps < 0:
            raise ValueError("recorded_steps must be non-negative")
        if type(self.recorded_success) is not bool:
            raise TypeError("recorded_success must be a bool")


@dataclass(frozen=True, slots=True)
class EpisodeReplayResult:
    """Recorded-versus-replayed invariants and the fresh task evaluation."""

    episode_name: str
    seed: int
    layout_matches: bool
    recorded_steps: int
    replayed_steps: int
    recorded_success: bool
    evaluation: EvaluationResult

    @property
    def consistent(self) -> bool:
        return (
            self.layout_matches
            and self.replayed_steps == self.recorded_steps
            and self.evaluation.success == self.recorded_success
        )


class EpisodeReplayRunner:
    """Replay every recorded frame without early termination or re-recording."""

    def __init__(self, env: ReplayEnvironment) -> None:
        """Replay recorded hdf5 episode, support only 1 episode at a time."""
        self._env = env
        self._evaluator = TaskEpisodeEvaluator(
            env.task,
            num_envs=1,
            device=env.device,
        )

    def run(self, episode: RecordedEpisode) -> EpisodeReplayResult:
        env_ids = torch.tensor([0], dtype=torch.int32, device=self._env.device)
        observation, _ = self._env.reset_to(
            episode.initial_state,
            env_ids,
            seed=episode.seed,
            is_relative=True,
        )
        self._evaluator.reset(env_ids)

        replayed_steps = 0
        success_verification_mask = torch.ones(
            1,
            dtype=torch.bool,
            device=self._env.device,
        )
        for action in episode.actions:
            observation, _ = self._env.step(action.unsqueeze(0))
            self._evaluator.update(
                _evaluator_observation(observation),
                success_verification_mask,
            )
            replayed_steps += 1

        evaluation = self._evaluator.finalize(
            env_ids,
            _evaluator_observation(observation),
        )[0]
        return EpisodeReplayResult(
            episode_name=episode.episode_name,
            seed=episode.seed,
            layout_matches=initial_state_matches_layout(
                episode.initial_state,
                episode.layout,
            ),
            recorded_steps=episode.recorded_steps,
            replayed_steps=replayed_steps,
            recorded_success=episode.recorded_success,
            evaluation=evaluation,
        )


def initial_state_matches_layout(
    initial_state: Mapping[str, object],
    layout: TaskLayout,
) -> bool:
    """Check exact recorded object poses against a regenerated task layout."""

    absolute_tolerance: float = 1.0e-5
    rigid_objects = initial_state.get("rigid_object")
    for object_name, placement in layout.assets.items():
        state = rigid_objects.get(object_name)
        root_pose = state.get("root_pose")
        expected = root_pose.new_tensor(
            (*placement.position_m, *placement.orientation_xyzw)
        )
        actual = root_pose[0]
        if not torch.allclose(
            actual[:3],
            expected[:3],
            atol=absolute_tolerance,
            rtol=0.0,
        ):
            return False
        quaternion_matches = torch.allclose(
            actual[3:],
            expected[3:],
            atol=absolute_tolerance,
            rtol=0.0,
        ) or torch.allclose(
            actual[3:],
            -expected[3:],
            atol=absolute_tolerance,
            rtol=0.0,
        )
        if not quaternion_matches:
            return False
    return True


def _evaluator_observation(observation: object) -> Mapping[str, Tensor]:
    evaluator_observation = observation.get("evaluator")
    return evaluator_observation


__all__ = [
    "EpisodeReplayResult",
    "EpisodeReplayRunner",
    "RecordedEpisode",
    "ReplayEnvironment",
]
