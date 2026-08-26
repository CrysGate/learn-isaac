"""Stable episode identity, lifecycle, and result value objects."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from scale_bench.tasks.common.evaluation import EvaluationResult
from scale_bench.tasks.common.layout import TaskLayout


class TerminationReason(StrEnum):
    """Why episode execution stopped, independent of task success."""

    GOAL_REACHED = "goal_reached"
    HORIZON_REACHED = "horizon_reached"
    CONTROLLER_FINISHED = "controller_finished"
    SKILL_FAILED = "skill_failed"
    INVALID_ACTION = "invalid_action"
    INVALID_ROBOT_STATE = "invalid_robot_state"
    CANCELLED = "cancelled"
    RUNTIME_ERROR = "runtime_error"


@dataclass(frozen=True, slots=True)
class EpisodeSpec:
    """Immutable input that identifies and configures one episode."""

    episode_id: str
    task_id: str
    seed: int
    layout: TaskLayout
    max_steps: int

    def __post_init__(self) -> None:
        if not self.episode_id.strip():
            raise ValueError("episode_id must not be empty")
        if not self.task_id.strip():
            raise ValueError("task_id must not be empty")
        if self.seed < 0:
            raise ValueError("seed must be non-negative")
        if self.max_steps <= 0:
            raise ValueError("max_steps must be positive")
        if self.layout.task_id != self.task_id:
            raise ValueError(
                "layout task_id must match the episode task_id: "
                f"{self.layout.task_id!r} != {self.task_id!r}"
            )
        if self.layout.seed is not None and self.layout.seed != self.seed:
            raise ValueError(
                "layout seed must match the episode seed: "
                f"{self.layout.seed} != {self.seed}"
            )


@dataclass(slots=True)
class EpisodeState:
    """Mutable per-slot execution state for one scheduled episode."""

    env_id: int
    spec: EpisodeSpec
    step_count: int = 0


@dataclass(frozen=True, slots=True)
class EpisodeTermination:
    """Execution termination metadata kept separate from task evaluation."""

    reason: TerminationReason
    retryable: bool = False
    message: str | None = None


@dataclass(frozen=True, slots=True)
class EpisodeResult:
    """Complete task outcome and execution termination for one episode."""

    spec: EpisodeSpec
    evaluation: EvaluationResult
    termination: EpisodeTermination
    steps: int

    def __post_init__(self) -> None:
        if not 0 <= self.steps <= self.spec.max_steps:
            raise ValueError(
                "steps must be between zero and the episode max_steps "
                f"({self.spec.max_steps}), got {self.steps}"
            )

    @property
    def success(self) -> bool:
        """Return the task result without inferring it from termination."""

        return self.evaluation.success


__all__ = [
    "EpisodeResult",
    "EpisodeSpec",
    "EpisodeState",
    "EpisodeTermination",
    "TerminationReason",
]
