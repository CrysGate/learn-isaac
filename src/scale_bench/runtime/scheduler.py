"""In-memory fixed-batch benchmark episode scheduling."""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from types import MappingProxyType
from typing import Protocol

from .episodes import EpisodeResult, EpisodeSpec, EpisodeState


class EpisodeStatus(StrEnum):
    """Scheduler-owned lifecycle state for a stable episode ID."""

    PENDING = "pending"
    RUNNING = "running"
    RETRY = "retry"
    COMPLETED = "completed"


class EpisodeBatchRunner(Protocol):
    """Run one fixed batch and return only after every assigned slot finishes."""

    @property
    def num_envs(self) -> int: ...

    def run_batch(
        self,
        states: Sequence[EpisodeState],
    ) -> Mapping[str, EpisodeResult]: ...


@dataclass(frozen=True, slots=True)
class BenchmarkRunResult:
    """Final results and scheduling statistics for one benchmark queue."""

    episodes: Mapping[str, EpisodeResult]
    attempts: Mapping[str, int]
    batch_count: int

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "episodes",
            MappingProxyType(dict(self.episodes)),
        )
        object.__setattr__(
            self,
            "attempts",
            MappingProxyType(dict(self.attempts)),
        )

    @property
    def successful_count(self) -> int:
        return sum(result.success for result in self.episodes.values())

    @property
    def all_succeeded(self) -> bool:
        return self.successful_count == len(self.episodes)


class BenchmarkScheduler:
    """Assign an episode queue to env slots one fixed batch at a time."""

    def __init__(
        self,
        episodes: Sequence[EpisodeSpec],
        *,
        max_retries: int = 0,
    ) -> None:
        specs = tuple(episodes)
        episode_ids = tuple(spec.episode_id for spec in specs)

        self._order = episode_ids
        self._pending = deque(specs)
        self._status = {episode_id: EpisodeStatus.PENDING for episode_id in episode_ids}
        self._attempts = {episode_id: 0 for episode_id in episode_ids}
        self._completed: dict[str, EpisodeResult] = {}
        self._batch_count = 0
        self._max_retries = max_retries

    @property
    def status(self) -> Mapping[str, EpisodeStatus]:
        return MappingProxyType(dict(self._status))

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def run(self, runner: EpisodeBatchRunner) -> BenchmarkRunResult:
        """Drain the queue without assigning a new episode inside a batch."""

        while self._pending:
            specs = tuple(
                self._pending.popleft()
                for _ in range(min(runner.num_envs, len(self._pending)))
            )
            for spec in specs:
                self._status[spec.episode_id] = EpisodeStatus.RUNNING
                self._attempts[spec.episode_id] += 1
            states = tuple(
                EpisodeState(env_id=env_id, spec=spec)
                for env_id, spec in enumerate(specs)
            )

            try:
                batch_results = dict(runner.run_batch(states))
            except Exception:
                for spec in reversed(specs):
                    self._pending.appendleft(spec)
                    self._status[spec.episode_id] = EpisodeStatus.PENDING
                raise

            expected_ids = {spec.episode_id for spec in specs}
            actual_ids = set(batch_results)
            if actual_ids != expected_ids:
                missing = sorted(expected_ids - actual_ids)
                unexpected = sorted(actual_ids - expected_ids)
                raise RuntimeError(
                    "batch runner returned the wrong episode IDs; "
                    f"missing={missing}, unexpected={unexpected}"
                )
            self._batch_count += 1

            for spec in specs:
                result = batch_results[spec.episode_id]
                retry_count = self._attempts[spec.episode_id] - 1
                if result.termination.retryable and retry_count < self._max_retries:
                    self._status[spec.episode_id] = EpisodeStatus.RETRY
                    self._pending.append(spec)
                    continue
                self._status[spec.episode_id] = EpisodeStatus.COMPLETED
                self._completed[spec.episode_id] = result

        ordered_results = {
            episode_id: self._completed[episode_id]
            for episode_id in self._order
        }
        return BenchmarkRunResult(
            episodes=ordered_results,
            attempts=self._attempts,
            batch_count=self._batch_count,
        )


__all__ = [
    "BenchmarkRunResult",
    "BenchmarkScheduler",
    "EpisodeBatchRunner",
    "EpisodeStatus",
]
