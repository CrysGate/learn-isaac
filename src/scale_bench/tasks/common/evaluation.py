"""Simulator-independent task evaluation result values."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping


@dataclass(frozen=True, slots=True)
class EpisodeEvaluatorSpec:
    """Task-owned stateful success semantics."""

    success_stability_steps: int = 1

    def __post_init__(self) -> None:
        if self.success_stability_steps <= 0:
            raise ValueError("success_stability_steps must be positive")


@dataclass(frozen=True, slots=True)
class EvaluationResult:
    """Success, progress, and diagnostics for one environment."""

    success: bool
    progress: float
    metrics: Mapping[str, float] = field(default_factory=dict)
    failure_reason: str | None = None

    def __post_init__(self) -> None:
        if type(self.success) is not bool:
            raise TypeError("success must be a bool")
        if not math.isfinite(self.progress) or not 0.0 <= self.progress <= 1.0:
            raise ValueError("progress must be a finite value between 0 and 1")
        if any(not name for name in self.metrics):
            raise ValueError("metric names must not be empty")
        if any(not math.isfinite(value) for value in self.metrics.values()):
            raise ValueError("metrics must contain only finite values")
        if self.success and self.failure_reason is not None:
            raise ValueError("a successful evaluation cannot have a failure reason")
        object.__setattr__(self, "metrics", MappingProxyType(dict(self.metrics)))


__all__ = ["EpisodeEvaluatorSpec", "EvaluationResult"]
