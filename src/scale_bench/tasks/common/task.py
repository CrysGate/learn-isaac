"""Common task contract and evaluator observation types."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol, TypeAlias

from torch import Tensor

from .evaluation import EpisodeEvaluatorSpec, EvaluationResult
from .layout import TaskLayout
from .placement import PlacementContext


EvaluatorTerms: TypeAlias = Mapping[str, object]
EvaluatorObservation: TypeAlias = Mapping[str, Tensor]
BatchedEvaluatorObservation: TypeAlias = Mapping[str, Tensor]


class Task(Protocol):
    """Identity, layout behavior, and task-specific evaluation contract."""

    @property
    def task_id(self) -> str: ...

    @property
    def instruction(self) -> str: ...

    @property
    def evaluator_spec(self) -> EpisodeEvaluatorSpec: ...

    def generate_layout(
        self,
        context: PlacementContext,
        seed: int,
    ) -> TaskLayout: ...

    def validate_layout(
        self,
        context: PlacementContext,
        layout: TaskLayout,
    ) -> None: ...

    def build_evaluator_terms(
        self,
        context: PlacementContext,
    ) -> EvaluatorTerms: ...

    def check_success(
        self,
        observation: BatchedEvaluatorObservation,
    ) -> Tensor: ...

    def evaluate(
        self,
        observation: EvaluatorObservation,
    ) -> EvaluationResult: ...


__all__ = [
    "BatchedEvaluatorObservation",
    "EvaluatorObservation",
    "EvaluatorTerms",
    "Task",
]
