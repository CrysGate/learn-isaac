"""Common task contract and evaluator observation types."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol, TypeAlias

from .evaluation import EvaluationResult
from .layout import TaskLayout
from .placement import PlacementContext


EvaluatorTerms: TypeAlias = Mapping[str, object]
EvaluatorObservation: TypeAlias = Mapping[str, object]


class Task(Protocol):
    """Identity, layout behavior, and task-specific evaluation contract."""

    @property
    def task_id(self) -> str: ...

    @property
    def instruction(self) -> str: ...

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

    def evaluate(
        self,
        observation: EvaluatorObservation,
    ) -> EvaluationResult: ...


__all__ = [
    "EvaluatorObservation",
    "EvaluatorTerms",
    "Task",
]
