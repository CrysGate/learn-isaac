"""Common simulator-independent task contract."""

from __future__ import annotations

from typing import Protocol

from .layout import TaskLayout
from .placement import PlacementContext


class Task(Protocol):
    """Identity, instruction, and deterministic layout behavior for a task."""

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


__all__ = ["Task"]
