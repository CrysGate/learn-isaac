"""Task builder protocol and built-in builder selection."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol

from isaaclab.assets import RigidObjectCfg

from scale_bench.tasks.common.layout import TaskLayout
from scale_bench.tasks.common.task import Task


class TaskBuilder(Protocol):
    """Convert one task layout into fresh native asset cfg objects."""

    def build_assets(
        self,
        task: Task,
        layout: TaskLayout,
    ) -> Mapping[str, RigidObjectCfg]: ...


def resolve_task_builder(
    task: Task,
    explicit_builder: TaskBuilder | None,
) -> TaskBuilder:
    """Select the explicit builder or the built-in rigid-object builder."""

    if explicit_builder is not None:
        return explicit_builder

    from scale_bench.isaaclab.builders.rigid_object_task import RigidObjectTaskBuilder
    from scale_bench.tasks.common.rigid_object import RigidObjectTask

    if isinstance(task, RigidObjectTask):
        return RigidObjectTaskBuilder()
    raise ValueError(
        f"no built-in TaskBuilder is registered for {type(task).__name__}; "
        "pass task_builder explicitly"
    )


__all__ = ["TaskBuilder", "resolve_task_builder"]
