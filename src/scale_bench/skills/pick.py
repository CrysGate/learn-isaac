"""Pick-only skill program."""

from __future__ import annotations

from collections.abc import Iterator

from .commands import Hold, SetGripper, SkillCommand
from .context import SkillContext
from .models import Pick
from .planner import SkillPlanner


def pick(
    context: SkillContext,
    planner: SkillPlanner,
    request: Pick,
) -> Iterator[SkillCommand]:
    """Observe, plan, grasp, settle, and lift one object."""

    yield Hold(steps=1, label="observe")
    plan = planner.plan_pick(request.object_name, request.arm, context)
    yield plan.pre_grasp
    yield plan.grasp
    yield SetGripper(plan.arm, closed=True, label="grasp")
    yield Hold(steps=request.settle_steps, label="grasped")
    grasp = context.measure_grasp(request.object_name, plan.arm)
    lift = planner.plan_lift(plan, grasp, context)
    yield lift
    yield Hold(steps=request.settle_steps, label="lifted")
    context.measure_grasp(request.object_name, plan.arm)


__all__ = ["pick"]
