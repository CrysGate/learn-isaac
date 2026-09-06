"""Complete pick-and-place skill program."""

from __future__ import annotations

from collections.abc import Iterator

from .commands import Hold, SetGripper, SkillCommand
from .context import SkillContext
from .models import PickAndPlace
from .planner import SkillPlanner


def pick_and_place(
    context: SkillContext,
    planner: SkillPlanner,
    request: PickAndPlace,
) -> Iterator[SkillCommand]:
    """Select one grasp, plan each stage from live state, then place it."""

    yield Hold(steps=1, label="observe")
    plan = planner.plan_pick(request.object_name, request.arm, context)
    yield plan.pre_grasp
    yield plan.grasp
    yield SetGripper(plan.arm, closed=True, label="grasp")
    yield Hold(steps=request.grasp_settle_steps, label="grasped")

    grasp = context.measure_grasp(request.object_name, plan.arm)
    lift = planner.plan_lift(plan, grasp, context)
    yield lift
    yield Hold(steps=request.grasp_settle_steps, label="lifted")

    lifted_grasp = context.measure_grasp(request.object_name, plan.arm)
    pre_place = planner.plan_pre_place(request, plan, lifted_grasp, context)
    yield pre_place

    # Transport can change the object-to-TCP relation through finger slip.
    transported_grasp = context.measure_grasp(request.object_name, plan.arm)
    place = planner.plan_place(request, plan, transported_grasp, context)
    yield place.adjust
    yield place.place
    yield SetGripper(place.arm, closed=False, label="release")
    yield Hold(steps=request.release_settle_steps, label="released")
    yield place.retreat
    yield place.clear


__all__ = ["pick_and_place"]
