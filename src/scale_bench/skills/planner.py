"""Operation planning over read-only facts and single-segment motion planners."""

from __future__ import annotations

import logging
import math
from collections import Counter
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol

from .commands import MoveToJoints, MoveToPose
from .context import (
    EmptyTool,
    GraspCandidate,
    GraspState,
    HeldObject,
    JointState,
    JointTrajectory,
    PlanningScene,
    SceneObject,
    SceneSnapshot,
    SkillContext,
)
from .errors import PlanningError, SkillError
from .geometry import (
    approach_start_pose,
    compose_pose,
    multiply_quaternions_xyzw,
    normalize_quaternion_xyzw,
    offset_z_env,
    quaternion_angular_distance_rad,
    quaternion_xyzw_from_axis_angle,
    quaternion_xyzw_from_rpy,
    rotate_vector_xyzw,
)
from .models import Arm, ArmSelection, PickAndPlace, Pose

LOGGER = logging.getLogger(__name__)


class MotionPlanner(Protocol):
    """Plan one collision-aware joint trajectory for a fixed arm."""

    @property
    def arm(self) -> Arm: ...

    def plan_pose(
        self,
        start: JointState,
        target_tcp_pose_env: Pose,
        scene: PlanningScene,
    ) -> JointTrajectory: ...

    def plan_joints(
        self,
        start: JointState,
        target_joint_state: JointState,
        scene: PlanningScene,
    ) -> JointTrajectory: ...


@dataclass(frozen=True, slots=True)
class PickPlan:
    object_name: str
    arm: Arm
    candidate: GraspCandidate
    pre_grasp: MoveToPose
    grasp: MoveToPose


@dataclass(frozen=True, slots=True)
class PlacePlan:
    arm: Arm
    pre_place: MoveToPose
    place: MoveToPose
    retreat: MoveToPose
    clear: MoveToJoints


class SkillPlanner(Protocol):
    def plan_pick(
        self,
        object_name: str,
        arm: ArmSelection,
        context: SkillContext,
    ) -> PickPlan: ...

    def plan_lift(
        self,
        plan: PickPlan,
        grasp: GraspState,
        context: SkillContext,
    ) -> MoveToPose: ...

    def plan_place(
        self,
        request: PickAndPlace,
        plan: PickPlan,
        grasp: GraspState,
        context: SkillContext,
    ) -> PlacePlan: ...


class OperationSkillPlanner:
    """Select one grasp and plan each operation stage from live state."""

    def __init__(
        self,
        motion_planners: Mapping[Arm, MotionPlanner],
        arm_base_positions_env_m: Mapping[Arm, tuple[float, float, float]],
        lift_height_m: float,
    ) -> None:
        self._motion_planners = dict(motion_planners)
        self._arm_base_positions_env_m = dict(arm_base_positions_env_m)
        self._lift_height_m = lift_height_m

    def plan_pick(
        self,
        object_name: str,
        arm: ArmSelection,
        context: SkillContext,
    ) -> PickPlan:
        snapshot = context.snapshot()
        source_object = snapshot.object(object_name)
        selected_arm = self._select_arm(arm, source_object)
        try:
            candidates = context.grasp_candidates(object_name, selected_arm)
        except SkillError as error:
            raise SkillError(
                f"could not obtain {selected_arm} grasp candidates for "
                f"{object_name!r}: "
                f"{error}"
            ) from error
        if not candidates:
            raise SkillError(f"{selected_arm} arm has no valid grasp candidate")

        unmanipulated_objects = _objects_excluding(
            snapshot.objects,
            source_object.name,
        )
        pick_transit_scene = _planning_scene(
            snapshot,
            selected_arm,
            snapshot.objects,
            EmptyTool(),
        )  # include the object in the current to pre_grasp stage
        grasp_contact_scene = _planning_scene(
            snapshot,
            selected_arm,
            unmanipulated_objects,
            EmptyTool(),
        )  # exclude the object in the pre_grasp to grasp stage
        failures: list[str] = []
        failure_stage_counts: Counter[str] = Counter()
        attempt_count = 0
        for candidate_index, candidate in enumerate(
            sorted(candidates, key=lambda item: item.score, reverse=True)
        ):
            for symmetry_index, grasp_tcp_pose_env in enumerate(
                _parallel_jaw_grasp_poses(
                    source_object.pose_env,
                    candidate,
                    snapshot.robot(selected_arm).tcp_pose_env.orientation_xyzw,
                )
            ):
                symmetry_variant = (
                    "smaller_wrist_rotation",
                    "larger_wrist_rotation",
                )[symmetry_index]
                symmetry_label = ("small", "large")[symmetry_index]
                pre_grasp_tcp_pose_env = approach_start_pose(
                    grasp_tcp_pose_env,
                    candidate.approach_axis_tcp,
                    candidate.approach_distance_m,
                )
                attempt_count += 1
                try:
                    pre_grasp = self._move(
                        selected_arm,
                        snapshot.robot(selected_arm).joints,
                        pre_grasp_tcp_pose_env,
                        pick_transit_scene,
                        "pre_grasp",
                    )
                    grasp = self._move(
                        selected_arm,
                        pre_grasp.trajectory.end,
                        grasp_tcp_pose_env,
                        grasp_contact_scene,
                        "grasp",
                    )
                except PlanningError as error:
                    failures.append(f"{error.stage}: {error.reason}")
                    failure_stage_counts[error.stage] += 1
                    LOGGER.debug(
                        "candidate=%d symmetry=%s score=%.4f stage=%s rejected: %s",
                        candidate_index,
                        symmetry_label,
                        candidate.score,
                        error.stage,
                        error.reason,
                        extra={
                            "event": "PLAN-TRY",
                            "event_fields": {
                                "object": object_name,
                                "arm": selected_arm,
                                "candidate_index": candidate_index,
                                "symmetry_variant": symmetry_variant,
                                "score": candidate.score,
                                "stage": error.stage,
                                "reason": error.reason,
                            },
                        },
                    )
                    continue

                arm_base_position_env_m = self._arm_base_positions_env_m[selected_arm]
                base_distance_m = math.sqrt(
                    sum(
                        (tcp_coordinate_env_m - base_coordinate_env_m) ** 2
                        for tcp_coordinate_env_m, base_coordinate_env_m in zip(
                            grasp_tcp_pose_env.position_m,
                            arm_base_position_env_m,
                            strict=True,
                        )
                    )
                )
                tcp_position_env_m = tuple(
                    round(value, 4) for value in grasp_tcp_pose_env.position_m
                )
                LOGGER.info(
                    "selected #%d/%s score=%.3f tries=%d",
                    candidate_index,
                    symmetry_label,
                    candidate.score,
                    attempt_count,
                    extra={
                        "event": "PLAN",
                        "event_fields": {
                            "object": object_name,
                            "arm": selected_arm,
                            "candidate_count": len(candidates),
                            "candidate_index": candidate_index,
                            "symmetry_variant": symmetry_variant,
                            "score": candidate.score,
                            "attempt_count": attempt_count,
                            "failure_stage_counts": dict(failure_stage_counts),
                            "base_distance_m": base_distance_m,
                            "tcp_position_env_m": tcp_position_env_m,
                        },
                    },
                )
                return PickPlan(
                    source_object.name,
                    selected_arm,
                    candidate,
                    pre_grasp,
                    grasp,
                )

        LOGGER.warning(
            "unreachable candidates=%d tries=%d",
            len(candidates),
            attempt_count,
            extra={
                "event": "PLAN",
                "event_fields": {
                    "object": object_name,
                    "arm": selected_arm,
                    "candidate_count": len(candidates),
                    "attempt_count": attempt_count,
                    "failure_stage_counts": dict(failure_stage_counts),
                    "last_failure": failures[-1],
                },
            },
        )
        raise SkillError(
            f"{selected_arm} arm could not reach any of {len(candidates)} valid "
            f"{object_name!r} grasps; last failure: {failures[-1]}"
        )

    def plan_lift(
        self,
        plan: PickPlan,
        grasp: GraspState,
        context: SkillContext,
    ) -> MoveToPose:
        snapshot = context.snapshot()
        source_object = snapshot.object(plan.object_name)
        lift_tcp_pose_env = Pose(
            offset_z_env(grasp.tcp_pose_env.position_m, self._lift_height_m),
            grasp.tcp_pose_env.orientation_xyzw,
        )
        unmanipulated_objects = _objects_excluding(
            snapshot.objects,
            source_object.name,
        )
        held_object_scene = _planning_scene(
            snapshot,
            plan.arm,
            unmanipulated_objects,
            HeldObject(source_object, grasp.tcp_pose_object),
        )
        return self._move(
            plan.arm,
            snapshot.robot(plan.arm).joints,
            lift_tcp_pose_env,
            held_object_scene,
            "lift",
        )

    def plan_place(
        self,
        request: PickAndPlace,
        plan: PickPlan,
        grasp: GraspState,
        context: SkillContext,
    ) -> PlacePlan:
        snapshot = context.snapshot()
        source_object = snapshot.object(request.object_name)
        target_object_orientations_env_xyzw = _target_object_orientations_env_xyzw(
            request,
            grasp.tcp_pose_object,
            grasp.tcp_pose_env.orientation_xyzw,
        )
        failures: list[str] = []
        for target_object_orientation_env_xyzw in target_object_orientations_env_xyzw:
            try:
                return self._plan_measured_place(
                    plan.arm,
                    source_object,
                    grasp,
                    plan.candidate.approach_axis_tcp,
                    plan.candidate.approach_distance_m,
                    target_object_orientation_env_xyzw,
                    request.target_object_pose_env.position_m,
                    snapshot,
                )
            except PlanningError as error:
                failures.append(f"{error.stage}: {error.reason}")
        raise SkillError(
            f"actual-grasp place planning failed for {request.object_name!r}: "
            + "; ".join(failures)
        )

    def _plan_measured_place(
        self,
        arm: Arm,
        source_object: SceneObject,
        grasp: GraspState,
        approach_axis_tcp: tuple[float, float, float],
        approach_distance_m: float,
        target_object_orientation_env_xyzw: tuple[float, float, float, float],
        target_object_position_env_m: tuple[float, float, float],
        snapshot: SceneSnapshot,
    ) -> PlacePlan:
        target_object_pose_env = Pose(
            target_object_position_env_m,
            target_object_orientation_env_xyzw,
        )
        place_tcp_pose_env = compose_pose(
            target_object_pose_env,
            grasp.tcp_pose_object,
        )
        pre_place_tcp_pose_env = approach_start_pose(
            place_tcp_pose_env,
            approach_axis_tcp,
            approach_distance_m,
        )
        unmanipulated_objects = _objects_excluding(
            snapshot.objects,
            source_object.name,
        )
        held_object_scene = _planning_scene(
            snapshot,
            arm,
            unmanipulated_objects,
            HeldObject(source_object, grasp.tcp_pose_object),
        )
        pre_place = self._move(
            arm,
            snapshot.robot(arm).joints,
            pre_place_tcp_pose_env,
            held_object_scene,
            "pre_place",
        )
        place = self._move(
            arm,
            pre_place.trajectory.end,
            place_tcp_pose_env,
            held_object_scene,
            "place",
        )
        placed_object = SceneObject(
            source_object.name,
            target_object_pose_env,
            source_object.size_m,
        )
        released_object_scene = _planning_scene(
            snapshot,
            arm,
            (*unmanipulated_objects, placed_object),
            EmptyTool(),
        )
        release_contact_scene = _planning_scene(
            snapshot,
            arm,
            unmanipulated_objects,
            EmptyTool(),
        )
        retreat = self._move(
            arm,
            place.trajectory.end,
            pre_place_tcp_pose_env,
            release_contact_scene,
            "retreat",
        )
        clear_target_joint_state = JointState(
            retreat.trajectory.end.positions.new_zeros(
                retreat.trajectory.end.positions.shape
            )
        )
        clear = self._move_joints(
            arm,
            retreat.trajectory.end,
            clear_target_joint_state,
            released_object_scene,
            "clear",
        )
        return PlacePlan(arm, pre_place, place, retreat, clear)

    def _move(
        self,
        arm: Arm,
        start: JointState,
        target_tcp_pose_env: Pose,
        scene: PlanningScene,
        stage: str,
    ) -> MoveToPose:
        try:
            trajectory = self._motion_planners[arm].plan_pose(
                start,
                target_tcp_pose_env,
                scene,
            )
        except PlanningError as error:
            raise PlanningError(arm, stage, error.reason) from error
        return MoveToPose(arm, target_tcp_pose_env, trajectory, stage)

    def _move_joints(
        self,
        arm: Arm,
        start: JointState,
        target_joint_state: JointState,
        scene: PlanningScene,
        stage: str,
    ) -> MoveToJoints:
        try:
            trajectory = self._motion_planners[arm].plan_joints(
                start,
                target_joint_state,
                scene,
            )
        except PlanningError as error:
            raise PlanningError(arm, stage, error.reason) from error
        return MoveToJoints(arm, target_joint_state, trajectory, stage)

    def _select_arm(
        self,
        arm_selection: ArmSelection,
        source_object: SceneObject,
    ) -> Arm:
        if arm_selection != "auto":
            return arm_selection

        arms: tuple[Arm, Arm] = ("left", "right")

        def distance_to_object(arm: Arm) -> float:
            arm_base_position_env_m = self._arm_base_positions_env_m[arm]
            return sum(
                (object_coordinate_env_m - base_coordinate_env_m) ** 2
                for object_coordinate_env_m, base_coordinate_env_m in zip(
                    source_object.pose_env.position_m,
                    arm_base_position_env_m,
                    strict=True,
                )
            )

        return min(arms, key=distance_to_object)


def _planning_scene(
    snapshot: SceneSnapshot,
    active_arm: Arm,
    objects: tuple[SceneObject, ...],
    tool: EmptyTool | HeldObject,
) -> PlanningScene:
    other_arm: Arm = "right" if active_arm == "left" else "left"
    return PlanningScene(
        table=snapshot.table,
        objects=objects,
        other_arm=other_arm,
        other_robot=snapshot.robot(other_arm),
        tool=tool,
    )


def _objects_excluding(
    objects: tuple[SceneObject, ...],
    object_name: str,
) -> tuple[SceneObject, ...]:
    return tuple(
        scene_object for scene_object in objects if scene_object.name != object_name
    )


def _parallel_jaw_grasp_poses(
    object_pose_env: Pose,
    candidate: GraspCandidate,
    reference_tcp_orientation_env_xyzw: tuple[float, float, float, float],
) -> tuple[Pose, Pose]:
    """Prefer the equivalent pose requiring less rotation from the live TCP."""

    canonical_tcp_pose_env = compose_pose(
        object_pose_env,
        candidate.tcp_pose_object,
    )
    half_turn_orientation_tcp_xyzw = quaternion_xyzw_from_axis_angle(
        candidate.approach_axis_tcp,
        math.pi,
    )
    alternate_tcp_pose_env = Pose(
        canonical_tcp_pose_env.position_m,
        normalize_quaternion_xyzw(
            multiply_quaternions_xyzw(
                canonical_tcp_pose_env.orientation_xyzw,
                half_turn_orientation_tcp_xyzw,
            )
        ),
    )
    canonical_rotation_from_reference_rad = quaternion_angular_distance_rad(
        reference_tcp_orientation_env_xyzw,
        canonical_tcp_pose_env.orientation_xyzw,
    )
    alternate_rotation_from_reference_rad = quaternion_angular_distance_rad(
        reference_tcp_orientation_env_xyzw,
        alternate_tcp_pose_env.orientation_xyzw,
    )
    if canonical_rotation_from_reference_rad <= alternate_rotation_from_reference_rad:
        return canonical_tcp_pose_env, alternate_tcp_pose_env
    return alternate_tcp_pose_env, canonical_tcp_pose_env


def _target_object_orientations_env_xyzw(
    request: PickAndPlace,
    tcp_pose_object: Pose,
    reference_tcp_orientation_env_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float, float], ...]:
    target_tcp_pose_env = compose_pose(
        request.target_object_pose_env,
        tcp_pose_object,
    )
    target_finger_open_axis_env = rotate_vector_xyzw(
        target_tcp_pose_env.orientation_xyzw,
        (0.0, 1.0, 0.0),
    )
    reference_finger_open_axis_env = rotate_vector_xyzw(
        reference_tcp_orientation_env_xyzw,
        (0.0, 1.0, 0.0),
    )
    alignment_yaw_env_rad = math.atan2(
        reference_finger_open_axis_env[1],
        reference_finger_open_axis_env[0],
    ) - math.atan2(
        target_finger_open_axis_env[1],
        target_finger_open_axis_env[0],
    )
    aligned_object_orientation_env_xyzw = normalize_quaternion_xyzw(
        multiply_quaternions_xyzw(
            quaternion_xyzw_from_rpy(0.0, 0.0, alignment_yaw_env_rad),
            request.target_object_pose_env.orientation_xyzw,
        )
    )
    yaw_offsets_env_rad = (
        0.0,
        math.pi / 4.0,
        -math.pi / 4.0,
        math.pi / 2.0,
        -math.pi / 2.0,
        3.0 * math.pi / 4.0,
        -3.0 * math.pi / 4.0,
        math.pi,
    )
    return tuple(
        normalize_quaternion_xyzw(
            multiply_quaternions_xyzw(
                quaternion_xyzw_from_rpy(0.0, 0.0, yaw_offset_env_rad),
                aligned_object_orientation_env_xyzw,
            )
        )
        for yaw_offset_env_rad in yaw_offsets_env_rad
    )


__all__ = [
    "MotionPlanner",
    "OperationSkillPlanner",
    "PickPlan",
    "PlacePlan",
    "SkillPlanner",
]
