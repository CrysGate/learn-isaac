"""Operation planning over read-only facts and single-segment motion planners."""

from __future__ import annotations

import logging
import math
from collections import Counter
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Literal, Protocol, TypeAlias

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
    quaternion_xyzw_from_axis_angle,
    quaternion_xyzw_from_rpy,
    relative_pose,
    rotate_vector_xyzw,
)
from .models import Arm, ArmSelection, PickAndPlace, Pose

LOGGER = logging.getLogger(__name__)

PlanningStage: TypeAlias = Literal[
    "pre_grasp",
    "grasp",
    "lift",
    "pre_place",
    "adjust",
    "place",
    "retreat",
    "clear",
]


class MotionPlanner(Protocol):
    """Plan one collision-aware joint trajectory for a fixed arm."""

    @property
    def arm(self) -> Arm: ...

    def plan_pose(
        self,
        start: JointState,
        target_tcp_pose_env: Pose,
        scene: PlanningScene,
        stage: PlanningStage,
        linear_axis_env: tuple[float, float, float] | None,
    ) -> JointTrajectory:
        """An axis fixes contact motion; None allows transit and adjustment."""
        ...

    def plan_joints(
        self,
        start: JointState,
        target_joint_state: JointState,
        scene: PlanningScene,
        stage: PlanningStage,
    ) -> JointTrajectory: ...

    def commit_inspection_stages(
        self,
        stages: tuple[PlanningStage, ...],
    ) -> None: ...


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
    adjust: MoveToPose
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

    def plan_pre_place(
        self,
        request: PickAndPlace,
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
            for grasp_tcp_pose_env in _parallel_jaw_grasp_poses(
                source_object.pose_env,
                candidate,
            ):
                pre_grasp_tcp_pose_env = approach_start_pose(
                    grasp_tcp_pose_env,
                    candidate.approach_axis_tcp,
                    candidate.approach_distance_m,
                )
                attempt_count += 1
                try:
                    camera_side_up_dot = _camera_side_up_dot(
                        grasp_tcp_pose_env,
                        snapshot.robot(selected_arm).camera_position_tcp_m,
                        candidate.approach_axis_tcp,
                    )
                    if camera_side_up_dot <= 0.0:
                        continue
                    pre_grasp = self._move(
                        selected_arm,
                        snapshot.robot(selected_arm).joints,
                        pre_grasp_tcp_pose_env,
                        pick_transit_scene,
                        "pre_grasp",
                        None,
                    )
                    grasp = self._move(
                        selected_arm,
                        pre_grasp.trajectory.end,
                        grasp_tcp_pose_env,
                        grasp_contact_scene,
                        "grasp",
                        rotate_vector_xyzw(
                            grasp_tcp_pose_env.orientation_xyzw,
                            candidate.approach_axis_tcp,
                        ),
                    )
                    held_object_scene = _planning_scene(
                        snapshot,
                        selected_arm,
                        unmanipulated_objects,
                        HeldObject(
                            source_object,
                            relative_pose(source_object.pose_env, grasp_tcp_pose_env),
                        ),
                    )
                    # Reject grasps that cannot lift; execution replans from live state.
                    self._lift_from_state(
                        selected_arm,
                        grasp.trajectory.end,
                        grasp_tcp_pose_env,
                        held_object_scene,
                    )
                except PlanningError as error:
                    failures.append(f"{error.stage}: {error.reason}")
                    failure_stage_counts[error.stage] += 1
                    LOGGER.debug(
                        "candidate=%d score=%.4f stage=%s rejected: %s",
                        candidate_index,
                        candidate.score,
                        error.stage,
                        error.reason,
                        extra={
                            "event": "PLAN-TRY",
                            "event_fields": {
                                "object": object_name,
                                "arm": selected_arm,
                                "candidate_index": candidate_index,
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
                    "selected #%d score=%.3f tries=%d",
                    candidate_index,
                    candidate.score,
                    attempt_count,
                    extra={
                        "event": "PLAN",
                        "event_fields": {
                            "object": object_name,
                            "arm": selected_arm,
                            "candidate_count": len(candidates),
                            "candidate_index": candidate_index,
                            "score": candidate.score,
                            "attempt_count": attempt_count,
                            "failure_stage_counts": dict(failure_stage_counts),
                            "base_distance_m": base_distance_m,
                            "tcp_position_env_m": tcp_position_env_m,
                            "camera_side_up_dot": camera_side_up_dot,
                        },
                    },
                )
                self._motion_planners[selected_arm].commit_inspection_stages(
                    ("pre_grasp", "grasp")
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
            f"{selected_arm} arm could not plan an upright grasp and lift for any of "
            f"{len(candidates)} valid {object_name!r} candidates; "
            f"last failure: {failures[-1]}"
        )

    def plan_lift(
        self,
        plan: PickPlan,
        grasp: GraspState,
        context: SkillContext,
    ) -> MoveToPose:
        snapshot = context.snapshot()
        source_object = snapshot.object(plan.object_name)
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
        lift = self._lift_from_state(
            plan.arm,
            snapshot.robot(plan.arm).joints,
            grasp.tcp_pose_env,
            held_object_scene,
        )
        self._motion_planners[plan.arm].commit_inspection_stages(("lift",))
        return lift

    def _lift_from_state(
        self,
        arm: Arm,
        joint_state: JointState,
        tcp_pose_env: Pose,
        scene: PlanningScene,
    ) -> MoveToPose:
        lift_tcp_pose_env = Pose(
            offset_z_env(tcp_pose_env.position_m, self._lift_height_m),
            tcp_pose_env.orientation_xyzw,
        )
        return self._move(
            arm,
            joint_state,
            lift_tcp_pose_env,
            scene,
            "lift",
            (0.0, 0.0, 1.0),
        )

    def plan_pre_place(
        self,
        request: PickAndPlace,
        plan: PickPlan,
        grasp: GraspState,
        context: SkillContext,
    ) -> MoveToPose:
        """Plan transport; placement uses a fresh grasp measurement on arrival."""
        snapshot = context.snapshot()
        source_object = snapshot.object(request.object_name)
        held_object_scene = _planning_scene(
            snapshot,
            plan.arm,
            _objects_excluding(snapshot.objects, source_object.name),
            HeldObject(source_object, grasp.tcp_pose_object),
        )
        target_object_orientations_env_xyzw = _target_object_orientations_env_xyzw(
            request,
            grasp.tcp_pose_object,
            grasp.tcp_pose_env.orientation_xyzw,
        )
        failures: list[str] = []
        for target_object_orientation_env_xyzw in target_object_orientations_env_xyzw:
            target_object_pose_env = Pose(
                request.target_object_pose_env.position_m,
                target_object_orientation_env_xyzw,
            )
            place_tcp_pose_env = compose_pose(target_object_pose_env, grasp.tcp_pose_object)
            try:
                pre_place = self._move_above_place(
                    plan.arm,
                    snapshot.robot(plan.arm).joints,
                    place_tcp_pose_env,
                    held_object_scene,
                    "pre_place",
                )
            except PlanningError as error:
                failures.append(f"{error.stage}: {error.reason}")
                continue
            self._motion_planners[plan.arm].commit_inspection_stages(("pre_place",))
            return pre_place
        raise SkillError(
            f"actual-grasp pre-place planning failed for {request.object_name!r}: "
            + "; ".join(failures)
        )

    def _move_above_place(
        self,
        arm: Arm,
        joint_state: JointState,
        place_tcp_pose_env: Pose,
        scene: PlanningScene,
        stage: Literal["pre_place", "adjust"],
    ) -> MoveToPose:
        above_place_tcp_pose_env = Pose(
            offset_z_env(place_tcp_pose_env.position_m, self._lift_height_m),
            place_tcp_pose_env.orientation_xyzw,
        )
        return self._move(
            arm,
            joint_state,
            above_place_tcp_pose_env,
            scene,
            stage,
            None,
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
        # The gripper stays closed through place; EmptyTool only omits its
        # collision geometry for the final descent and the released retreat.
        object_contact_scene = _planning_scene(
            snapshot,
            arm,
            unmanipulated_objects,
            EmptyTool(),
        )
        adjust = self._move_above_place(
            arm,
            snapshot.robot(arm).joints,
            place_tcp_pose_env,
            held_object_scene,
            "adjust",
        )
        place = self._move(
            arm,
            adjust.trajectory.end,
            place_tcp_pose_env,
            object_contact_scene,
            "place",
            (0.0, 0.0, -1.0),
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
        retreat_tcp_pose_env = approach_start_pose(
            place_tcp_pose_env,
            approach_axis_tcp,
            approach_distance_m,
        )
        retreat_axis_env = tuple(
            -component_env
            for component_env in rotate_vector_xyzw(
                place_tcp_pose_env.orientation_xyzw,
                approach_axis_tcp,
            )
        )
        retreat = self._move(
            arm,
            place.trajectory.end,
            retreat_tcp_pose_env,
            object_contact_scene,
            "retreat",
            retreat_axis_env,
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
        self._motion_planners[arm].commit_inspection_stages(
            ("adjust", "place", "retreat", "clear")
        )
        return PlacePlan(arm, adjust, place, retreat, clear)

    def _move(
        self,
        arm: Arm,
        start: JointState,
        target_tcp_pose_env: Pose,
        scene: PlanningScene,
        stage: PlanningStage,
        linear_axis_env: tuple[float, float, float] | None,
    ) -> MoveToPose:
        """Use an axis for contact motion; None permits transit and uprighting."""
        try:
            trajectory = self._motion_planners[arm].plan_pose(
                start,
                target_tcp_pose_env,
                scene,
                stage,
                linear_axis_env,
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
        stage: PlanningStage,
    ) -> MoveToJoints:
        try:
            trajectory = self._motion_planners[arm].plan_joints(
                start,
                target_joint_state,
                scene,
                stage,
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
        camera_stand=snapshot.camera_stand,
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


def _camera_side_up_dot(
    tcp_pose_env: Pose,
    camera_position_tcp_m: tuple[float, float, float],
    approach_axis_tcp: tuple[float, float, float],
) -> float:
    """Measure the camera mounting side, excluding its axial TCP offset."""
    camera_axial_offset_m = sum(
        component_m * axis_component
        for component_m, axis_component in zip(
            camera_position_tcp_m, approach_axis_tcp, strict=True
        )
    )
    camera_side_tcp_m = tuple(
        component_m - camera_axial_offset_m * axis_component
        for component_m, axis_component in zip(
            camera_position_tcp_m, approach_axis_tcp, strict=True
        )
    )
    camera_side_length_m = math.hypot(*camera_side_tcp_m)
    camera_side_axis_tcp = tuple(
        component_m / camera_side_length_m for component_m in camera_side_tcp_m
    )
    camera_side_axis_env = rotate_vector_xyzw(
        tcp_pose_env.orientation_xyzw,
        camera_side_axis_tcp,
    )
    return camera_side_axis_env[2]


def _parallel_jaw_grasp_poses(
    object_pose_env: Pose,
    candidate: GraspCandidate,
) -> tuple[Pose, Pose]:
    """Return the original grasp and its half-turn equivalent for upright filtering."""

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
    return canonical_tcp_pose_env, alternate_tcp_pose_env


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
    "PlanningStage",
    "PlacePlan",
    "SkillPlanner",
]
