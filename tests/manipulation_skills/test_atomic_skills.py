"""Fast state-machine tests for robot-independent manipulation skills."""

from __future__ import annotations

import math

import pytest
import torch
from isaaclab.utils.math import quat_from_angle_axis

from manipulation_skills import (
    CartesianMotionConfig,
    CloseGripperSkill,
    GraspCandidate,
    GripperConfig,
    HomeConfig,
    HomeSkill,
    InsertConfig,
    InsertSkill,
    JointPositionGoal,
    LiftGoal,
    MoveToPoseSkill,
    ObjectPoseGoal,
    OpenGripperSkill,
    PickConfig,
    PickSkill,
    PlaceConfig,
    PlaceSkill,
    RotateConfig,
    RotateSkill,
    SkillSequence,
)
from manipulation_skills.core import Pose, clone_pose, compose_pose, relative_pose
from manipulation_skills.pick import ResolvedGrasp


IDENTITY = torch.tensor((1.0, 0.0, 0.0, 0.0))


class FakeRuntime:
    step_dt = 0.1
    max_gripper_aperture_m = 0.1

    def __init__(self) -> None:
        self.tcp = (torch.tensor((0.0, 0.0, 0.30)), IDENTITY.clone())
        self.objects = {
            "object": (torch.tensor((0.25, 0.0, 0.10)), IDENTITY.clone())
        }
        self.aperture = self.max_gripper_aperture_m
        self.joints = torch.tensor((0.4, -0.3, 0.2))
        self.home_joints = torch.zeros(3)
        self.contact_possible = True
        self.grasped = False
        self._tcp_to_object: Pose | None = None
        self._pending: tuple[str, object, bool | None] | None = None

    def hold_action(self, gripper_open: bool | None = None) -> torch.Tensor:
        self._pending = ("hold", None, gripper_open)
        return torch.tensor((0.0,))

    def move_action(
        self,
        target_tcp_pose_w: Pose,
        gripper_open: bool | None,
    ) -> torch.Tensor:
        self._pending = ("move", clone_pose(target_tcp_pose_w), gripper_open)
        return torch.tensor((1.0,))

    def joint_action(
        self,
        joint_positions: torch.Tensor,
        gripper_open: bool | None = None,
    ) -> torch.Tensor:
        self._pending = ("joint", joint_positions.clone(), gripper_open)
        return torch.tensor((2.0,))

    def tcp_pose_w(self) -> Pose:
        return clone_pose(self.tcp)

    def object_pose_w(self, object_name: str) -> Pose:
        return clone_pose(self.objects[object_name])

    def gripper_aperture_m(self) -> float:
        return self.aperture

    def arm_joint_positions(self) -> torch.Tensor:
        return self.joints.clone()

    def home_joint_positions(self) -> torch.Tensor:
        return self.home_joints.clone()

    def default_grasp(
        self,
        object_name: str,
        height_offset_m: float,
    ) -> ResolvedGrasp:
        position, orientation = self.object_pose_w(object_name)
        position[2] += height_offset_m
        return ResolvedGrasp(
            (position, orientation),
            position.new_tensor((1.0, 0.0, 0.0)),
        )

    def set_grasped(self, object_name: str = "object") -> None:
        self.grasped = True
        self.aperture = 0.02
        self._tcp_to_object = relative_pose(
            self.tcp,
            self.objects[object_name],
        )

    def apply(self) -> None:
        assert self._pending is not None
        command, target, gripper_open = self._pending
        if command == "move":
            assert isinstance(target, tuple)
            self.tcp = clone_pose(target)
            if self.grasped:
                assert self._tcp_to_object is not None
                self.objects["object"] = compose_pose(
                    self.tcp,
                    self._tcp_to_object,
                )
        elif command == "joint":
            assert isinstance(target, torch.Tensor)
            self.joints = target.clone()

        if gripper_open is True:
            self.aperture = self.max_gripper_aperture_m
            self.grasped = False
            self._tcp_to_object = None
        elif gripper_open is False:
            if not self.grasped and self.contact_possible:
                self.set_grasped()
            elif not self.grasped:
                self.aperture = 0.0
        self._pending = None


def run_skill(skill, runtime: FakeRuntime, max_steps: int = 500):
    phases = []
    for _ in range(max_steps):
        result = skill.tick()
        phases.append(result.phase)
        runtime.apply()
        if result.done:
            return result, phases
    raise AssertionError(f"skill did not finish after {max_steps} steps")


def fast_motion() -> CartesianMotionConfig:
    return CartesianMotionConfig(
        linear_speed_m_s=1.0,
        angular_speed_rad_s=5.0,
        settle_timeout_s=0.3,
        position_tolerance_m=1.0e-4,
        orientation_tolerance_rad=1.0e-3,
    )


def fast_gripper() -> GripperConfig:
    return GripperConfig(
        minimum_duration_s=0.1,
        timeout_s=0.4,
        aperture_tolerance_m=1.0e-3,
        minimum_contact_aperture_m=0.002,
    )


def test_move_gripper_and_home_primitives() -> None:
    runtime = FakeRuntime()
    target = (torch.tensor((0.1, 0.2, 0.4)), IDENTITY.clone())

    result, _ = run_skill(
        MoveToPoseSkill(runtime, target, config=fast_motion()),
        runtime,
    )
    assert result.succeeded
    assert torch.allclose(runtime.tcp[0], target[0])

    result, _ = run_skill(
        CloseGripperSkill(
            runtime,
            require_contact=True,
            config=fast_gripper(),
        ),
        runtime,
    )
    assert result.succeeded
    assert runtime.grasped

    result, _ = run_skill(
        OpenGripperSkill(runtime, config=fast_gripper()),
        runtime,
    )
    assert result.succeeded
    assert not runtime.grasped

    result, _ = run_skill(
        HomeSkill(
            runtime,
            config=HomeConfig(
                joint_speed_rad_s=2.0,
                joint_tolerance_rad=1.0e-4,
                settle_timeout_s=0.3,
            ),
        ),
        runtime,
    )
    assert result.succeeded
    assert torch.allclose(runtime.joints, runtime.home_joints)


def test_pick_lifts_and_verifies_object() -> None:
    runtime = FakeRuntime()
    skill = PickSkill(
        runtime,
        "object",
        grasp=GraspCandidate(
            position_object_m=(0.0, 0.0, 0.0),
            orientation_object_xyzw=(1.0, 0.0, 0.0, 0.0),
        ),
        config=PickConfig(
            approach_distance_m=0.05,
            lift_distance_m=0.08,
            linear_speed_m_s=1.0,
            angular_speed_rad_s=5.0,
            open_duration_s=0.1,
            open_timeout_s=0.4,
            close_duration_s=0.1,
            verify_duration_s=0.1,
            move_settle_timeout_s=0.3,
            position_tolerance_m=1.0e-4,
            orientation_tolerance_rad=1.0e-3,
            gripper_tolerance_m=1.0e-3,
            lift_tolerance_m=0.01,
        ),
    )

    result, phases = run_skill(skill, runtime)

    assert result.succeeded
    assert "pregrasp" in phases
    assert "approach" in phases
    assert "close" in phases
    assert "lift" in phases
    assert runtime.objects["object"][0][2].item() == pytest.approx(0.18)


def test_pick_reports_a_missed_grasp() -> None:
    runtime = FakeRuntime()
    runtime.contact_possible = False
    skill = PickSkill(
        runtime,
        "object",
        config=PickConfig(
            linear_speed_m_s=1.0,
            angular_speed_rad_s=5.0,
            open_duration_s=0.1,
            open_timeout_s=0.4,
            close_duration_s=0.1,
            verify_duration_s=0.1,
            move_settle_timeout_s=0.3,
        ),
    )

    result, _ = run_skill(skill, runtime)

    assert not result.succeeded
    assert "without capturing" in result.message


def test_place_releases_at_target_and_retreats() -> None:
    runtime = FakeRuntime()
    runtime.set_grasped()
    target = (torch.tensor((0.10, 0.15, 0.10)), IDENTITY.clone())
    skill = PlaceSkill(
        runtime,
        "object",
        target,
        config=PlaceConfig(
            preplace_distance_m=0.05,
            retreat_distance_m=0.05,
            verify_duration_s=0.1,
            verify_timeout_s=0.5,
            object_position_tolerance_m=1.0e-4,
            object_orientation_tolerance_rad=1.0e-3,
            motion=fast_motion(),
            gripper=fast_gripper(),
        ),
    )

    result, phases = run_skill(skill, runtime)

    assert result.succeeded
    assert not runtime.grasped
    assert torch.allclose(runtime.objects["object"][0], target[0], atol=1.0e-5)
    assert "descend" in phases
    assert "retreat" in phases


def test_place_can_release_above_a_surface_target() -> None:
    runtime = FakeRuntime()
    runtime.set_grasped()
    target = (torch.tensor((0.10, 0.15, 0.10)), IDENTITY.clone())
    skill = PlaceSkill(
        runtime,
        "object",
        target,
        config=PlaceConfig(
            preplace_distance_m=0.05,
            retreat_distance_m=0.05,
            release_clearance_m=0.04,
            verify_duration_s=0.1,
            verify_timeout_s=0.5,
            object_position_tolerance_m=1.0e-4,
            object_orientation_tolerance_rad=1.0e-3,
            motion=fast_motion(),
            gripper=fast_gripper(),
        ),
    )

    released_height = None
    for _ in range(500):
        result = skill.tick()
        runtime.apply()
        if result.phase == "open" and not runtime.grasped:
            released_height = runtime.objects["object"][0][2].item()
            runtime.objects["object"] = clone_pose(target)
        if result.done:
            break

    assert result.succeeded
    assert released_height == pytest.approx(0.14)


def test_insert_uses_preinsert_axis_and_releases() -> None:
    runtime = FakeRuntime()
    runtime.set_grasped()
    target = (torch.tensor((0.35, 0.0, 0.10)), IDENTITY.clone())
    skill = InsertSkill(
        runtime,
        "object",
        target,
        torch.tensor((1.0, 0.0, 0.0)),
        config=InsertConfig(
            preinsert_distance_m=0.05,
            retreat_distance_m=0.05,
            verify_duration_s=0.1,
            verify_timeout_s=0.5,
            object_position_tolerance_m=1.0e-4,
            object_orientation_tolerance_rad=1.0e-3,
            motion=fast_motion(),
            gripper=fast_gripper(),
        ),
    )

    result, phases = run_skill(skill, runtime)

    assert result.succeeded
    assert "preinsert" in phases
    assert "insert" in phases
    assert not runtime.grasped
    assert torch.allclose(runtime.objects["object"][0], target[0], atol=1.0e-5)


def test_insert_rejects_a_zero_direction() -> None:
    runtime = FakeRuntime()
    with pytest.raises(ValueError, match="cannot be zero"):
        InsertSkill(
            runtime,
            "object",
            runtime.object_pose_w("object"),
            torch.zeros(3),
        )


def test_rotate_changes_the_grasped_object_orientation() -> None:
    runtime = FakeRuntime()
    runtime.set_grasped()
    target = quat_from_angle_axis(
        torch.tensor(math.pi / 2.0),
        torch.tensor((0.0, 0.0, 1.0)),
    )
    skill = RotateSkill(
        runtime,
        "object",
        target,
        config=RotateConfig(
            lift_distance_m=0.03,
            verify_duration_s=0.1,
            verify_timeout_s=0.5,
            object_orientation_tolerance_rad=1.0e-3,
            motion=fast_motion(),
        ),
    )

    result, phases = run_skill(skill, runtime)

    assert result.succeeded
    assert "rotate" in phases
    actual = runtime.objects["object"][1]
    assert abs(torch.dot(actual, target).item()) == pytest.approx(1.0)


def test_skill_sequence_constructs_later_skills_lazily() -> None:
    runtime = FakeRuntime()
    created = []

    def first():
        created.append("first")
        return CloseGripperSkill(
            runtime,
            require_contact=True,
            config=fast_gripper(),
        )

    def second():
        created.append("second")
        assert runtime.grasped
        return OpenGripperSkill(runtime, config=fast_gripper())

    sequence = SkillSequence((first, second))

    assert created == []
    result, phases = run_skill(sequence, runtime)
    assert result.succeeded
    assert created == ["first", "second"]
    assert "0:succeeded" in phases
    assert phases[-1] == "1:succeeded"


def test_task_goals_are_independent_from_skill_results() -> None:
    runtime = FakeRuntime()
    initial_height = runtime.objects["object"][0][2].item()
    lifted_pose = runtime.object_pose_w("object")
    lifted_pose[0][2] += 0.09

    lift_result = LiftGoal(initial_height, 0.08).evaluate(lifted_pose)
    pose_result = ObjectPoseGoal(lifted_pose).evaluate(lifted_pose)
    joint_result = JointPositionGoal(runtime.home_joints).evaluate(
        runtime.home_joints
    )

    assert lift_result.succeeded
    assert pose_result.succeeded
    assert joint_result.succeeded
