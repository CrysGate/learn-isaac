"""Read-only geometry with explicit subject-and-reference-frame names.

Geometry follows ``<subject>_<quantity>_<reference_frame>``.  Positions append
``_m``, orientations append ``_xyzw``, and ``env`` denotes the local frame of
one parallel environment rather than Isaac Sim's shared absolute world frame.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, TypeAlias

import torch
from torch import Tensor

from .models import Arm, Pose


@dataclass(frozen=True, slots=True)
class JointState:
    positions: Tensor

    def __post_init__(self) -> None:
        if self.positions.ndim != 1:
            raise ValueError("joint state must be a one-dimensional tensor")


@dataclass(frozen=True, slots=True)
class JointTrajectory:
    positions: Tensor

    def __post_init__(self) -> None:
        if self.positions.ndim != 2 or self.positions.shape[0] == 0:
            raise ValueError("joint trajectory must contain one or more waypoints")
        if not torch.isfinite(self.positions).all().item():
            raise ValueError("joint trajectory must contain finite values")

    @property
    def end(self) -> JointState:
        return JointState(self.positions[-1])


@dataclass(frozen=True, slots=True)
class RobotState:
    joints: JointState
    tcp_pose_env: Pose


@dataclass(frozen=True, slots=True)
class SceneObject:
    name: str
    pose_env: Pose
    size_m: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class SceneSnapshot:
    left_robot: RobotState
    right_robot: RobotState
    table: SceneObject
    camera_stand: tuple[SceneObject, ...]
    objects: tuple[SceneObject, ...]

    def robot(self, arm: Arm) -> RobotState:
        return self.left_robot if arm == "left" else self.right_robot

    def object(self, object_name: str) -> SceneObject:
        for scene_object in self.objects:
            if scene_object.name == object_name:
                return scene_object
        raise ValueError(f"scene has no object {object_name!r}")


@dataclass(frozen=True, slots=True)
class GraspCandidate:
    """Parallel-jaw TCP pose expressed in the object frame.

    The TCP +Y axis is the finger-opening axis.  Exchanging the two identical
    fingers therefore produces an equivalent pose after a half-turn around
    ``approach_axis_tcp``.
    """

    tcp_pose_object: Pose
    approach_axis_tcp: tuple[float, float, float]
    approach_distance_m: float
    score: float

    def __post_init__(self) -> None:
        axis_norm = math.sqrt(sum(value * value for value in self.approach_axis_tcp))
        if not math.isclose(axis_norm, 1.0, abs_tol=1.0e-6):
            raise ValueError("grasp approach axis must be a unit vector")
        if not math.isfinite(self.score) or not 0.0 <= self.score <= 1.0:
            raise ValueError("grasp score must be within [0, 1]")


@dataclass(frozen=True, slots=True)
class GraspState:
    """Measured relationship after a gripper has closed on an object."""

    object_name: str
    arm: Arm
    gripper_aperture_m: float
    object_pose_env: Pose
    tcp_pose_env: Pose
    tcp_pose_object: Pose


@dataclass(frozen=True, slots=True)
class EmptyTool:
    """Collision state for an empty gripper."""


@dataclass(frozen=True, slots=True)
class HeldObject:
    """Collision state for an object rigidly held at the measured grasp."""

    object: SceneObject
    tcp_pose_object: Pose


ToolState: TypeAlias = EmptyTool | HeldObject


@dataclass(frozen=True, slots=True)
class PlanningScene:
    """All collision facts required to plan one arm segment."""

    table: SceneObject
    camera_stand: tuple[SceneObject, ...]
    objects: tuple[SceneObject, ...]
    other_arm: Arm
    other_robot: RobotState
    tool: ToolState


class SkillContext(Protocol):
    """Read live facts for one environment slot without planning or mutation."""

    def snapshot(self) -> SceneSnapshot: ...

    def grasp_candidates(
        self,
        object_name: str,
        arm: Arm,
    ) -> tuple[GraspCandidate, ...]: ...

    def measure_grasp(self, object_name: str, arm: Arm) -> GraspState: ...


__all__ = [
    "EmptyTool",
    "GraspCandidate",
    "GraspState",
    "HeldObject",
    "JointState",
    "JointTrajectory",
    "PlanningScene",
    "RobotState",
    "SceneObject",
    "SceneSnapshot",
    "SkillContext",
    "ToolState",
]
