"""Shared contracts and Cartesian motion support for atomic skills."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Protocol, TypeAlias

import torch
from isaaclab.utils.math import (
    combine_frame_transforms,
    quat_slerp,
    subtract_frame_transforms,
)


Pose: TypeAlias = tuple[torch.Tensor, torch.Tensor]


def finite_positive(value: float, name: str) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive and finite")


def finite_non_negative(value: float, name: str) -> None:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be non-negative and finite")


def validate_pose(pose: Pose, name: str = "pose") -> None:
    position, orientation = pose
    if position.shape != (3,) or not torch.isfinite(position).all():
        raise ValueError(f"{name} position must contain three finite values")
    if orientation.shape != (4,) or not torch.isfinite(orientation).all():
        raise ValueError(f"{name} orientation must contain four finite values")
    norm = torch.linalg.vector_norm(orientation).item()
    if not math.isclose(norm, 1.0, abs_tol=1.0e-5):
        raise ValueError(f"{name} orientation must be a unit quaternion")


def clone_pose(pose: Pose) -> Pose:
    return pose[0].clone(), pose[1].clone()


def orientation_error(actual: torch.Tensor, target: torch.Tensor) -> float:
    dot = torch.dot(actual, target).abs().clamp(max=1.0)
    return float((2.0 * torch.acos(dot)).item())


def pose_error(actual: Pose, target: Pose) -> tuple[float, float]:
    position = float(torch.linalg.vector_norm(target[0] - actual[0]).item())
    return position, orientation_error(actual[1], target[1])


def interpolate_pose(start: Pose, target: Pose, tau: float) -> Pose:
    position = start[0] + tau * (target[0] - start[0])
    orientation = quat_slerp(start[1], target[1], tau)
    return position, orientation


def compose_pose(parent: Pose, child_in_parent: Pose) -> Pose:
    """Compose two poses using Isaac Lab's scalar-first quaternion convention."""

    position, orientation = combine_frame_transforms(
        parent[0].unsqueeze(0),
        parent[1].unsqueeze(0),
        child_in_parent[0].unsqueeze(0),
        child_in_parent[1].unsqueeze(0),
    )
    return position[0], orientation[0]


def relative_pose(parent: Pose, child: Pose) -> Pose:
    """Return the child pose expressed in the parent frame."""

    position, orientation = subtract_frame_transforms(
        parent[0].unsqueeze(0),
        parent[1].unsqueeze(0),
        child[0].unsqueeze(0),
        child[1].unsqueeze(0),
    )
    return position[0], orientation[0]


def normalized_direction(
    direction: torch.Tensor,
    name: str = "direction",
) -> torch.Tensor:
    if direction.shape != (3,) or not torch.isfinite(direction).all():
        raise ValueError(f"{name} must contain three finite values")
    norm = torch.linalg.vector_norm(direction)
    if norm.item() <= 1.0e-8:
        raise ValueError(f"{name} cannot be zero")
    return direction / norm


class MotionRuntime(Protocol):
    """Runtime operations required by Cartesian manipulation skills."""

    step_dt: float

    def hold_action(self, gripper_open: bool | None = None) -> torch.Tensor: ...

    def move_action(
        self,
        target_tcp_pose_w: Pose,
        gripper_open: bool | None,
    ) -> torch.Tensor: ...

    def tcp_pose_w(self) -> Pose: ...


class ManipulationRuntime(MotionRuntime, Protocol):
    """Runtime state required by object and gripper skills."""

    max_gripper_aperture_m: float

    def object_pose_w(self, object_name: str) -> Pose: ...

    def gripper_aperture_m(self) -> float: ...


class JointRuntime(MotionRuntime, Protocol):
    """Runtime operations required to return one arm to its home state."""

    def arm_joint_positions(self) -> torch.Tensor: ...

    def home_joint_positions(self) -> torch.Tensor: ...

    def joint_action(
        self,
        joint_positions: torch.Tensor,
        gripper_open: bool | None = None,
    ) -> torch.Tensor: ...


@dataclass(frozen=True)
class SkillStep:
    """One environment action and the state of its producing skill."""

    action: torch.Tensor
    phase: str
    done: bool
    succeeded: bool
    message: str = ""


class AtomicSkill(Protocol):
    """Common tick-driven interface implemented by every atomic skill."""

    @property
    def done(self) -> bool: ...

    @property
    def succeeded(self) -> bool: ...

    def tick(self) -> SkillStep: ...


@dataclass(frozen=True)
class CartesianMotionConfig:
    """Speed and convergence thresholds for one Cartesian segment."""

    linear_speed_m_s: float = 0.15
    angular_speed_rad_s: float = 1.0
    settle_timeout_s: float = 5.0
    position_tolerance_m: float = 0.008
    orientation_tolerance_rad: float = 0.08

    def __post_init__(self) -> None:
        for name in (
            "linear_speed_m_s",
            "angular_speed_rad_s",
            "settle_timeout_s",
            "position_tolerance_m",
            "orientation_tolerance_rad",
        ):
            finite_positive(getattr(self, name), name)


class SegmentStatus(str, Enum):
    MOVING = "moving"
    REACHED = "reached"
    FAILED = "failed"


@dataclass(frozen=True)
class SegmentStep:
    action: torch.Tensor | None
    status: SegmentStatus
    message: str = ""


class CartesianSegment:
    """Time-scaled Cartesian interpolation with feedback convergence checks."""

    def __init__(
        self,
        runtime: MotionRuntime,
        target: Pose,
        config: CartesianMotionConfig,
    ) -> None:
        validate_pose(target, "target")
        self.runtime = runtime
        self.config = config
        self.start = clone_pose(runtime.tcp_pose_w())
        self.target = clone_pose(target)
        distance = torch.linalg.vector_norm(target[0] - self.start[0]).item()
        angle = orientation_error(self.start[1], target[1])
        self.duration_s = max(
            runtime.step_dt,
            distance / config.linear_speed_m_s,
            angle / config.angular_speed_rad_s,
        )
        self.elapsed_s = 0.0

    def tick(self, gripper_open: bool | None) -> SegmentStep:
        if self.elapsed_s >= self.duration_s:
            position, orientation = pose_error(
                self.runtime.tcp_pose_w(),
                self.target,
            )
            if (
                position <= self.config.position_tolerance_m
                and orientation <= self.config.orientation_tolerance_rad
            ):
                return SegmentStep(None, SegmentStatus.REACHED)
            if self.elapsed_s >= self.duration_s + self.config.settle_timeout_s:
                return SegmentStep(
                    None,
                    SegmentStatus.FAILED,
                    "Cartesian target did not converge: "
                    f"position error {position:.4f} m, "
                    f"orientation error {orientation:.4f} rad",
                )

        tau = min(
            1.0,
            (self.elapsed_s + self.runtime.step_dt) / self.duration_s,
        )
        target = interpolate_pose(self.start, self.target, tau)
        action = self.runtime.move_action(target, gripper_open)
        self.elapsed_s += self.runtime.step_dt
        return SegmentStep(action, SegmentStatus.MOVING)


__all__ = [
    "AtomicSkill",
    "CartesianMotionConfig",
    "CartesianSegment",
    "JointRuntime",
    "ManipulationRuntime",
    "MotionRuntime",
    "Pose",
    "SegmentStatus",
    "SegmentStep",
    "SkillStep",
    "clone_pose",
    "compose_pose",
    "finite_non_negative",
    "finite_positive",
    "interpolate_pose",
    "normalized_direction",
    "orientation_error",
    "pose_error",
    "relative_pose",
    "validate_pose",
]
