"""Independent final-state checks used by atomic-skill task runners."""

from __future__ import annotations

from dataclasses import dataclass

import torch

from .core import Pose, finite_positive, pose_error, validate_pose


@dataclass(frozen=True)
class GoalResult:
    succeeded: bool
    message: str
    metrics: dict[str, float]


@dataclass(frozen=True)
class LiftGoal:
    initial_height_m: float
    minimum_lift_m: float

    def __post_init__(self) -> None:
        finite_positive(self.minimum_lift_m, "minimum_lift_m")

    def evaluate(self, object_pose_w: Pose) -> GoalResult:
        lifted = float(object_pose_w[0][2].item()) - self.initial_height_m
        return GoalResult(
            succeeded=lifted >= self.minimum_lift_m,
            message=(
                f"object lifted {lifted:.4f} m; "
                f"required {self.minimum_lift_m:.4f} m"
            ),
            metrics={"lift_m": lifted},
        )


@dataclass(frozen=True)
class ObjectPoseGoal:
    target_pose_w: Pose
    position_tolerance_m: float = 0.025
    orientation_tolerance_rad: float = 0.20

    def __post_init__(self) -> None:
        validate_pose(self.target_pose_w, "target_pose_w")
        finite_positive(self.position_tolerance_m, "position_tolerance_m")
        finite_positive(
            self.orientation_tolerance_rad,
            "orientation_tolerance_rad",
        )

    def evaluate(self, object_pose_w: Pose) -> GoalResult:
        position, orientation = pose_error(object_pose_w, self.target_pose_w)
        return GoalResult(
            succeeded=(
                position <= self.position_tolerance_m
                and orientation <= self.orientation_tolerance_rad
            ),
            message=(
                f"object pose error is {position:.4f} m and "
                f"{orientation:.4f} rad"
            ),
            metrics={
                "position_error_m": position,
                "orientation_error_rad": orientation,
            },
        )


@dataclass(frozen=True)
class JointPositionGoal:
    target_positions: torch.Tensor
    tolerance_rad: float = 0.02

    def __post_init__(self) -> None:
        if self.target_positions.ndim != 1 or not torch.isfinite(
            self.target_positions
        ).all():
            raise ValueError("target_positions must be a finite joint vector")
        finite_positive(self.tolerance_rad, "tolerance_rad")

    def evaluate(self, joint_positions: torch.Tensor) -> GoalResult:
        if joint_positions.shape != self.target_positions.shape:
            raise ValueError("joint_positions shape does not match the goal")
        error = torch.max(
            torch.abs(joint_positions - self.target_positions)
        ).item()
        return GoalResult(
            succeeded=error <= self.tolerance_rad,
            message=(
                f"maximum home joint error is {error:.4f} rad; "
                f"allowed {self.tolerance_rad:.4f} rad"
            ),
            metrics={"maximum_joint_error_rad": error},
        )


__all__ = [
    "GoalResult",
    "JointPositionGoal",
    "LiftGoal",
    "ObjectPoseGoal",
]
