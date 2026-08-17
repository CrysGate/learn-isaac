"""Atomic Cartesian and joint-space motion skills."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import torch

from .core import (
    CartesianMotionConfig,
    CartesianSegment,
    JointRuntime,
    MotionRuntime,
    Pose,
    SegmentStatus,
    SkillStep,
    finite_positive,
)


class MovePhase(str, Enum):
    MOVE = "move"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class MoveToPoseSkill:
    """Move the TCP to one world-space pose and verify convergence."""

    def __init__(
        self,
        runtime: MotionRuntime,
        target_pose_w: Pose,
        *,
        gripper_open: bool | None = None,
        config: CartesianMotionConfig | None = None,
    ) -> None:
        self.runtime = runtime
        self.gripper_open = gripper_open
        self.phase = MovePhase.MOVE
        self.message = ""
        self._segment = CartesianSegment(
            runtime,
            target_pose_w,
            config or CartesianMotionConfig(),
        )

    @property
    def done(self) -> bool:
        return self.phase in {MovePhase.SUCCEEDED, MovePhase.FAILED}

    @property
    def succeeded(self) -> bool:
        return self.phase is MovePhase.SUCCEEDED

    def tick(self) -> SkillStep:
        if self.done:
            return self._step(self.runtime.hold_action(self.gripper_open))
        update = self._segment.tick(self.gripper_open)
        if update.status is SegmentStatus.REACHED:
            self.phase = MovePhase.SUCCEEDED
            return self._step(self.runtime.hold_action(self.gripper_open))
        if update.status is SegmentStatus.FAILED:
            self.phase = MovePhase.FAILED
            self.message = update.message
            return self._step(self.runtime.hold_action(self.gripper_open))
        assert update.action is not None
        return self._step(update.action)

    def _step(self, action: torch.Tensor) -> SkillStep:
        return SkillStep(
            action,
            self.phase.value,
            self.done,
            self.succeeded,
            self.message,
        )


@dataclass(frozen=True)
class HomeConfig:
    joint_speed_rad_s: float = 0.8
    joint_tolerance_rad: float = 0.02
    settle_timeout_s: float = 5.0

    def __post_init__(self) -> None:
        for name in (
            "joint_speed_rad_s",
            "joint_tolerance_rad",
            "settle_timeout_s",
        ):
            finite_positive(getattr(self, name), name)


class HomePhase(str, Enum):
    MOVE = "move"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class HomeSkill:
    """Return one arm to the profile's configured home joint positions."""

    def __init__(
        self,
        runtime: JointRuntime,
        *,
        gripper_open: bool | None = None,
        config: HomeConfig | None = None,
    ) -> None:
        self.runtime = runtime
        self.gripper_open = gripper_open
        self.config = config or HomeConfig()
        self.phase = HomePhase.MOVE
        self.message = ""
        self._start = runtime.arm_joint_positions().clone()
        self._target = runtime.home_joint_positions().clone()
        if self._start.shape != self._target.shape:
            raise ValueError("current and home joint vectors must have the same shape")
        if not torch.isfinite(self._start).all() or not torch.isfinite(self._target).all():
            raise ValueError("current and home joint vectors must be finite")
        max_distance = torch.max(torch.abs(self._target - self._start)).item()
        self._duration_s = max(
            runtime.step_dt,
            max_distance / self.config.joint_speed_rad_s,
        )
        self._elapsed_s = 0.0

    @property
    def done(self) -> bool:
        return self.phase in {HomePhase.SUCCEEDED, HomePhase.FAILED}

    @property
    def succeeded(self) -> bool:
        return self.phase is HomePhase.SUCCEEDED

    def tick(self) -> SkillStep:
        if self.done:
            return self._step(self.runtime.hold_action(self.gripper_open))
        if self._elapsed_s >= self._duration_s:
            error = torch.max(
                torch.abs(self.runtime.arm_joint_positions() - self._target)
            ).item()
            if error <= self.config.joint_tolerance_rad:
                self.phase = HomePhase.SUCCEEDED
                return self._step(self.runtime.hold_action(self.gripper_open))
            if self._elapsed_s >= self._duration_s + self.config.settle_timeout_s:
                self.phase = HomePhase.FAILED
                self.message = f"home target did not converge: joint error {error:.4f} rad"
                return self._step(self.runtime.hold_action(self.gripper_open))

        tau = min(1.0, (self._elapsed_s + self.runtime.step_dt) / self._duration_s)
        target = self._start + tau * (self._target - self._start)
        self._elapsed_s += self.runtime.step_dt
        return self._step(self.runtime.joint_action(target, self.gripper_open))

    def _step(self, action: torch.Tensor) -> SkillStep:
        return SkillStep(
            action,
            self.phase.value,
            self.done,
            self.succeeded,
            self.message,
        )


__all__ = [
    "HomeConfig",
    "HomePhase",
    "HomeSkill",
    "MovePhase",
    "MoveToPoseSkill",
]
