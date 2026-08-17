"""Robot-independent in-hand rotation operation."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import torch

from .core import (
    CartesianMotionConfig,
    CartesianSegment,
    ManipulationRuntime,
    SegmentStatus,
    SkillStep,
    compose_pose,
    finite_non_negative,
    finite_positive,
    orientation_error,
    relative_pose,
)


@dataclass(frozen=True)
class RotateConfig:
    lift_distance_m: float = 0.05
    verify_duration_s: float = 0.25
    verify_timeout_s: float = 2.0
    object_orientation_tolerance_rad: float = 0.15
    motion: CartesianMotionConfig = field(default_factory=CartesianMotionConfig)

    def __post_init__(self) -> None:
        finite_non_negative(self.lift_distance_m, "lift_distance_m")
        for name in (
            "verify_duration_s",
            "verify_timeout_s",
            "object_orientation_tolerance_rad",
        ):
            finite_positive(getattr(self, name), name)
        if self.verify_duration_s > self.verify_timeout_s:
            raise ValueError("verify_duration_s cannot exceed verify_timeout_s")


class RotatePhase(str, Enum):
    LIFT = "lift"
    ROTATE = "rotate"
    VERIFY = "verify"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class RotateSkill:
    """Lift a grasped object and rotate it to a world-space orientation."""

    def __init__(
        self,
        runtime: ManipulationRuntime,
        object_name: str,
        target_object_orientation_wxyz: torch.Tensor,
        *,
        config: RotateConfig | None = None,
    ) -> None:
        if not object_name:
            raise ValueError("object_name cannot be empty")
        orientation = target_object_orientation_wxyz.to(runtime.tcp_pose_w()[1])
        if orientation.shape != (4,) or not torch.isfinite(orientation).all():
            raise ValueError(
                "target_object_orientation_wxyz must contain four finite values"
            )
        if not math.isclose(
            torch.linalg.vector_norm(orientation).item(),
            1.0,
            abs_tol=1.0e-5,
        ):
            raise ValueError("target_object_orientation_wxyz must be a unit quaternion")
        self.runtime = runtime
        self.object_name = object_name
        self.target_orientation = orientation.clone()
        self.config = config or RotateConfig()
        self.phase = RotatePhase.LIFT
        self.message = ""
        self._object_to_tcp = relative_pose(
            runtime.object_pose_w(object_name),
            runtime.tcp_pose_w(),
        )
        tcp_position, tcp_orientation = runtime.tcp_pose_w()
        lift_position = tcp_position.clone()
        lift_position[2] += self.config.lift_distance_m
        self._segment = CartesianSegment(
            runtime,
            (lift_position, tcp_orientation.clone()),
            self.config.motion,
        )
        self._verify_elapsed_s = 0.0
        self._stable_elapsed_s = 0.0

    @property
    def done(self) -> bool:
        return self.phase in {RotatePhase.SUCCEEDED, RotatePhase.FAILED}

    @property
    def succeeded(self) -> bool:
        return self.phase is RotatePhase.SUCCEEDED

    def tick(self) -> SkillStep:
        if self.done:
            return self._step(self.runtime.hold_action(False))
        if self.phase in {RotatePhase.LIFT, RotatePhase.ROTATE}:
            return self._tick_motion()
        return self._tick_verify()

    def _tick_motion(self) -> SkillStep:
        update = self._segment.tick(False)
        if update.status is SegmentStatus.FAILED:
            return self._fail(update.message)
        if update.status is SegmentStatus.REACHED:
            if self.phase is RotatePhase.LIFT:
                object_position, _ = self.runtime.object_pose_w(self.object_name)
                target_object_pose = (
                    object_position.clone(),
                    self.target_orientation,
                )
                target_tcp = compose_pose(target_object_pose, self._object_to_tcp)
                self.phase = RotatePhase.ROTATE
                self._segment = CartesianSegment(
                    self.runtime,
                    target_tcp,
                    self.config.motion,
                )
            else:
                self.phase = RotatePhase.VERIFY
            return self.tick()
        assert update.action is not None
        return self._step(update.action)

    def _tick_verify(self) -> SkillStep:
        _, orientation = self.runtime.object_pose_w(self.object_name)
        error = orientation_error(orientation, self.target_orientation)
        self._stable_elapsed_s = (
            self._stable_elapsed_s + self.runtime.step_dt
            if error <= self.config.object_orientation_tolerance_rad
            else 0.0
        )
        self._verify_elapsed_s += self.runtime.step_dt
        if self._stable_elapsed_s >= self.config.verify_duration_s:
            self.phase = RotatePhase.SUCCEEDED
            self.message = f"object rotated with {error:.4f} rad orientation error"
        elif self._verify_elapsed_s >= self.config.verify_timeout_s:
            return self._fail(
                f"object did not reach target orientation; error {error:.4f} rad"
            )
        return self._step(self.runtime.hold_action(False))

    def _fail(self, message: str) -> SkillStep:
        self.phase = RotatePhase.FAILED
        self.message = message
        return self._step(self.runtime.hold_action())

    def _step(self, action: torch.Tensor) -> SkillStep:
        return SkillStep(
            action,
            self.phase.value,
            self.done,
            self.succeeded,
            self.message,
        )


__all__ = ["RotateConfig", "RotatePhase", "RotateSkill"]
