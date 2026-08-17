"""Robot-independent place operation for one grasped rigid object."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import torch

from .core import (
    CartesianMotionConfig,
    CartesianSegment,
    ManipulationRuntime,
    Pose,
    SegmentStatus,
    SkillStep,
    compose_pose,
    finite_non_negative,
    finite_positive,
    normalized_direction,
    pose_error,
    relative_pose,
    validate_pose,
)
from .gripper import GripperConfig, OpenGripperSkill


@dataclass(frozen=True)
class PlaceConfig:
    preplace_distance_m: float = 0.10
    retreat_distance_m: float = 0.10
    release_clearance_m: float = 0.0
    verify_duration_s: float = 0.30
    verify_timeout_s: float = 2.0
    object_position_tolerance_m: float = 0.025
    object_orientation_tolerance_rad: float = 0.20
    motion: CartesianMotionConfig = field(default_factory=CartesianMotionConfig)
    gripper: GripperConfig = field(default_factory=GripperConfig)

    def __post_init__(self) -> None:
        for name in (
            "preplace_distance_m",
            "retreat_distance_m",
            "verify_duration_s",
            "verify_timeout_s",
            "object_position_tolerance_m",
            "object_orientation_tolerance_rad",
        ):
            finite_positive(getattr(self, name), name)
        finite_non_negative(self.release_clearance_m, "release_clearance_m")
        if self.verify_duration_s > self.verify_timeout_s:
            raise ValueError("verify_duration_s cannot exceed verify_timeout_s")


class PlacePhase(str, Enum):
    PREPLACE = "preplace"
    DESCEND = "descend"
    OPEN = "open"
    RETREAT = "retreat"
    VERIFY = "verify"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class PlaceSkill:
    """Move a grasped object to a target pose, release it, and retreat."""

    def __init__(
        self,
        runtime: ManipulationRuntime,
        object_name: str,
        target_object_pose_w: Pose,
        *,
        placement_direction_w: torch.Tensor | None = None,
        config: PlaceConfig | None = None,
    ) -> None:
        if not object_name:
            raise ValueError("object_name cannot be empty")
        validate_pose(target_object_pose_w, "target_object_pose_w")
        self.runtime = runtime
        self.object_name = object_name
        self.target_object_pose_w = (
            target_object_pose_w[0].clone(),
            target_object_pose_w[1].clone(),
        )
        self.config = config or PlaceConfig()
        default_direction = target_object_pose_w[0].new_tensor((0.0, 0.0, -1.0))
        self.direction_w = normalized_direction(
            default_direction
            if placement_direction_w is None
            else placement_direction_w.to(target_object_pose_w[0]),
            "placement_direction_w",
        )
        self.phase = PlacePhase.PREPLACE
        self.message = ""
        self._object_to_tcp = relative_pose(
            runtime.object_pose_w(object_name),
            runtime.tcp_pose_w(),
        )
        release_position = (
            self.target_object_pose_w[0]
            - self.direction_w * self.config.release_clearance_m
        )
        self._release_object_pose = (
            release_position,
            self.target_object_pose_w[1].clone(),
        )
        self._target_tcp = compose_pose(
            self._release_object_pose,
            self._object_to_tcp,
        )
        preplace_position = (
            self._target_tcp[0]
            - self.direction_w * self.config.preplace_distance_m
        )
        self._preplace_tcp = (preplace_position, self._target_tcp[1].clone())
        self._segment = CartesianSegment(
            runtime,
            self._preplace_tcp,
            self.config.motion,
        )
        self._gripper: OpenGripperSkill | None = None
        self._verify_elapsed_s = 0.0
        self._stable_elapsed_s = 0.0

    @property
    def done(self) -> bool:
        return self.phase in {PlacePhase.SUCCEEDED, PlacePhase.FAILED}

    @property
    def succeeded(self) -> bool:
        return self.phase is PlacePhase.SUCCEEDED

    def tick(self) -> SkillStep:
        if self.done:
            return self._step(self.runtime.hold_action(True))

        if self.phase in {
            PlacePhase.PREPLACE,
            PlacePhase.DESCEND,
            PlacePhase.RETREAT,
        }:
            return self._tick_motion()
        if self.phase is PlacePhase.OPEN:
            return self._tick_open()
        return self._tick_verify()

    def _tick_motion(self) -> SkillStep:
        gripper_open = self.phase is PlacePhase.RETREAT
        update = self._segment.tick(gripper_open)
        if update.status is SegmentStatus.FAILED:
            return self._fail(update.message)
        if update.status is SegmentStatus.REACHED:
            if self.phase is PlacePhase.PREPLACE:
                self.phase = PlacePhase.DESCEND
                self._segment = CartesianSegment(
                    self.runtime,
                    self._target_tcp,
                    self.config.motion,
                )
            elif self.phase is PlacePhase.DESCEND:
                self.phase = PlacePhase.OPEN
                self._gripper = OpenGripperSkill(
                    self.runtime,
                    config=self.config.gripper,
                )
            else:
                self.phase = PlacePhase.VERIFY
            return self.tick()
        assert update.action is not None
        return self._step(update.action)

    def _tick_open(self) -> SkillStep:
        assert self._gripper is not None
        update = self._gripper.tick()
        if update.done:
            if not update.succeeded:
                return self._fail(update.message)
            retreat_position = (
                self.runtime.tcp_pose_w()[0]
                - self.direction_w * self.config.retreat_distance_m
            )
            retreat = (retreat_position, self.runtime.tcp_pose_w()[1].clone())
            self.phase = PlacePhase.RETREAT
            self._segment = CartesianSegment(
                self.runtime,
                retreat,
                self.config.motion,
            )
        return self._step(update.action)

    def _tick_verify(self) -> SkillStep:
        position, orientation = pose_error(
            self.runtime.object_pose_w(self.object_name),
            self.target_object_pose_w,
        )
        within_tolerance = (
            position <= self.config.object_position_tolerance_m
            and orientation <= self.config.object_orientation_tolerance_rad
        )
        self._stable_elapsed_s = (
            self._stable_elapsed_s + self.runtime.step_dt
            if within_tolerance
            else 0.0
        )
        self._verify_elapsed_s += self.runtime.step_dt
        if self._stable_elapsed_s >= self.config.verify_duration_s:
            self.phase = PlacePhase.SUCCEEDED
            self.message = (
                f"object placed with {position:.4f} m position error and "
                f"{orientation:.4f} rad orientation error"
            )
        elif self._verify_elapsed_s >= self.config.verify_timeout_s:
            return self._fail(
                "placed object did not settle at the target: "
                f"position error {position:.4f} m, "
                f"orientation error {orientation:.4f} rad"
            )
        return self._step(self.runtime.hold_action(True))

    def _fail(self, message: str) -> SkillStep:
        self.phase = PlacePhase.FAILED
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


__all__ = ["PlaceConfig", "PlacePhase", "PlaceSkill"]
