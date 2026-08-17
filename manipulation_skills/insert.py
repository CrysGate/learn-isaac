"""Robot-independent straight-line insertion operation."""

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
    finite_positive,
    normalized_direction,
    pose_error,
    relative_pose,
    validate_pose,
)
from .gripper import GripperConfig, OpenGripperSkill


def _default_insert_motion() -> CartesianMotionConfig:
    return CartesianMotionConfig(linear_speed_m_s=0.05)


@dataclass(frozen=True)
class InsertConfig:
    preinsert_distance_m: float = 0.08
    retreat_distance_m: float = 0.08
    release_after_insert: bool = True
    verify_duration_s: float = 0.25
    verify_timeout_s: float = 2.0
    object_position_tolerance_m: float = 0.015
    object_orientation_tolerance_rad: float = 0.12
    motion: CartesianMotionConfig = field(default_factory=_default_insert_motion)
    gripper: GripperConfig = field(default_factory=GripperConfig)

    def __post_init__(self) -> None:
        for name in (
            "preinsert_distance_m",
            "retreat_distance_m",
            "verify_duration_s",
            "verify_timeout_s",
            "object_position_tolerance_m",
            "object_orientation_tolerance_rad",
        ):
            finite_positive(getattr(self, name), name)
        if self.verify_duration_s > self.verify_timeout_s:
            raise ValueError("verify_duration_s cannot exceed verify_timeout_s")


class InsertPhase(str, Enum):
    PREINSERT = "preinsert"
    INSERT = "insert"
    RELEASE = "release"
    RETREAT = "retreat"
    VERIFY = "verify"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class InsertSkill:
    """Approach along one axis, insert a grasped object, and optionally release."""

    def __init__(
        self,
        runtime: ManipulationRuntime,
        object_name: str,
        target_object_pose_w: Pose,
        insertion_direction_w: torch.Tensor,
        *,
        config: InsertConfig | None = None,
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
        self.direction_w = normalized_direction(
            insertion_direction_w.to(target_object_pose_w[0]),
            "insertion_direction_w",
        )
        self.config = config or InsertConfig()
        self.phase = InsertPhase.PREINSERT
        self.message = ""
        object_to_tcp = relative_pose(
            runtime.object_pose_w(object_name),
            runtime.tcp_pose_w(),
        )
        self._target_tcp = compose_pose(self.target_object_pose_w, object_to_tcp)
        preinsert_position = (
            self._target_tcp[0]
            - self.direction_w * self.config.preinsert_distance_m
        )
        self._preinsert_tcp = (preinsert_position, self._target_tcp[1].clone())
        self._segment = CartesianSegment(
            runtime,
            self._preinsert_tcp,
            self.config.motion,
        )
        self._gripper: OpenGripperSkill | None = None
        self._verify_elapsed_s = 0.0
        self._stable_elapsed_s = 0.0

    @property
    def done(self) -> bool:
        return self.phase in {InsertPhase.SUCCEEDED, InsertPhase.FAILED}

    @property
    def succeeded(self) -> bool:
        return self.phase is InsertPhase.SUCCEEDED

    def tick(self) -> SkillStep:
        if self.done:
            return self._step(
                self.runtime.hold_action(
                    True if self.config.release_after_insert else False
                )
            )
        if self.phase in {
            InsertPhase.PREINSERT,
            InsertPhase.INSERT,
            InsertPhase.RETREAT,
        }:
            return self._tick_motion()
        if self.phase is InsertPhase.RELEASE:
            return self._tick_release()
        return self._tick_verify()

    def _tick_motion(self) -> SkillStep:
        gripper_open = self.phase is InsertPhase.RETREAT
        update = self._segment.tick(gripper_open)
        if update.status is SegmentStatus.FAILED:
            return self._fail(update.message)
        if update.status is SegmentStatus.REACHED:
            if self.phase is InsertPhase.PREINSERT:
                self.phase = InsertPhase.INSERT
                self._segment = CartesianSegment(
                    self.runtime,
                    self._target_tcp,
                    self.config.motion,
                )
            elif self.phase is InsertPhase.INSERT:
                if self.config.release_after_insert:
                    self.phase = InsertPhase.RELEASE
                    self._gripper = OpenGripperSkill(
                        self.runtime,
                        config=self.config.gripper,
                    )
                else:
                    self.phase = InsertPhase.VERIFY
            else:
                self.phase = InsertPhase.VERIFY
            return self.tick()
        assert update.action is not None
        return self._step(update.action)

    def _tick_release(self) -> SkillStep:
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
            self.phase = InsertPhase.RETREAT
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
            self.phase = InsertPhase.SUCCEEDED
            self.message = (
                f"object inserted with {position:.4f} m position error and "
                f"{orientation:.4f} rad orientation error"
            )
        elif self._verify_elapsed_s >= self.config.verify_timeout_s:
            return self._fail(
                "inserted object did not remain at the target: "
                f"position error {position:.4f} m, "
                f"orientation error {orientation:.4f} rad"
            )
        return self._step(
            self.runtime.hold_action(
                True if self.config.release_after_insert else False
            )
        )

    def _fail(self, message: str) -> SkillStep:
        self.phase = InsertPhase.FAILED
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


__all__ = ["InsertConfig", "InsertPhase", "InsertSkill"]
