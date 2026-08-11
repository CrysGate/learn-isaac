"""Robot-independent state machine for one pick-and-lift operation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Protocol, TypeAlias

import torch
from isaaclab.utils.math import (
    combine_frame_transforms,
    quat_apply,
    quat_slerp,
)


Pose: TypeAlias = tuple[torch.Tensor, torch.Tensor]


def _finite_tuple(values: tuple[float, ...], size: int, name: str) -> None:
    if len(values) != size or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain {size} finite values")


@dataclass(frozen=True)
class GraspCandidate:
    """TCP grasp pose in the object frame and its local approach axis."""

    position_object_m: tuple[float, float, float]
    orientation_object_xyzw: tuple[float, float, float, float]
    approach_axis_tcp: tuple[float, float, float] = (1.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        _finite_tuple(self.position_object_m, 3, "position_object_m")
        _finite_tuple(self.orientation_object_xyzw, 4, "orientation_object_xyzw")
        _finite_tuple(self.approach_axis_tcp, 3, "approach_axis_tcp")
        if not math.isclose(
            math.sqrt(sum(value * value for value in self.orientation_object_xyzw)),
            1.0,
            abs_tol=1.0e-5,
        ):
            raise ValueError("orientation_object_xyzw must be a unit quaternion")
        if not math.isclose(
            math.sqrt(sum(value * value for value in self.approach_axis_tcp)),
            1.0,
            abs_tol=1.0e-5,
        ):
            raise ValueError("approach_axis_tcp must be a unit vector")


@dataclass(frozen=True)
class PickConfig:
    """Motion, timing, and success thresholds for :class:`PickSkill`."""

    approach_distance_m: float = 0.10
    lift_distance_m: float = 0.10
    default_grasp_height_offset_m: float = 0.005
    linear_speed_m_s: float = 0.15
    angular_speed_rad_s: float = 1.0
    open_duration_s: float = 0.35
    open_timeout_s: float = 1.5
    close_duration_s: float = 0.75
    verify_duration_s: float = 0.30
    move_settle_timeout_s: float = 5.0
    position_tolerance_m: float = 0.008
    orientation_tolerance_rad: float = 0.08
    gripper_tolerance_m: float = 0.003
    minimum_grasp_aperture_m: float = 0.002
    lift_tolerance_m: float = 0.015
    max_joint_step_rad: float = 0.04

    def __post_init__(self) -> None:
        positive = (
            "approach_distance_m",
            "lift_distance_m",
            "linear_speed_m_s",
            "angular_speed_rad_s",
            "open_timeout_s",
            "close_duration_s",
            "verify_duration_s",
            "move_settle_timeout_s",
            "position_tolerance_m",
            "orientation_tolerance_rad",
            "gripper_tolerance_m",
            "lift_tolerance_m",
            "max_joint_step_rad",
        )
        for name in positive:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.open_duration_s < 0.0 or self.open_duration_s > self.open_timeout_s:
            raise ValueError("open_duration_s must be between zero and open_timeout_s")
        if self.minimum_grasp_aperture_m < 0.0:
            raise ValueError("minimum_grasp_aperture_m cannot be negative")


@dataclass(frozen=True)
class ResolvedGrasp:
    """A world-space grasp resolved by a runtime adapter."""

    pose_w: Pose
    approach_direction_w: torch.Tensor


class PickRuntime(Protocol):
    """Small runtime boundary required by the pick state machine."""

    step_dt: float
    max_gripper_aperture_m: float

    def hold_action(self, gripper_open: bool | None = None) -> torch.Tensor: ...

    def move_action(self, target_tcp_pose_w: Pose, gripper_open: bool) -> torch.Tensor: ...

    def tcp_pose_w(self) -> Pose: ...

    def object_pose_w(self, object_name: str) -> Pose: ...

    def gripper_aperture_m(self) -> float: ...

    def default_grasp(
        self,
        object_name: str,
        height_offset_m: float,
    ) -> ResolvedGrasp: ...


class PickPhase(str, Enum):
    OPEN = "open"
    PREGRASP = "pregrasp"
    APPROACH = "approach"
    CLOSE = "close"
    LIFT = "lift"
    VERIFY = "verify"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


@dataclass(frozen=True)
class PickStep:
    """One environment action plus the state reached while producing it."""

    action: torch.Tensor
    phase: PickPhase
    done: bool
    succeeded: bool
    message: str = ""


class PickSkill:
    """Open, approach, grasp, lift, and verify one rigid object."""

    def __init__(
        self,
        runtime: PickRuntime,
        object_name: str,
        *,
        grasp: GraspCandidate | None = None,
        config: PickConfig | None = None,
    ) -> None:
        if not object_name:
            raise ValueError("object_name cannot be empty")
        self.runtime = runtime
        self.object_name = object_name
        self.grasp = grasp
        self.config = config or PickConfig()
        self.phase = PickPhase.OPEN
        self.message = ""
        self._elapsed_s = 0.0
        self._move_start: Pose | None = None
        self._move_target: Pose | None = None
        self._move_duration_s = 0.0
        self._resolved_grasp: ResolvedGrasp | None = None
        self._object_z_before_lift: float | None = None

    @property
    def done(self) -> bool:
        return self.phase in {PickPhase.SUCCEEDED, PickPhase.FAILED}

    @property
    def succeeded(self) -> bool:
        return self.phase is PickPhase.SUCCEEDED

    def tick(self) -> PickStep:
        """Return the action for the next environment step."""

        if self.done:
            return self._step(self.runtime.hold_action(False if self.succeeded else None))

        try:
            for _ in range(4):
                if self.phase is PickPhase.OPEN:
                    action = self._tick_open()
                elif self.phase in {
                    PickPhase.PREGRASP,
                    PickPhase.APPROACH,
                    PickPhase.LIFT,
                }:
                    action = self._tick_move()
                elif self.phase is PickPhase.CLOSE:
                    action = self._tick_close()
                else:
                    action = self._tick_verify()
                if self.done:
                    return self._step(
                        self.runtime.hold_action(
                            False if self.succeeded else None
                        )
                    )
                if action is not None:
                    self._elapsed_s += self.runtime.step_dt
                    return self._step(action)
        except (RuntimeError, ValueError) as error:
            self._fail(str(error))

        if not self.done:
            self._fail("pick state machine exceeded its transition limit")
        return self._step(self.runtime.hold_action())

    def _tick_open(self) -> torch.Tensor | None:
        aperture = self.runtime.gripper_aperture_m()
        opened = (
            aperture
            >= self.runtime.max_gripper_aperture_m
            - self.config.gripper_tolerance_m
        )
        if self._elapsed_s >= self.config.open_duration_s and opened:
            self._resolved_grasp = self._resolve_grasp()
            target_pos, target_quat = self._resolved_grasp.pose_w
            pregrasp_pos = (
                target_pos
                - self._resolved_grasp.approach_direction_w
                * self.config.approach_distance_m
            )
            self._start_move(PickPhase.PREGRASP, (pregrasp_pos, target_quat))
            return None
        if self._elapsed_s >= self.config.open_timeout_s:
            self._fail(
                f"gripper opened to {aperture:.4f} m; expected "
                f"{self.runtime.max_gripper_aperture_m:.4f} m"
            )
            return None
        return self.runtime.hold_action(True)

    def _tick_move(self) -> torch.Tensor | None:
        assert self._move_start is not None and self._move_target is not None
        if self._elapsed_s >= self._move_duration_s:
            position_error, orientation_error = _pose_error(
                self.runtime.tcp_pose_w(),
                self._move_target,
            )
            if (
                position_error <= self.config.position_tolerance_m
                and orientation_error <= self.config.orientation_tolerance_rad
            ):
                if self.phase is PickPhase.PREGRASP:
                    self._resolved_grasp = self._resolve_grasp()
                    self._start_move(
                        PickPhase.APPROACH,
                        self._resolved_grasp.pose_w,
                    )
                elif self.phase is PickPhase.APPROACH:
                    self.phase = PickPhase.CLOSE
                    self._elapsed_s = 0.0
                else:
                    self.phase = PickPhase.VERIFY
                    self._elapsed_s = 0.0
                return None
            if (
                self._elapsed_s
                >= self._move_duration_s + self.config.move_settle_timeout_s
            ):
                self._fail(
                    f"{self.phase.value} did not converge: position error "
                    f"{position_error:.4f} m, orientation error "
                    f"{orientation_error:.4f} rad"
                )
                return None

        tau = min(1.0, (self._elapsed_s + self.runtime.step_dt) / self._move_duration_s)
        target = _interpolate_pose(self._move_start, self._move_target, tau)
        return self.runtime.move_action(
            target,
            gripper_open=self.phase in {PickPhase.PREGRASP, PickPhase.APPROACH},
        )

    def _tick_close(self) -> torch.Tensor | None:
        assert self._resolved_grasp is not None
        if self._elapsed_s >= self.config.close_duration_s:
            aperture = self.runtime.gripper_aperture_m()
            if aperture < self.config.minimum_grasp_aperture_m:
                self._fail(
                    f"gripper fully closed at {aperture:.4f} m without "
                    "capturing an object"
                )
                return None
            object_pos, _ = self.runtime.object_pose_w(self.object_name)
            self._object_z_before_lift = float(object_pos[2].item())
            tcp_pos, tcp_quat = self.runtime.tcp_pose_w()
            lift_pos = tcp_pos.clone()
            lift_pos[2] += self.config.lift_distance_m
            self._start_move(PickPhase.LIFT, (lift_pos, tcp_quat))
            return None
        return self.runtime.move_action(
            self._resolved_grasp.pose_w,
            gripper_open=False,
        )

    def _tick_verify(self) -> torch.Tensor | None:
        assert self._move_target is not None
        if self._elapsed_s >= self.config.verify_duration_s:
            assert self._object_z_before_lift is not None
            object_pos, _ = self.runtime.object_pose_w(self.object_name)
            lifted = float(object_pos[2].item()) - self._object_z_before_lift
            required = self.config.lift_distance_m - self.config.lift_tolerance_m
            if lifted < required:
                self._fail(
                    f"object lifted {lifted:.4f} m; expected at least "
                    f"{required:.4f} m"
                )
            else:
                self.phase = PickPhase.SUCCEEDED
                self.message = f"object lifted {lifted:.4f} m"
            return None
        return self.runtime.move_action(self._move_target, gripper_open=False)

    def _resolve_grasp(self) -> ResolvedGrasp:
        if self.grasp is None:
            return self.runtime.default_grasp(
                self.object_name,
                self.config.default_grasp_height_offset_m,
            )

        object_pos, object_quat = self.runtime.object_pose_w(self.object_name)
        grasp_pos_object = object_pos.new_tensor(self.grasp.position_object_m)
        grasp_quat_object = object_quat.new_tensor(
            self.grasp.orientation_object_xyzw
        )
        grasp_pos, grasp_quat = combine_frame_transforms(
            object_pos.unsqueeze(0),
            object_quat.unsqueeze(0),
            grasp_pos_object.unsqueeze(0),
            grasp_quat_object.unsqueeze(0),
        )
        axis_tcp = object_pos.new_tensor(self.grasp.approach_axis_tcp)
        direction = quat_apply(grasp_quat, axis_tcp.unsqueeze(0))[0]
        return ResolvedGrasp((grasp_pos[0], grasp_quat[0]), direction)

    def _start_move(self, phase: PickPhase, target: Pose) -> None:
        start = self.runtime.tcp_pose_w()
        distance = torch.linalg.vector_norm(target[0] - start[0]).item()
        angle = _orientation_error(start[1], target[1])
        self.phase = phase
        self._elapsed_s = 0.0
        self._move_start = (start[0].clone(), start[1].clone())
        self._move_target = (target[0].clone(), target[1].clone())
        self._move_duration_s = max(
            self.runtime.step_dt,
            distance / self.config.linear_speed_m_s,
            angle / self.config.angular_speed_rad_s,
        )

    def _fail(self, message: str) -> None:
        self.phase = PickPhase.FAILED
        self.message = message

    def _step(self, action: torch.Tensor) -> PickStep:
        return PickStep(
            action=action,
            phase=self.phase,
            done=self.done,
            succeeded=self.succeeded,
            message=self.message,
        )


def _interpolate_pose(start: Pose, target: Pose, tau: float) -> Pose:
    position = start[0] + tau * (target[0] - start[0])
    orientation = quat_slerp(start[1], target[1], tau)
    return position, orientation


def _orientation_error(actual: torch.Tensor, target: torch.Tensor) -> float:
    dot = torch.dot(actual, target).abs().clamp(max=1.0)
    return float((2.0 * torch.acos(dot)).item())


def _pose_error(actual: Pose, target: Pose) -> tuple[float, float]:
    position = float(torch.linalg.vector_norm(target[0] - actual[0]).item())
    return position, _orientation_error(actual[1], target[1])


__all__ = [
    "GraspCandidate",
    "PickConfig",
    "PickPhase",
    "PickRuntime",
    "PickSkill",
    "PickStep",
    "Pose",
    "ResolvedGrasp",
]
