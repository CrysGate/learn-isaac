"""Atomic open and close skills for a parallel-jaw gripper."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import torch

from .core import (
    ManipulationRuntime,
    SkillStep,
    finite_non_negative,
    finite_positive,
)


@dataclass(frozen=True)
class GripperConfig:
    minimum_duration_s: float = 0.35
    timeout_s: float = 1.5
    aperture_tolerance_m: float = 0.003
    minimum_contact_aperture_m: float = 0.002

    def __post_init__(self) -> None:
        finite_non_negative(self.minimum_duration_s, "minimum_duration_s")
        finite_positive(self.timeout_s, "timeout_s")
        finite_positive(self.aperture_tolerance_m, "aperture_tolerance_m")
        finite_non_negative(
            self.minimum_contact_aperture_m,
            "minimum_contact_aperture_m",
        )
        if self.minimum_duration_s > self.timeout_s:
            raise ValueError("minimum_duration_s cannot exceed timeout_s")


class GripperPhase(str, Enum):
    COMMAND = "command"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class GripperSkill:
    """Command one gripper state and verify aperture feedback."""

    def __init__(
        self,
        runtime: ManipulationRuntime,
        opened: bool,
        *,
        require_contact: bool = False,
        config: GripperConfig | None = None,
    ) -> None:
        self.runtime = runtime
        self.opened = opened
        self.require_contact = require_contact
        self.config = config or GripperConfig()
        self.phase = GripperPhase.COMMAND
        self.message = ""
        self._elapsed_s = 0.0

    @property
    def done(self) -> bool:
        return self.phase in {GripperPhase.SUCCEEDED, GripperPhase.FAILED}

    @property
    def succeeded(self) -> bool:
        return self.phase is GripperPhase.SUCCEEDED

    def tick(self) -> SkillStep:
        if self.done:
            return self._step(self.runtime.hold_action(self.opened))

        aperture = self.runtime.gripper_aperture_m()
        if self._elapsed_s >= self.config.minimum_duration_s:
            reached = (
                aperture
                >= self.runtime.max_gripper_aperture_m
                - self.config.aperture_tolerance_m
                if self.opened
                else aperture <= self.config.aperture_tolerance_m
            )
            contact = (
                not self.opened
                and aperture >= self.config.minimum_contact_aperture_m
            )
            command_succeeded = (
                reached
                if self.opened
                else contact
                if self.require_contact
                else reached or contact
            )
            if command_succeeded:
                self.phase = GripperPhase.SUCCEEDED
                return self._step(self.runtime.hold_action(self.opened))

        if self._elapsed_s >= self.config.timeout_s:
            self.phase = GripperPhase.FAILED
            expectation = (
                f"open to {self.runtime.max_gripper_aperture_m:.4f} m"
                if self.opened
                else "close"
            )
            self.message = (
                f"gripper failed to {expectation}; aperture is {aperture:.4f} m"
            )
            return self._step(self.runtime.hold_action())

        self._elapsed_s += self.runtime.step_dt
        return self._step(self.runtime.hold_action(self.opened))

    def _step(self, action: torch.Tensor) -> SkillStep:
        return SkillStep(
            action,
            self.phase.value,
            self.done,
            self.succeeded,
            self.message,
        )


class OpenGripperSkill(GripperSkill):
    def __init__(
        self,
        runtime: ManipulationRuntime,
        *,
        config: GripperConfig | None = None,
    ) -> None:
        super().__init__(runtime, True, config=config)


class CloseGripperSkill(GripperSkill):
    def __init__(
        self,
        runtime: ManipulationRuntime,
        *,
        require_contact: bool = False,
        config: GripperConfig | None = None,
    ) -> None:
        super().__init__(
            runtime,
            False,
            require_contact=require_contact,
            config=config,
        )


__all__ = [
    "CloseGripperSkill",
    "GripperConfig",
    "GripperPhase",
    "GripperSkill",
    "OpenGripperSkill",
]
