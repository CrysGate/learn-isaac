"""Expected manipulation planning and execution failures."""

from __future__ import annotations

from .models import Arm


class SkillError(RuntimeError):
    """A failure that should terminate only the affected episode."""


class PlanningError(SkillError):
    """A failed motion segment with its arm and operation stage."""

    def __init__(self, arm: Arm, stage: str, reason: str) -> None:
        self.arm = arm
        self.stage = stage
        self.reason = reason
        super().__init__(f"{arm} arm planning failed at {stage}: {reason}")


__all__ = ["PlanningError", "SkillError"]
