"""Immutable commands containing fully planned robot motion."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TypeAlias

from .context import JointState, JointTrajectory
from .models import Arm, Pose


@dataclass(frozen=True, slots=True)
class MoveToPose:
    arm: Arm
    target_tcp_pose_env: Pose
    trajectory: JointTrajectory
    label: str


@dataclass(frozen=True, slots=True)
class MoveToJoints:
    arm: Arm
    target_joint_state: JointState
    trajectory: JointTrajectory
    label: str


@dataclass(frozen=True, slots=True)
class SetGripper:
    arm: Arm
    closed: bool
    label: str
    steps: int = 8


@dataclass(frozen=True, slots=True)
class Hold:
    steps: int
    label: str


SkillCommand: TypeAlias = MoveToPose | MoveToJoints | SetGripper | Hold


__all__ = ["Hold", "MoveToJoints", "MoveToPose", "SetGripper", "SkillCommand"]
