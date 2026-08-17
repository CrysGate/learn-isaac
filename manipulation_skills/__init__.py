"""Atomic manipulation skills kept outside the benchmark runtime package."""

from typing import Any

from .core import CartesianMotionConfig, Pose, SkillStep
from .gripper import (
    CloseGripperSkill,
    GripperConfig,
    GripperPhase,
    GripperSkill,
    OpenGripperSkill,
)
from .goals import GoalResult, JointPositionGoal, LiftGoal, ObjectPoseGoal
from .insert import InsertConfig, InsertPhase, InsertSkill
from .motion import HomeConfig, HomePhase, HomeSkill, MovePhase, MoveToPoseSkill
from .pick import GraspCandidate, PickConfig, PickPhase, PickSkill, PickStep
from .place import PlaceConfig, PlacePhase, PlaceSkill
from .rotate import RotateConfig, RotatePhase, RotateSkill
from .sequence import SkillFactory, SkillSequence


def _piper_factory(name: str, *args: Any, **kwargs: Any) -> Any:
    from . import piper

    return getattr(piper, name)(*args, **kwargs)


def pick(*args: Any, **kwargs: Any) -> PickSkill:
    """Load the Piper adapter only after Isaac Sim has been launched."""

    return _piper_factory("pick", *args, **kwargs)


def move_to_pose(*args: Any, **kwargs: Any) -> MoveToPoseSkill:
    return _piper_factory("move_to_pose", *args, **kwargs)


def open_gripper(*args: Any, **kwargs: Any) -> OpenGripperSkill:
    return _piper_factory("open_gripper", *args, **kwargs)


def close_gripper(*args: Any, **kwargs: Any) -> CloseGripperSkill:
    return _piper_factory("close_gripper", *args, **kwargs)


def place(*args: Any, **kwargs: Any) -> PlaceSkill:
    return _piper_factory("place", *args, **kwargs)


def insert(*args: Any, **kwargs: Any) -> InsertSkill:
    return _piper_factory("insert", *args, **kwargs)


def rotate(*args: Any, **kwargs: Any) -> RotateSkill:
    return _piper_factory("rotate", *args, **kwargs)


def home(*args: Any, **kwargs: Any) -> HomeSkill:
    return _piper_factory("home", *args, **kwargs)

__all__ = [
    "CartesianMotionConfig",
    "CloseGripperSkill",
    "GraspCandidate",
    "GripperConfig",
    "GripperPhase",
    "GripperSkill",
    "GoalResult",
    "HomeConfig",
    "HomePhase",
    "HomeSkill",
    "InsertConfig",
    "InsertPhase",
    "InsertSkill",
    "JointPositionGoal",
    "LiftGoal",
    "MovePhase",
    "MoveToPoseSkill",
    "OpenGripperSkill",
    "ObjectPoseGoal",
    "PickConfig",
    "PickPhase",
    "PickSkill",
    "PickStep",
    "PlaceConfig",
    "PlacePhase",
    "PlaceSkill",
    "Pose",
    "RotateConfig",
    "RotatePhase",
    "RotateSkill",
    "SkillFactory",
    "SkillSequence",
    "SkillStep",
    "close_gripper",
    "home",
    "insert",
    "move_to_pose",
    "open_gripper",
    "pick",
    "place",
    "rotate",
]
