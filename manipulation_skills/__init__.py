"""Atomic manipulation skills kept outside the benchmark runtime package."""

from typing import Any

from .pick import GraspCandidate, PickConfig, PickPhase, PickSkill, PickStep


def pick(*args: Any, **kwargs: Any) -> PickSkill:
    """Load the Piper adapter only after Isaac Sim has been launched."""

    from .piper import pick as piper_pick

    return piper_pick(*args, **kwargs)

__all__ = [
    "GraspCandidate",
    "PickConfig",
    "PickPhase",
    "PickSkill",
    "PickStep",
    "pick",
]
