"""Configuration models for the single-object pick-and-place task."""

from __future__ import annotations

from scale_bench.config.base import (
    Name,
    Position2,
)
from scale_bench.tasks.common.rigid_object import (
    RigidObjectAssetConfig,
    RigidObjectTaskConfig,
    TargetPlacementConfig,
)


class PickObjectConfig(RigidObjectAssetConfig):
    """The one rigid object manipulated by every episode."""

    name: Name


class TargetSlotConfig(TargetPlacementConfig):
    """Fixed tabletop destination and its success tolerances."""

    position_xy_m: Position2


class SingleObjectPickAndPlaceConfig(RigidObjectTaskConfig):
    """Typed configuration for one random-source, fixed-target transfer."""

    object: PickObjectConfig
    target_slot: TargetSlotConfig


__all__ = [
    "PickObjectConfig",
    "SingleObjectPickAndPlaceConfig",
    "TargetSlotConfig",
]
