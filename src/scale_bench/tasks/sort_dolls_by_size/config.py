"""Configuration models for the sort-dolls-by-size task."""

from __future__ import annotations

from typing import Annotated, Self

from pydantic import Field, field_validator, model_validator

from scale_bench.config.base import FiniteFloat
from scale_bench.tasks.common.rigid_object import (
    RigidObjectAssetConfig,
    RigidObjectTaskConfig,
    TargetPlacementConfig,
)


AssetId = Annotated[str, Field(pattern=r"^\d{5}$")]


class DollAssetConfig(RigidObjectAssetConfig):
    """One nesting-doll asset."""

    asset_id: AssetId


class TargetSlotsConfig(TargetPlacementConfig):
    """Fixed tabletop slots ordered in the positive Y direction."""

    x_positions_m: tuple[FiniteFloat, ...] = Field(min_length=2)
    y_positions_m: tuple[FiniteFloat, ...] = Field(min_length=2)

    @field_validator("y_positions_m")
    @classmethod
    def _validate_y_positions(
        cls,
        value: tuple[float, ...],
    ) -> tuple[float, ...]:
        if any(left >= right for left, right in zip(value, value[1:])):
            raise ValueError("y_positions_m must be strictly increasing")
        return value

    @model_validator(mode="after")
    def _validate_slot_counts(self) -> Self:
        if len(self.x_positions_m) != len(self.y_positions_m):
            raise ValueError("x_positions_m and y_positions_m must have equal length")
        return self


class SortDollsBySizeConfig(RigidObjectTaskConfig):
    """Typed task configuration and nesting-doll constraints."""

    dolls: tuple[DollAssetConfig, ...] = Field(min_length=2)
    target_slots: TargetSlotsConfig

    @model_validator(mode="after")
    def _validate_dolls(self) -> Self:
        ids = tuple(doll.asset_id for doll in self.dolls)
        if len(ids) != len(set(ids)):
            raise ValueError("doll asset_id values must be unique")
        if len(ids) != len(self.target_slots.y_positions_m):
            raise ValueError(
                "the number of dolls must match the number of target slots"
            )
        return self


__all__ = ["DollAssetConfig", "SortDollsBySizeConfig", "TargetSlotsConfig"]
