"""Configuration models for the sort-dolls-by-size task."""

from __future__ import annotations

from typing import Annotated, Self

from pydantic import Field, model_validator

from scale_bench.tasks.common.rigid_object import (
    RigidObjectAssetConfig,
    RigidObjectTaskConfig,
)


AssetId = Annotated[str, Field(pattern=r"^\d{5}$")]


class DollAssetConfig(RigidObjectAssetConfig):
    """One nesting-doll asset."""

    asset_id: AssetId


class SortDollsBySizeConfig(RigidObjectTaskConfig):
    """Typed task configuration and nesting-doll constraints."""

    dolls: tuple[DollAssetConfig, ...] = Field(min_length=2)

    @model_validator(mode="after")
    def _validate_dolls(self) -> Self:
        ids = tuple(doll.asset_id for doll in self.dolls)
        if len(ids) != len(set(ids)):
            raise ValueError("doll asset_id values must be unique")
        return self


__all__ = ["DollAssetConfig", "SortDollsBySizeConfig"]
