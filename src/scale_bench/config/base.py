"""Shared immutable model base and constrained configuration value types."""

from __future__ import annotations

import math
from typing import Annotated, Literal, TypeAlias
from pydantic import BaseModel, ConfigDict, Field, StrictInt


FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
NonNegativeFloat = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]
PositiveFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
UnitIntervalFloat = Annotated[float, Field(ge=0.0, le=1.0, allow_inf_nan=False)]
NonNegativeInt = Annotated[StrictInt, Field(ge=0)]
PositiveInt = Annotated[StrictInt, Field(gt=0)]
Name = Annotated[str, Field(min_length=1)]
AssetReference = Annotated[str, Field(min_length=1, json_schema_extra={"path_kind": "asset"})]
OptionalAssetReference = Annotated[str | None, Field(json_schema_extra={"path_kind": "asset"})]
ConfigReference = Annotated[str, Field(min_length=1, json_schema_extra={"path_kind": "config"})]
CameraConvention: TypeAlias = Literal["opengl", "ros", "world"]

Position2: TypeAlias = tuple[FiniteFloat, FiniteFloat]
Position3: TypeAlias = tuple[FiniteFloat, FiniteFloat, FiniteFloat]
Quaternion: TypeAlias = tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]

class FrozenModel(BaseModel):
    """Base for configuration data with strict fields and immutable attributes."""

    model_config = ConfigDict(extra="forbid", frozen=True)


def require_unique(names: tuple[str, ...], label: str) -> None:
    """Reject duplicate semantic names while preserving declared ordering."""

    if len(names) != len(set(names)):
        raise ValueError(f"{label} contains duplicate names")


def require_unit_quaternion(value: Quaternion, field_name: str) -> None:
    """Validate an XYZW quaternion using the project-wide tolerance."""

    norm = math.sqrt(sum(component * component for component in value))
    if not math.isclose(norm, 1.0, abs_tol=1.0e-6):
        raise ValueError(f"{field_name} must be a unit quaternion")
