"""Typed scene-level configuration loaded from YAML."""

from __future__ import annotations

import math
from functools import cache
from pathlib import Path
from typing import Annotated, Self

import yaml
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    StrictBool,
    StrictInt,
    ValidationError,
    field_validator,
    model_validator,
)

from scale_bench import REPOSITORY_ROOT
from scale_bench.sensors import CameraConvention


DEFAULT_SCENE_CONFIG_PATH = REPOSITORY_ROOT / "configs/scene/default.yml"
FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
NonNegativeFloat = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]
PositiveFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
UnitIntervalFloat = Annotated[float, Field(ge=0.0, le=1.0, allow_inf_nan=False)]
PositiveInt = Annotated[StrictInt, Field(gt=0)]
Name = Annotated[str, Field(min_length=1)]

Position2 = tuple[FiniteFloat, FiniteFloat]
Position3 = tuple[FiniteFloat, FiniteFloat, FiniteFloat]
Quaternion = tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]


class _SceneModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


def _require_unit_quaternion(value: Quaternion, *, field_name: str) -> None:
    norm = math.sqrt(sum(component * component for component in value))
    if not math.isclose(norm, 1.0, abs_tol=1.0e-6):
        raise ValueError(f"{field_name} must be a unit quaternion")


class RoomConfig(_SceneModel):
    """Room USD and uniform scale."""

    usd_path: Name
    scale: PositiveFloat = 0.5


class SurfaceConfig(_SceneModel):
    """A textured, collidable cuboid used for the ground or table."""

    position_m: Position3
    size_m: tuple[PositiveFloat, PositiveFloat, PositiveFloat]
    material_path: Name | None
    uv_scale: tuple[PositiveFloat, PositiveFloat] = (1.0, 1.0)
    static_friction: NonNegativeFloat
    dynamic_friction: NonNegativeFloat
    restitution: UnitIntervalFloat


class RobotMountConfig(_SceneModel):
    """Robot base pose on the table-height plane."""

    position_xy_m: Position2
    orientation_xyzw: Quaternion

    @model_validator(mode="after")
    def _validate_orientation(self) -> Self:
        _require_unit_quaternion(
            self.orientation_xyzw,
            field_name="orientation_xyzw",
        )
        return self


class RobotMountsConfig(_SceneModel):
    """Required left and right robot mounts."""

    left: RobotMountConfig
    right: RobotMountConfig


class OverheadCameraConfig(_SceneModel):
    """Camera profile and its scene-local stand and sensor poses."""

    profile_path: Name
    stand_usd_path: Name
    stand_position_xy_m: Position2
    stand_orientation_xyzw: Quaternion
    sensor_local_position_m: Position3
    sensor_local_orientation_xyzw: Quaternion
    convention: CameraConvention = "opengl"

    @model_validator(mode="after")
    def _validate_orientations(self) -> Self:
        _require_unit_quaternion(
            self.stand_orientation_xyzw,
            field_name="stand_orientation_xyzw",
        )
        _require_unit_quaternion(
            self.sensor_local_orientation_xyzw,
            field_name="sensor_local_orientation_xyzw",
        )
        return self


class LightingConfig(_SceneModel):
    """Environment texture and dome-light intensity."""

    texture_path: Name
    intensity: NonNegativeFloat


class SceneRuntimeConfig(_SceneModel):
    """Interactive-scene cloning settings."""

    num_envs: PositiveInt
    env_spacing_m: PositiveFloat
    replicate_physics: StrictBool
    clone_in_fabric: StrictBool


class TaskObjectPlacementArea(_SceneModel):
    """Axis-aligned XY bounds in the environment-local frame."""

    x_range_m: tuple[FiniteFloat, FiniteFloat]
    y_range_m: tuple[FiniteFloat, FiniteFloat]

    @field_validator("x_range_m", "y_range_m")
    @classmethod
    def _validate_range(cls, value: tuple[float, float]) -> tuple[float, float]:
        if value[0] >= value[1]:
            raise ValueError("lower bound must be less than upper bound")
        return value


class SceneConfig(_SceneModel):
    """Top-level scene preset with typed scene-wide metadata."""

    room: RoomConfig
    ground: SurfaceConfig
    table: SurfaceConfig
    task_object_placement_area: TaskObjectPlacementArea
    robot_mounts: RobotMountsConfig
    camera: OverheadCameraConfig
    lighting: LightingConfig
    runtime: SceneRuntimeConfig

    @property
    def table_top_z_m(self) -> float:
        return self.table.position_m[2] + self.table.size_m[2] / 2.0

    @classmethod
    @cache
    def load(cls, config_path: str | Path = DEFAULT_SCENE_CONFIG_PATH) -> Self:
        """Load a scene preset relative to the repository root."""

        path = Path(config_path)
        if not path.is_absolute():
            path = REPOSITORY_ROOT / path
        try:
            return cls.model_validate(yaml.safe_load(path.read_text(encoding="utf-8")))
        except (OSError, yaml.YAMLError, ValidationError) as error:
            raise ValueError(f"Could not load scene config {path}:\n{error}") from error


__all__ = [
    "LightingConfig",
    "OverheadCameraConfig",
    "RobotMountConfig",
    "RobotMountsConfig",
    "RoomConfig",
    "SceneConfig",
    "SceneRuntimeConfig",
    "SurfaceConfig",
    "TaskObjectPlacementArea",
]
