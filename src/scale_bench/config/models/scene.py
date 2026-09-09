"""Room, surfaces, mounts, cameras, and lighting configuration."""

from __future__ import annotations

from typing import Self

from pydantic import field_validator, model_validator

from scale_bench.config.base import (
    AssetReference,
    CameraConvention,
    ConfigReference,
    FiniteFloat,
    FrozenModel,
    NonNegativeFloat,
    OptionalAssetReference,
    Position2,
    Position3,
    PositiveFloat,
    Quaternion,
    UnitIntervalFloat,
    require_unit_quaternion,
)
from scale_bench.config.models.grasp import AnyGraspConfig


class RoomConfig(FrozenModel):
    usd_path: AssetReference
    scale: PositiveFloat = 0.5


class SurfaceConfig(FrozenModel):
    position_m: Position3
    size_m: tuple[PositiveFloat, PositiveFloat, PositiveFloat]
    material_path: OptionalAssetReference
    uv_scale: tuple[PositiveFloat, PositiveFloat] = (1.0, 1.0)
    static_friction: NonNegativeFloat
    dynamic_friction: NonNegativeFloat
    restitution: UnitIntervalFloat


class RobotMountConfig(FrozenModel):
    position_xy_m: Position2
    orientation_xyzw: Quaternion

    @model_validator(mode="after")
    def _validate_orientation(self) -> Self:
        require_unit_quaternion(self.orientation_xyzw, "orientation_xyzw")
        return self


class RobotMountsConfig(FrozenModel):
    left: RobotMountConfig
    right: RobotMountConfig


class ManipulationConfig(FrozenModel):
    lift_height_m: PositiveFloat


class OverheadCameraConfig(FrozenModel):
    profile_path: ConfigReference
    stand_usd_path: AssetReference
    stand_position_xy_m: Position2
    stand_orientation_xyzw: Quaternion
    sensor_local_position_m: Position3
    sensor_local_orientation_xyzw: Quaternion
    convention: CameraConvention = "opengl"

    @model_validator(mode="after")
    def _validate_orientations(self) -> Self:
        require_unit_quaternion(
            self.stand_orientation_xyzw,
            "stand_orientation_xyzw",
        )
        require_unit_quaternion(
            self.sensor_local_orientation_xyzw,
            "sensor_local_orientation_xyzw",
        )
        return self


class LightingConfig(FrozenModel):
    texture_path: AssetReference
    intensity: NonNegativeFloat


class TaskObjectPlacementArea(FrozenModel):
    x_range_m: tuple[FiniteFloat, FiniteFloat]
    y_range_m: tuple[FiniteFloat, FiniteFloat]

    @field_validator("x_range_m", "y_range_m")
    @classmethod
    def _validate_range(cls, value: tuple[float, float]) -> tuple[float, float]:
        if value[0] >= value[1]:
            raise ValueError("lower bound must be less than upper bound")
        return value


class OcclusionConfig(FrozenModel):
    """Visual occluder used for AnyGrasp viewpoint experiments."""

    enabled: bool = False
    size_m: tuple[PositiveFloat, PositiveFloat, PositiveFloat] = (
        0.10,
        0.015,
        0.14,
    )
    line_fraction_min: UnitIntervalFloat = 0.55
    line_fraction_max: UnitIntervalFloat = 0.80
    lateral_offset_m: NonNegativeFloat = 0.05
    vertical_offset_m: NonNegativeFloat = 0.05

    @model_validator(mode="after")
    def _validate_line_fraction_range(self) -> Self:
        if self.line_fraction_min >= self.line_fraction_max:
            raise ValueError(
                "line_fraction_min must be less than line_fraction_max"
            )
        return self


class SceneConfig(FrozenModel):
    """Static scene description, excluding environment lifecycle settings."""

    room: RoomConfig
    ground: SurfaceConfig
    table: SurfaceConfig
    task_object_placement_area: TaskObjectPlacementArea
    robot_mounts: RobotMountsConfig
    manipulation: ManipulationConfig
    camera: OverheadCameraConfig
    anygrasp: AnyGraspConfig | None = None
    lighting: LightingConfig
    occlusion: OcclusionConfig = OcclusionConfig()

    @property
    def table_top_z_m(self) -> float:
        return self.table.position_m[2] + self.table.size_m[2] / 2.0