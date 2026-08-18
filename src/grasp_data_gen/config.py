"""Grasp configuration built on ScaleBench's public config API."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Annotated, Self
from pydantic import Field, StrictBool, StrictInt, model_validator

from scale_bench.config.base import (
    ConfigReference,
    FrozenModel,
    Name,
    NonNegativeFloat,
    PositiveFloat,
    PositiveInt,
    Position3,
    require_unique,
)
from scale_bench.config.loader import load_config
from scale_bench.config.models.robot import RobotConfig


AbsolutePrimPath = Annotated[str, Field(pattern=r"^/[A-Za-z_][A-Za-z0-9_/]*$")]
RelativePrimPath = Annotated[str, Field(pattern=r"^[A-Za-z_][A-Za-z0-9_/]*$")]
UnitDot = Annotated[float, Field(ge=-1.0, le=1.0, allow_inf_nan=False)]
UnitInterval = Annotated[float, Field(gt=0.0, le=1.0, allow_inf_nan=False)]


def _require_unit_vector(value: Position3, field_name: str) -> None:
    norm = math.sqrt(sum(component * component for component in value))
    if not math.isclose(norm, 1.0, abs_tol=1.0e-6):
        raise ValueError(f"{field_name} must be a unit vector")


class GraspSamplerConfig(FrozenModel):
    sampler_type: str = "antipodal"
    num_candidates: PositiveInt = 256
    num_orientations: PositiveInt = 8
    standoff_axis_base: Position3 = (0.0, 0.0, 1.0)
    gripper_approach_direction: Position3 = (0.0, 0.0, 1.0)
    grasp_align_axis: Position3 = (0.0, 1.0, 0.0)
    orientation_sample_axis: Position3 = (0.0, 1.0, 0.0)
    approach_axis_tcp: Position3 = (1.0, 0.0, 0.0)
    lateral_sigma: NonNegativeFloat = 0.0
    random_seed: StrictInt = 0

    @model_validator(mode="after")
    def _validate_axes(self) -> Self:
        for field_name in (
            "standoff_axis_base",
            "gripper_approach_direction",
            "grasp_align_axis",
            "orientation_sample_axis",
            "approach_axis_tcp",
        ):
            _require_unit_vector(getattr(self, field_name), field_name)
        return self


class TabletopGraspFilterConfig(FrozenModel):
    enabled: StrictBool = True
    up_axis_object: Position3 = (0.0, 0.0, 1.0)
    maximum_approach_up_dot: UnitDot = 0.0
    gripper_clearance_m: NonNegativeFloat = 0.003
    approach_distance_m: PositiveFloat = 0.10
    require_camera_above_tcp: StrictBool = False
    minimum_camera_height_above_tcp_m: NonNegativeFloat = 0.0
    minimum_camera_side_up_dot: UnitDot = 0.0

    @model_validator(mode="after")
    def _validate_up_axis(self) -> Self:
        _require_unit_vector(self.up_axis_object, "up_axis_object")
        return self


class GraspPhysicsValidationConfig(FrozenModel):
    close_steps: PositiveInt = 48
    hold_steps: PositiveInt = 30
    physics_dt: PositiveFloat = 1.0 / 60.0
    minimum_residual_aperture_m: PositiveFloat = 0.002
    minimum_closure_travel_m: PositiveFloat = 0.002
    maximum_command_openness_spread: UnitInterval = 0.15
    hold_position_tolerance_m: PositiveFloat = 0.02
    hold_orientation_tolerance_rad: PositiveFloat = 0.35


class GraspGenerationConfig(FrozenModel):
    """Recipe for assembling, sampling, and testing one gripper."""

    name: Name
    robot_profile: ConfigReference
    source_root_prim: AbsolutePrimPath
    base_link: Name
    link_names: Annotated[tuple[Name, ...], Field(min_length=2)]
    joint_scope: RelativePrimPath = "joints"
    tcp_prim_path: RelativePrimPath
    material_prim_paths: tuple[RelativePrimPath, ...] = ()
    sampler: GraspSamplerConfig = GraspSamplerConfig()
    tabletop_filter: TabletopGraspFilterConfig = TabletopGraspFilterConfig()
    validation: GraspPhysicsValidationConfig = GraspPhysicsValidationConfig()

    @model_validator(mode="after")
    def _validate_assembly(self) -> Self:
        require_unique(self.link_names, "link_names")
        require_unique(self.material_prim_paths, "material_prim_paths")
        if self.base_link not in self.link_names:
            raise ValueError("base_link must be included in link_names")
        return self


def load_grasp_config(
    config_path: str | Path,
    *,
    asset_root: str | Path,
) -> tuple[GraspGenerationConfig, RobotConfig]:
    """Load a grasp recipe and its shared ScaleBench robot profile."""

    path = Path(config_path).expanduser().resolve()
    generation = load_config(path, GraspGenerationConfig)
    robot = load_config(generation.robot_profile, RobotConfig, asset_root=asset_root)
    return generation, robot
