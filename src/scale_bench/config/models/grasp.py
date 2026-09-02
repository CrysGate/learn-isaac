"""Validated object-local grasp candidates for one robot TCP."""

from __future__ import annotations

import math
from typing import Self
from urllib.parse import urlsplit

from pydantic import Field, StrictBool, field_validator, model_validator

from scale_bench.config.base import (
    FrozenModel,
    Name,
    NonNegativeFloat,
    NonNegativeInt,
    Position3,
    PositiveFloat,
    PositiveInt,
    Quaternion,
    UnitIntervalFloat,
    require_unit_quaternion,
)


class AnyGraspConfig(FrozenModel):
    """Runtime AnyGrasp HTTP service and candidate-selection settings."""

    service_url: Name = "http://127.0.0.1:5001"
    request_timeout_s: PositiveFloat = 60.0
    capture_distance_m: PositiveFloat
    depth_trunc_m: PositiveFloat = 2.0
    top_k: PositiveInt = 100
    min_score: UnitIntervalFloat = 0.0
    collision_detection: StrictBool = True
    dense_grasp: StrictBool = False
    approach_distance_m: PositiveFloat = 0.10
    target_margin_m: NonNegativeFloat = 0.015
    minimum_target_points: PositiveInt = 128
    minimum_point_height_above_table_m: NonNegativeFloat = 0.002
    minimum_tcp_height_above_table_m: NonNegativeFloat = 0.015
    maximum_open_axis_vertical_dot: UnitIntervalFloat = 0.35

    @field_validator("service_url")
    @classmethod
    def _validate_service_url(cls, value: str) -> str:
        normalized = value.rstrip("/")
        parsed = urlsplit(normalized)
        if parsed.scheme not in {"http", "https"} or not parsed.netloc:
            raise ValueError("service_url must be an absolute HTTP(S) URL")
        if parsed.query or parsed.fragment:
            raise ValueError("service_url must not contain a query or fragment")
        return normalized

class GraspCandidateConfig(FrozenModel):
    """One physics-validated ``T_object_tcp`` candidate."""

    candidate_id: NonNegativeInt
    position_object_m: Position3
    orientation_object_xyzw: Quaternion
    approach_axis_tcp: Position3 = (1.0, 0.0, 0.0)
    score: UnitIntervalFloat

    @model_validator(mode="after")
    def _validate_pose(self) -> Self:
        require_unit_quaternion(
            self.orientation_object_xyzw,
            "orientation_object_xyzw",
        )
        axis_norm = math.sqrt(sum(value * value for value in self.approach_axis_tcp))
        if not math.isclose(axis_norm, 1.0, abs_tol=1.0e-6):
            raise ValueError("approach_axis_tcp must be a unit vector")
        return self


class GraspCatalogConfig(FrozenModel):
    """Compact generated grasp catalog consumed by manipulation skills."""

    robot_name: Name
    tcp_parent_frame: Name
    tcp_position_m: Position3
    tcp_orientation_xyzw: Quaternion
    approach_distance_m: PositiveFloat
    objects: dict[Name, tuple[GraspCandidateConfig, ...]] = Field(min_length=1)

    @model_validator(mode="after")
    def _validate_catalog(self) -> Self:
        require_unit_quaternion(
            self.tcp_orientation_xyzw,
            "tcp_orientation_xyzw",
        )
        for object_name, candidates in self.objects.items():
            if not candidates:
                raise ValueError(f"{object_name!r} has no grasp candidates")
            ids = tuple(candidate.candidate_id for candidate in candidates)
            if len(ids) != len(set(ids)):
                raise ValueError(
                    f"{object_name!r} contains duplicate grasp candidate IDs"
                )
        return self


__all__ = ["AnyGraspConfig", "GraspCandidateConfig", "GraspCatalogConfig"]
