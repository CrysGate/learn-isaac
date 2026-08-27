"""Camera optics, output, and coordinate configuration."""

from __future__ import annotations

from typing import Annotated, Self
from pydantic import Field, model_validator

from scale_bench.config.base import (
    FiniteFloat,
    FrozenModel,
    Name,
    PositiveFloat,
    PositiveInt,
)


class CameraConfig(FrozenModel):
    """Reusable optical and output parameters for one camera model."""

    model: Name
    width: PositiveInt
    height: PositiveInt
    update_period_s: PositiveFloat
    data_types: Annotated[tuple[Name, ...], Field(min_length=1)]
    focal_length_mm: PositiveFloat
    intrinsic_source: Name
    intrinsic_matrix_px: tuple[
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
        FiniteFloat,
    ]
    distortion_model: Name
    distortion_coefficients: tuple[FiniteFloat, ...]
    clipping_range_m: tuple[PositiveFloat, PositiveFloat]

    @model_validator(mode="after")
    def _validate_camera_parameters(self) -> Self:
        if len(self.data_types) != len(set(self.data_types)):
            raise ValueError("data_types contains duplicate names")

        fx, _, _, second_row_x, fy, _, third_row_x, third_row_y, scale = (
            self.intrinsic_matrix_px
        )
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError("intrinsic focal lengths must be positive")
        if (second_row_x, third_row_x, third_row_y, scale) != (
            0.0,
            0.0,
            0.0,
            1.0,
        ):
            raise ValueError(
                "intrinsic_matrix_px must be a pinhole calibration matrix"
            )

        near, far = self.clipping_range_m
        if near >= far:
            raise ValueError(
                "clipping_range_m far plane must be greater than near plane"
            )
        return self
