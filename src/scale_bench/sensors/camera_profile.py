"""Load reusable camera parameters and build Isaac Lab camera configs."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Annotated, Literal, Self, TypeAlias

import yaml
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationError,
    model_validator,
)

from scale_bench import REPOSITORY_ROOT

if TYPE_CHECKING:
    from isaaclab.sensors import CameraCfg


FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
PositiveFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
PositiveInt = Annotated[int, Field(gt=0)]
Name = Annotated[str, Field(min_length=1)]
CameraConvention: TypeAlias = Literal["opengl", "ros", "world"]


class CameraProfile(BaseModel):
    """Reusable optical and output parameters for one camera model."""

    model_config = ConfigDict(extra="forbid", frozen=True)

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

    @classmethod
    def load(cls, config_path: str | Path) -> Self:
        """Load and validate a YAML profile relative to the repository root."""

        path = Path(config_path)
        if not path.is_absolute():
            path = REPOSITORY_ROOT / path
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
            return cls.model_validate(document)
        except (OSError, yaml.YAMLError, ValidationError) as error:
            raise ValueError(
                f"Could not load camera profile {path}:\n{error}"
            ) from error

    def build_camera_cfg(
        self,
        *,
        prim_path: str,
        position_m: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float],
        convention: CameraConvention,
    ) -> CameraCfg:
        """Build a fresh ``CameraCfg`` at the supplied scene-local pose."""

        import isaaclab.sim as sim_utils
        from isaaclab.sensors import CameraCfg

        return CameraCfg(
            prim_path=prim_path,
            update_period=self.update_period_s,
            width=self.width,
            height=self.height,
            data_types=list(self.data_types),
            spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
                intrinsic_matrix=list(self.intrinsic_matrix_px),
                width=self.width,
                height=self.height,
                clipping_range=self.clipping_range_m,
                focal_length=self.focal_length_mm,
            ),
            offset=CameraCfg.OffsetCfg(
                pos=position_m,
                rot=orientation_xyzw,
                convention=convention,
            ),
        )


__all__ = ["CameraConvention", "CameraProfile"]
