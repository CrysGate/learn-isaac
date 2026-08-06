"""Typed scene-level configuration loaded from YAML."""

from __future__ import annotations

from functools import cache
from pathlib import Path
from typing import Annotated, Any, Self

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SCENE_CONFIG_PATH = REPOSITORY_ROOT / "configs/scene/default.yml"
FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]


class TaskObjectPlacementArea(BaseModel):
    """Axis-aligned XY bounds in the environment-local frame."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    x_range_m: tuple[FiniteFloat, FiniteFloat]
    y_range_m: tuple[FiniteFloat, FiniteFloat]

    @field_validator("x_range_m", "y_range_m")
    @classmethod
    def _validate_range(cls, value: tuple[float, float]) -> tuple[float, float]:
        if value[0] >= value[1]:
            raise ValueError("lower bound must be less than upper bound")
        return value


class SceneConfig(BaseModel):
    """Top-level scene preset with typed scene-wide metadata."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    room: dict[str, Any]
    ground: dict[str, Any]
    table: dict[str, Any]
    task_object_placement_area: TaskObjectPlacementArea
    robot_mounts: dict[str, dict[str, Any]]
    camera: dict[str, Any]
    lighting: dict[str, Any]
    runtime: dict[str, Any]

    @property
    def table_top_z_m(self) -> float:
        return self.table["position_m"][2] + self.table["size_m"][2] / 2.0

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


__all__ = ["SceneConfig"]
