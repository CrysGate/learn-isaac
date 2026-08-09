"""Typed runtime settings owned by the manager-based environment."""

from __future__ import annotations

from functools import cache
from pathlib import Path
from typing import Annotated, Literal, Self

import yaml
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    StrictBool,
    StrictInt,
    ValidationError,
)

from scale_bench import REPOSITORY_ROOT


DEFAULT_ENV_CONFIG_PATH = REPOSITORY_ROOT / "configs/envs/default.yml"
PositiveInt = Annotated[StrictInt, Field(gt=0)]
NonNegativeInt = Annotated[StrictInt, Field(ge=0)]


class EnvRuntimeConfig(BaseModel):
    """Small set of environment-lifecycle settings outside scene and simulation presets."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    control_decimation: PositiveInt = 4
    arm_action_mode: Literal["joint_position"] = "joint_position"
    num_rerenders_on_reset: NonNegativeInt = 1
    wait_for_textures: StrictBool = True
    seed: NonNegativeInt | None = 0

    @classmethod
    @cache
    def load(cls, config_path: str | Path = DEFAULT_ENV_CONFIG_PATH) -> Self:
        """Load an environment runtime preset relative to the repository root."""

        path = Path(config_path)
        if not path.is_absolute():
            path = REPOSITORY_ROOT / path
        try:
            return cls.model_validate(yaml.safe_load(path.read_text(encoding="utf-8")))
        except (OSError, yaml.YAMLError, ValidationError) as error:
            raise ValueError(f"Could not load environment config {path}:\n{error}") from error


__all__ = ["EnvRuntimeConfig"]
