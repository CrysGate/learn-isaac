"""Environment lifecycle, cloning, control, and reset configuration."""

from __future__ import annotations

from typing import Literal
from pydantic import StrictBool

from scale_bench.config.base import (
    FrozenModel,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
)


class EnvironmentConfig(FrozenModel):
    """Settings owned by environment creation and reset lifecycle."""

    num_envs: PositiveInt = 1
    env_spacing_m: PositiveFloat = 5.0
    control_decimation: PositiveInt = 4
    replicate_physics: StrictBool = True
    clone_in_fabric: StrictBool = False
    arm_action_mode: Literal["joint_position"] = "joint_position"
    num_rerenders_on_reset: NonNegativeInt = 1
    wait_for_textures: StrictBool = True
    seed: NonNegativeInt | None = 0
