"""Physics, rendering, device, and timing configuration."""

from __future__ import annotations

from typing import Annotated, Literal
from pydantic import Field, StrictBool

from scale_bench.config.base import (
    FiniteFloat,
    FrozenModel,
    PositiveFloat,
    PositiveInt,
)


Device = Annotated[str, Field(pattern=r"^(cpu|cuda(?::[0-9]+)?)$")]


class RenderConfig(FrozenModel):
    rendering_mode: Literal["performance", "balanced", "quality"] | None = None
    antialiasing_mode: Literal["Off", "FXAA", "DLSS", "TAA", "DLAA"] | None = None
    enable_translucency: StrictBool = True
    enable_reflections: StrictBool = True
    enable_global_illumination: StrictBool = True


class PhysxConfig(FrozenModel):
    enable_external_forces_every_iteration: StrictBool = True


class SimulationConfig(FrozenModel):
    """Top-level simulator-independent simulation configuration."""

    device: Device = "cuda:0"
    physics_dt_s: PositiveFloat = 1.0 / 120.0
    gravity_m_s2: tuple[FiniteFloat, FiniteFloat, FiniteFloat] = (0.0, 0.0, -9.81)
    render_interval: PositiveInt = 4
    render: RenderConfig = RenderConfig()
    physx: PhysxConfig = PhysxConfig()
