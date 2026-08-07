"""Load simulation runtime parameters and build Isaac Lab configs."""

from __future__ import annotations

from functools import cache
from pathlib import Path
from typing import TYPE_CHECKING, Annotated, Literal, Self

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

if TYPE_CHECKING:
    from isaaclab.sim import SimulationCfg


DEFAULT_SIM_CONFIG_PATH = REPOSITORY_ROOT / "configs/sim/default.yml"

FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
PositiveFloat = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
PositiveInt = Annotated[StrictInt, Field(gt=0)]
Device = Annotated[str, Field(pattern=r"^(cpu|cuda(?::[0-9]+)?)$")]


class _SimModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class RenderConfig(_SimModel):
    """Small set of rendering choices that define observation quality."""

    rendering_mode: Literal["performance", "balanced", "quality"] | None = None
    antialiasing_mode: Literal["Off", "FXAA", "DLSS", "TAA", "DLAA"] | None = None
    enable_translucency: StrictBool = True
    enable_reflections: StrictBool = True
    enable_global_illumination: StrictBool = True


class PhysxConfig(_SimModel):
    """Manipulation-specific override to the native PhysX defaults."""

    enable_external_forces_every_iteration: StrictBool = True


class SimConfig(_SimModel):
    """Top-level, configuration-file representation of ``SimulationCfg``."""

    device: Device = "cuda:0"
    physics_dt_s: PositiveFloat = 1.0 / 120.0
    gravity_m_s2: tuple[FiniteFloat, FiniteFloat, FiniteFloat] = (0.0, 0.0, -9.81)
    render_interval: PositiveInt = 4
    render: RenderConfig = RenderConfig()
    physx: PhysxConfig = PhysxConfig()

    @property
    def physics_frequency_hz(self) -> float:
        return 1.0 / self.physics_dt_s

    @property
    def render_dt_s(self) -> float:
        return self.physics_dt_s * self.render_interval

    @property
    def render_frequency_hz(self) -> float:
        return 1.0 / self.render_dt_s

    @classmethod
    @cache
    def load(cls, config_path: str | Path = DEFAULT_SIM_CONFIG_PATH) -> Self:
        """Load and validate a simulation preset relative to the repository root."""

        path = Path(config_path)
        if not path.is_absolute():
            path = REPOSITORY_ROOT / path
        try:
            return cls.model_validate(yaml.safe_load(path.read_text(encoding="utf-8")))
        except (OSError, yaml.YAMLError, ValidationError) as error:
            raise ValueError(f"Could not load simulation config {path}:\n{error}") from error

    def build_simulation_cfg(self, *, device: str | None = None) -> SimulationCfg:
        """Build a fresh native Isaac Lab config, optionally overriding the device."""

        from isaaclab.sim import RenderCfg, SimulationCfg
        from isaaclab_physx.physics import PhysxCfg

        resolved_device = self.device if device is None else device
        if not _is_valid_device(resolved_device):
            raise ValueError(
                "device must be 'cpu', 'cuda', or an indexed device such as 'cuda:0'"
            )

        physics_cfg = PhysxCfg(**self.physx.model_dump())
        render_cfg = RenderCfg(**self.render.model_dump(exclude_none=True))

        return SimulationCfg(
            device=resolved_device,
            dt=self.physics_dt_s,
            gravity=self.gravity_m_s2,
            render_interval=self.render_interval,
            physics=physics_cfg,
            render=render_cfg,
        )


def _is_valid_device(device: str) -> bool:
    if device in {"cpu", "cuda"}:
        return True
    prefix, separator, index = device.partition(":")
    return (
        prefix == "cuda"
        and separator == ":"
        and index.isascii()
        and index.isdigit()
    )


__all__ = [
    "PhysxConfig",
    "RenderConfig",
    "SimConfig",
]
