"""Convert pure simulation configuration into a native Isaac Lab cfg."""

from __future__ import annotations

from isaaclab.sim import RenderCfg, SimulationCfg
from isaaclab_physx.physics import PhysxCfg

from scale_bench.config.models.simulation import SimulationConfig


def build_simulation_cfg(
    config: SimulationConfig,
    *,
    device: str | None = None,
) -> SimulationCfg:
    """Return a fresh simulation cfg from validated pure data."""

    resolved_device = config.device if device is None else device
    if not _is_valid_device(resolved_device):
        raise ValueError(
            "device must be 'cpu', 'cuda', or an indexed device such as 'cuda:0'"
        )

    return SimulationCfg(
        device=resolved_device,
        dt=config.physics_dt_s,
        gravity=config.gravity_m_s2,
        render_interval=config.render_interval,
        physics=PhysxCfg(**config.physx.model_dump()),
        render=RenderCfg(**config.render.model_dump(exclude_none=True)),
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


__all__ = ["build_simulation_cfg"]
