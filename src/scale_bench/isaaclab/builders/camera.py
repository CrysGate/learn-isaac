"""Convert pure camera configuration into a native Isaac Lab camera cfg."""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.sensors import CameraCfg

from scale_bench.config.base import CameraConvention
from scale_bench.config.models.camera import CameraConfig


def build_camera_cfg(
    config: CameraConfig,
    *,
    prim_path: str,
    position_m: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
    convention: CameraConvention,
) -> CameraCfg:
    """Return a fresh native camera cfg from validated pure data."""

    return CameraCfg(
        prim_path=prim_path,
        update_period=config.update_period_s,
        width=config.width,
        height=config.height,
        data_types=list(config.data_types),
        spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
            intrinsic_matrix=list(config.intrinsic_matrix_px),
            width=config.width,
            height=config.height,
            clipping_range=config.clipping_range_m,
            focal_length=config.focal_length_mm,
        ),
        offset=CameraCfg.OffsetCfg(
            pos=position_m,
            rot=orientation_xyzw,
            convention=convention,
        ),
    )


__all__ = ["build_camera_cfg"]
