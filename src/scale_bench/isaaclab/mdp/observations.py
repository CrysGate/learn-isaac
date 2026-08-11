"""Observation terms used by the ScaleBench Observation Manager."""

from __future__ import annotations

import torch
from isaaclab.assets import Articulation
from isaaclab.envs import ManagerBasedEnv
from isaaclab.envs.utils.io_descriptors import (
    GenericObservationIODescriptor,
    generic_io_descriptor,
    record_dtype,
    record_joint_names,
    record_shape,
)
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import Camera


def _record_camera_metadata(
    output: torch.Tensor,
    descriptor: GenericObservationIODescriptor,
    *,
    env: ManagerBasedEnv,
    sensor_cfg: SceneEntityCfg,
    data_type: str,
) -> None:
    sensor: Camera = env.scene.sensors[sensor_cfg.name]
    descriptor.shape = tuple(output.shape[1:])
    descriptor.dtype = str(output.dtype)
    descriptor.extras = {
        "camera_name": sensor_cfg.name,
        "data_type": data_type,
        "layout": "HWC",
        "width": sensor.cfg.width,
        "height": sensor.cfg.height,
        "intrinsic_matrix_px": sensor.data.intrinsic_matrices.torch[0]
        .detach()
        .cpu()
        .reshape(-1)
        .tolist(),
        "clipping_range_m": list(sensor.cfg.spawn.clipping_range),
    }


@generic_io_descriptor(
    observation_type="CameraImage",
    on_inspect=_record_camera_metadata,
)
def camera_image(
    env: ManagerBasedEnv,
    sensor_cfg: SceneEntityCfg,
    data_type: str,
) -> torch.Tensor:
    """Return an unmodified camera output tensor in HWC layout."""

    sensor: Camera = env.scene.sensors[sensor_cfg.name]
    return sensor.data.output[data_type].torch.clone()


@generic_io_descriptor(
    observation_type="GripperJointState",
    on_inspect=[record_joint_names, record_dtype, record_shape],
    units="rad or m, joint-dependent",
)
def gripper_joint_pos(
    env: ManagerBasedEnv,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Return actual parallel-gripper joint positions."""

    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.joint_pos.torch[:, asset_cfg.joint_ids]


__all__ = ["camera_image", "gripper_joint_pos"]
