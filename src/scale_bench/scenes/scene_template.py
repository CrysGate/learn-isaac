"""Build the dual-arm tabletop scene from a YAML configuration."""

from __future__ import annotations

from dataclasses import MISSING
from functools import cache
from pathlib import Path
from typing import Any

import isaaclab.sim as sim_utils
import yaml
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils.configclass import configclass

from .uv_cuboid import UvCuboidCfg


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SCENE_CONFIG_PATH = REPOSITORY_ROOT / "configs/scene/default.yml"


@cache
def _load_scene_config(config_path: str | Path) -> dict[str, Any]:
    path = Path(config_path)
    if not path.is_absolute():
        path = REPOSITORY_ROOT / path

    with path.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError(f"Scene config must be a mapping: {path}")
    return config


def _asset_path(value: str | None) -> str | None:
    if value is None or "://" in value:
        return value
    path = Path(value)
    return str(path if path.is_absolute() else REPOSITORY_ROOT / path)


def _room_cfg(spec: dict[str, Any]) -> AssetBaseCfg:
    scale = float(spec.get("scale", 0.5))
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Room",
        spawn=sim_utils.UsdFileCfg(
            usd_path=_asset_path(spec["usd_path"]),
            scale=(scale, scale, scale),
        ),
    )


def _surface_cfg(prim_path: str, spec: dict[str, Any]) -> AssetBaseCfg:
    material_path = _asset_path(spec["material_path"])
    return AssetBaseCfg(
        prim_path=prim_path,
        init_state=AssetBaseCfg.InitialStateCfg(pos=tuple(spec["position_m"])),
        spawn=UvCuboidCfg(
            size=tuple(spec["size_m"]),
            uv_scale=tuple(spec.get("uv_scale", (1.0, 1.0))),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=spec["static_friction"],
                dynamic_friction=spec["dynamic_friction"],
                restitution=spec["restitution"],
            ),
            visual_material=(
                sim_utils.MdlFileCfg(mdl_path=material_path)
                if material_path is not None
                else None
            ),
        ),
    )


def _camera_stand_cfg(table_top_z_m: float, spec: dict[str, Any]) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/CameraStand",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(*spec["stand_position_xy_m"], table_top_z_m),
            rot=tuple(spec["stand_orientation_xyzw"]),
        ),
        spawn=sim_utils.UsdFileCfg(usd_path=_asset_path(spec["stand_usd_path"])),
    )


def _camera_cfg(spec: dict[str, Any]) -> CameraCfg:
    width = int(spec["width"])
    height = int(spec["height"])
    return CameraCfg(
        prim_path="{ENV_REGEX_NS}/CameraStand/D435Sensor",
        update_period=spec["update_period_s"],
        width=width,
        height=height,
        data_types=list(spec["data_types"]),
        spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
            intrinsic_matrix=list(spec["intrinsic_matrix_px"]),
            width=width,
            height=height,
            clipping_range=tuple(spec["clipping_range_m"]),
            focal_length=spec["focal_length_mm"],
        ),
        offset=CameraCfg.OffsetCfg(
            pos=tuple(spec["sensor_local_position_m"]),
            rot=tuple(spec["sensor_local_orientation_xyzw"]),
            convention=spec["convention"],
        ),
    )


def _light_cfg(spec: dict[str, Any]) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="/World/EnvironmentLight",
        spawn=sim_utils.DomeLightCfg(
            texture_file=_asset_path(spec["texture_path"]),
            intensity=spec["intensity"],
        ),
    )


@configclass
class DualArmTabletopSceneCfg(InteractiveSceneCfg):
    """Room, table, two robot mounts, and one overhead RGB-D camera."""

    room: AssetBaseCfg = MISSING
    ground: AssetBaseCfg = MISSING
    table: AssetBaseCfg = MISSING
    camera_stand: AssetBaseCfg = MISSING
    left_robot: ArticulationCfg = MISSING
    right_robot: ArticulationCfg = MISSING
    overhead_camera: CameraCfg = MISSING
    environment_light: AssetBaseCfg = MISSING


def _mounted_robot_cfg(
    robot_cfg: ArticulationCfg,
    prim_path: str,
    mount: dict[str, Any],
    table_top_z_m: float,
) -> ArticulationCfg:
    mounted_cfg = robot_cfg.copy()
    mounted_cfg.prim_path = prim_path
    mounted_cfg.init_state.pos = (*mount["position_xy_m"], table_top_z_m)
    mounted_cfg.init_state.rot = tuple(mount["orientation_xyzw"])
    return mounted_cfg


def create_dual_arm_tabletop_scene_cfg(
    *,
    left_robot_cfg: ArticulationCfg,
    right_robot_cfg: ArticulationCfg,
    config_path: str | Path = DEFAULT_SCENE_CONFIG_PATH,
    num_envs: int | None = None,
    env_spacing_m: float | None = None,
) -> DualArmTabletopSceneCfg:
    """Create the scene from one YAML file."""

    config = _load_scene_config(config_path)
    table = config["table"]
    runtime = config["runtime"]
    camera = config["camera"]
    table_top_z_m = table["position_m"][2] + table["size_m"][2] / 2.0

    scene_cfg = DualArmTabletopSceneCfg(
        num_envs=runtime["num_envs"] if num_envs is None else num_envs,
        env_spacing=runtime["env_spacing_m"] if env_spacing_m is None else env_spacing_m,
        replicate_physics=runtime["replicate_physics"],
        clone_in_fabric=runtime["clone_in_fabric"],
        room=_room_cfg(config["room"]),
        ground=_surface_cfg("{ENV_REGEX_NS}/Ground", config["ground"]),
        table=_surface_cfg("{ENV_REGEX_NS}/Table", table),
        camera_stand=_camera_stand_cfg(table_top_z_m, camera),
        overhead_camera=_camera_cfg(camera),
        environment_light=_light_cfg(config["lighting"]),
    )
    scene_cfg.left_robot = _mounted_robot_cfg(
        left_robot_cfg,
        "{ENV_REGEX_NS}/LeftRobot",
        config["robot_mounts"]["left"],
        table_top_z_m,
    )
    scene_cfg.right_robot = _mounted_robot_cfg(
        right_robot_cfg,
        "{ENV_REGEX_NS}/RightRobot",
        config["robot_mounts"]["right"],
        table_top_z_m,
    )
    return scene_cfg


__all__ = [
    "DualArmTabletopSceneCfg",
    "create_dual_arm_tabletop_scene_cfg",
]
