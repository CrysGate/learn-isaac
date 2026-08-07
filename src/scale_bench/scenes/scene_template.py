"""Build the dual-arm tabletop scene from a YAML configuration."""

from __future__ import annotations

from dataclasses import MISSING
from pathlib import Path
from typing import TYPE_CHECKING, overload

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialBaseCfg
from isaaclab.utils.configclass import configclass

from scale_bench import REPOSITORY_ROOT
from scale_bench.sensors import CameraProfile

from .scene_config import (
    DEFAULT_SCENE_CONFIG_PATH,
    LightingConfig,
    OverheadCameraConfig,
    RobotMountConfig,
    RoomConfig,
    SceneConfig,
    SurfaceConfig,
)
from .uv_cuboid import UvCuboidCfg

if TYPE_CHECKING:
    from scale_bench.robots import RobotProfile


@overload
def _asset_path(value: str) -> str: ...


@overload
def _asset_path(value: None) -> None: ...


def _asset_path(value: str | None) -> str | None:
    if value is None or "://" in value:
        return value
    path = Path(value)
    return str(path if path.is_absolute() else REPOSITORY_ROOT / path)


def _room_cfg(spec: RoomConfig) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Room",
        spawn=sim_utils.UsdFileCfg(
            usd_path=_asset_path(spec.usd_path),
            scale=(spec.scale, spec.scale, spec.scale),
        ),
    )


def _surface_cfg(prim_path: str, spec: SurfaceConfig) -> AssetBaseCfg:
    material_path = _asset_path(spec.material_path)
    return AssetBaseCfg(
        prim_path=prim_path,
        init_state=AssetBaseCfg.InitialStateCfg(pos=spec.position_m),
        spawn=UvCuboidCfg(
            size=spec.size_m,
            uv_scale=spec.uv_scale,
            collision_props=sim_utils.PhysxCollisionPropertiesCfg(
                collision_enabled=True
            ),
            physics_material=RigidBodyMaterialBaseCfg(
                static_friction=spec.static_friction,
                dynamic_friction=spec.dynamic_friction,
                restitution=spec.restitution,
            ),
            visual_material=(
                sim_utils.MdlFileCfg(mdl_path=material_path)
                if material_path is not None
                else None
            ),
        ),
    )


def _camera_stand_cfg(
    table_top_z_m: float,
    spec: OverheadCameraConfig,
) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/CameraStand",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(*spec.stand_position_xy_m, table_top_z_m),
            rot=spec.stand_orientation_xyzw,
        ),
        spawn=sim_utils.UsdFileCfg(usd_path=_asset_path(spec.stand_usd_path)),
    )


def _camera_cfg(spec: OverheadCameraConfig) -> CameraCfg:
    profile = CameraProfile.load(spec.profile_path)
    return profile.build_camera_cfg(
        prim_path="{ENV_REGEX_NS}/CameraStand/OverheadCamera",
        position_m=spec.sensor_local_position_m,
        orientation_xyzw=spec.sensor_local_orientation_xyzw,
        convention=spec.convention,
    )


def _light_cfg(spec: LightingConfig) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="/World/EnvironmentLight",
        spawn=sim_utils.DomeLightCfg(
            texture_file=_asset_path(spec.texture_path),
            intensity=spec.intensity,
        ),
    )


@configclass
class DualArmTabletopSceneCfg(InteractiveSceneCfg):
    """Room, table, two camera-equipped robots, and one overhead camera."""

    room: AssetBaseCfg = MISSING
    ground: AssetBaseCfg = MISSING
    table: AssetBaseCfg = MISSING
    camera_stand: AssetBaseCfg = MISSING
    left_robot: ArticulationCfg = MISSING
    right_robot: ArticulationCfg = MISSING
    left_robot_camera: CameraCfg = MISSING
    right_robot_camera: CameraCfg = MISSING
    overhead_camera: CameraCfg = MISSING
    environment_light: AssetBaseCfg = MISSING


def _mounted_robot_cfg(
    robot_cfg: ArticulationCfg,
    prim_path: str,
    mount: RobotMountConfig,
    table_top_z_m: float,
) -> ArticulationCfg:
    mounted_cfg = robot_cfg.copy()
    mounted_cfg.prim_path = prim_path
    mounted_cfg.init_state.pos = (*mount.position_xy_m, table_top_z_m)
    mounted_cfg.init_state.rot = mount.orientation_xyzw
    return mounted_cfg


def create_dual_arm_tabletop_scene_cfg(
    *,
    left_robot_profile: RobotProfile,
    right_robot_profile: RobotProfile,
    config_path: str | Path = DEFAULT_SCENE_CONFIG_PATH,
    num_envs: int | None = None,
    env_spacing_m: float | None = None,
) -> DualArmTabletopSceneCfg:
    """Create the scene from robot profiles/configs and a scene preset."""

    if left_robot_profile is None:
        raise ValueError("left_robot_profile is required")
    if right_robot_profile is None:
        raise ValueError("right_robot_profile is required")

    left_robot_cfg = left_robot_profile.build_articulation_cfg()
    right_robot_cfg = right_robot_profile.build_articulation_cfg()

    config = SceneConfig.load(config_path)
    table = config.table
    runtime = config.runtime
    camera = config.camera
    table_top_z_m = config.table_top_z_m

    scene_cfg = DualArmTabletopSceneCfg(
        num_envs=runtime.num_envs if num_envs is None else num_envs,
        env_spacing=runtime.env_spacing_m if env_spacing_m is None else env_spacing_m,
        replicate_physics=runtime.replicate_physics,
        clone_in_fabric=runtime.clone_in_fabric,
        room=_room_cfg(config.room),
        ground=_surface_cfg("{ENV_REGEX_NS}/Ground", config.ground),
        table=_surface_cfg("{ENV_REGEX_NS}/Table", table),
        camera_stand=_camera_stand_cfg(table_top_z_m, camera),
        left_robot_camera=(
            left_robot_profile.build_camera_cfg(
                robot_prim_path="{ENV_REGEX_NS}/LeftRobot"
            )
        ),
        right_robot_camera=(
            right_robot_profile.build_camera_cfg(
                robot_prim_path="{ENV_REGEX_NS}/RightRobot"
            )
        ),
        overhead_camera=_camera_cfg(camera),
        environment_light=_light_cfg(config.lighting),
    )
    scene_cfg.left_robot = _mounted_robot_cfg(
        left_robot_cfg,
        "{ENV_REGEX_NS}/LeftRobot",
        config.robot_mounts.left,
        table_top_z_m,
    )
    scene_cfg.right_robot = _mounted_robot_cfg(
        right_robot_cfg,
        "{ENV_REGEX_NS}/RightRobot",
        config.robot_mounts.right,
        table_top_z_m,
    )
    return scene_cfg


__all__ = [
    "DualArmTabletopSceneCfg",
    "create_dual_arm_tabletop_scene_cfg",
]
