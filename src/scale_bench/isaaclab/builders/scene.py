"""Build the native dual-arm scene from resolved pure configuration."""

from __future__ import annotations

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialBaseCfg
from isaaclab.utils.configclass import configclass

from scale_bench.config.loader import load_config
from scale_bench.config.models.camera import CameraConfig
from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import (
    LightingConfig,
    OverheadCameraConfig,
    RobotMountConfig,
    RoomConfig,
    SceneConfig,
    SurfaceConfig,
)
from scale_bench.isaaclab.builders.camera import build_camera_cfg
from scale_bench.isaaclab.builders.robot import (
    build_mounted_camera_cfg,
    build_robot_cfg,
)
from scale_bench.isaaclab.spawners.uv_cuboid import UvCuboidCfg


@configclass
class DualArmTabletopSceneCfg(InteractiveSceneCfg):
    """Room, table, two camera-equipped robots, and one overhead camera."""

    room: AssetBaseCfg = MISSING
    ground: AssetBaseCfg = MISSING
    table: AssetBaseCfg = MISSING
    camera_stand: AssetBaseCfg = MISSING
    left_robot: ArticulationCfg = MISSING
    right_robot: ArticulationCfg = MISSING
    left_robot_camera: CameraCfg | None = MISSING
    right_robot_camera: CameraCfg | None = MISSING
    overhead_camera: CameraCfg = MISSING
    environment_light: AssetBaseCfg = MISSING


def build_scene_cfg(
    *,
    left_robot_config: RobotConfig,
    right_robot_config: RobotConfig,
    scene_config: SceneConfig,
    environment_config: EnvironmentConfig,
    num_envs: int | None = None,
    env_spacing_m: float | None = None,
) -> DualArmTabletopSceneCfg:
    """Return a fresh native scene cfg from already resolved pure configs."""

    environment_config = _apply_environment_overrides(
        environment_config,
        num_envs=num_envs,
        env_spacing_m=env_spacing_m,
    )
    table_top_z_m = scene_config.table_top_z_m
    scene_cfg = DualArmTabletopSceneCfg(
        num_envs=environment_config.num_envs,
        env_spacing=environment_config.env_spacing_m,
        replicate_physics=environment_config.replicate_physics,
        clone_in_fabric=environment_config.clone_in_fabric,
        room=_room_cfg(scene_config.room),
        ground=_surface_cfg("{ENV_REGEX_NS}/Ground", scene_config.ground),
        table=_surface_cfg("{ENV_REGEX_NS}/Table", scene_config.table),
        camera_stand=_camera_stand_cfg(table_top_z_m, scene_config.camera),
        left_robot=_mounted_robot_cfg(
            build_robot_cfg(left_robot_config),
            "{ENV_REGEX_NS}/LeftRobot",
            scene_config.robot_mounts.left,
            table_top_z_m,
        ),
        right_robot=_mounted_robot_cfg(
            build_robot_cfg(right_robot_config),
            "{ENV_REGEX_NS}/RightRobot",
            scene_config.robot_mounts.right,
            table_top_z_m,
        ),
        left_robot_camera=build_mounted_camera_cfg(
            left_robot_config,
            robot_prim_path="{ENV_REGEX_NS}/LeftRobot",
        ),
        right_robot_camera=build_mounted_camera_cfg(
            right_robot_config,
            robot_prim_path="{ENV_REGEX_NS}/RightRobot",
        ),
        overhead_camera=_overhead_camera_cfg(scene_config.camera),
        environment_light=_light_cfg(scene_config.lighting),
    )
    return scene_cfg


def _apply_environment_overrides(
    config: EnvironmentConfig,
    *,
    num_envs: int | None,
    env_spacing_m: float | None,
) -> EnvironmentConfig:
    updates = {}
    if num_envs is not None:
        updates["num_envs"] = num_envs
    if env_spacing_m is not None:
        updates["env_spacing_m"] = env_spacing_m
    if not updates:
        return config
    return EnvironmentConfig.model_validate({**config.model_dump(), **updates})


def _room_cfg(spec: RoomConfig) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Room",
        spawn=sim_utils.UsdFileCfg(
            usd_path=spec.usd_path,
            scale=(spec.scale, spec.scale, spec.scale),
        ),
    )


def _surface_cfg(prim_path: str, spec: SurfaceConfig) -> AssetBaseCfg:
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
                sim_utils.MdlFileCfg(mdl_path=spec.material_path)
                if spec.material_path is not None
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
        spawn=sim_utils.UsdFileCfg(usd_path=spec.stand_usd_path),
    )


def _overhead_camera_cfg(spec: OverheadCameraConfig) -> CameraCfg:
    profile = load_config(spec.profile_path, CameraConfig)
    return build_camera_cfg(
        profile,
        prim_path="{ENV_REGEX_NS}/CameraStand/OverheadCamera",
        position_m=spec.sensor_local_position_m,
        orientation_xyzw=spec.sensor_local_orientation_xyzw,
        convention=spec.convention,
    )


def _light_cfg(spec: LightingConfig) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="/World/EnvironmentLight",
        spawn=sim_utils.DomeLightCfg(
            texture_file=spec.texture_path,
            intensity=spec.intensity,
        ),
    )


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


__all__ = ["DualArmTabletopSceneCfg", "build_scene_cfg"]
