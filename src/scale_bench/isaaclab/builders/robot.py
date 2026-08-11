"""Convert pure robot configuration into native Isaac Lab cfg objects."""

from __future__ import annotations

from collections.abc import Mapping

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sensors import CameraCfg
from isaaclab_physx.sim.schemas import (
    PhysxArticulationRootPropertiesCfg,
    PhysxRigidBodyPropertiesCfg,
)

from scale_bench.config.loader import load_config
from scale_bench.config.models.camera import CameraConfig
from scale_bench.config.models.robot import ActuatorValue, RobotConfig
from scale_bench.isaaclab.builders.camera import build_camera_cfg


def build_robot_cfg(
    config: RobotConfig,
    *,
    prim_path: str | None = None,
) -> ArticulationCfg:
    """Return a fresh articulation cfg using the USD's authored dimensions."""

    def copy_value(value: ActuatorValue) -> float | dict[str, float] | None:
        return dict(value) if isinstance(value, Mapping) else value

    actuators = {
        name: ImplicitActuatorCfg(
            joint_names_expr=list(spec.joint_names),
            stiffness=copy_value(spec.stiffness),
            damping=copy_value(spec.damping),
            effort_limit_sim=copy_value(spec.effort_limit_sim),
            velocity_limit_sim=copy_value(spec.velocity_limit_sim),
        )
        for name, spec in config.actuators.items()
    }
    joint_order = (
        *config.kinematics.arm_joint_names,
        *config.gripper.joint_names,
    )
    cfg = ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=config.usd_path,
            rigid_props=PhysxRigidBodyPropertiesCfg(
                disable_gravity=config.disable_gravity
            ),
            articulation_props=PhysxArticulationRootPropertiesCfg(
                fix_root_link=config.fixed_base,
                enabled_self_collisions=config.self_collisions,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                joint_name: config.initial_joint_positions[joint_name]
                for joint_name in joint_order
            }
        ),
        actuators=actuators,
    )
    if prim_path is not None:
        cfg.prim_path = prim_path
    return cfg


def build_mounted_camera_cfg(
    config: RobotConfig,
    *,
    robot_prim_path: str,
) -> CameraCfg | None:
    """Build the configured wrist camera below a robot root."""

    if config.camera is None:
        return None
    root_prim_path = robot_prim_path.rstrip("/")
    if not root_prim_path:
        raise ValueError("robot_prim_path must not be empty")

    mount = config.camera
    profile = load_config(mount.profile_path, CameraConfig)
    return build_camera_cfg(
        profile,
        prim_path=(
            f"{root_prim_path}/{mount.parent_prim_path}/{mount.sensor_prim_name}"
        ),
        position_m=mount.position_m,
        orientation_xyzw=mount.orientation_xyzw,
        convention=mount.convention,
    )


__all__ = [
    "build_mounted_camera_cfg",
    "build_robot_cfg",
]
