"""Build Isaac Lab Action Manager configuration from pure robot data."""

from __future__ import annotations

import re
from dataclasses import MISSING
from typing import Literal

from isaaclab.envs.mdp.actions import JointPositionActionCfg
from isaaclab.utils.configclass import configclass

from scale_bench.config.models.robot import RobotConfig


ArmActionMode = Literal["joint_position"]


@configclass
class ActionsCfg:
    """Action terms consumed directly by the Isaac Lab Action Manager."""

    left_arm: JointPositionActionCfg = MISSING
    left_gripper: JointPositionActionCfg = MISSING
    right_arm: JointPositionActionCfg = MISSING
    right_gripper: JointPositionActionCfg = MISSING


def build_actions_cfg(
    *,
    left_robot_config: RobotConfig,
    right_robot_config: RobotConfig,
    arm_action_mode: ArmActionMode,
) -> ActionsCfg:
    """Build action term configs in their fixed public order."""

    if arm_action_mode != "joint_position":
        raise ValueError(f"unsupported arm action mode: {arm_action_mode}")
    return ActionsCfg(
        left_arm=_arm_action_cfg("left_robot", left_robot_config),
        left_gripper=_gripper_action_cfg("left_robot", left_robot_config),
        right_arm=_arm_action_cfg("right_robot", right_robot_config),
        right_gripper=_gripper_action_cfg("right_robot", right_robot_config),
    )


def _arm_action_cfg(
    asset_name: str,
    config: RobotConfig,
) -> JointPositionActionCfg:
    return JointPositionActionCfg(
        asset_name=asset_name,
        joint_names=list(config.kinematics.arm_joint_names),
        preserve_order=True,
        use_default_offset=False,
        scale=1.0,
        offset=0.0,
    )


def _gripper_action_cfg(
    asset_name: str,
    config: RobotConfig,
) -> JointPositionActionCfg:
    gripper = config.gripper
    joint_names = list(gripper.command_joint_names)
    return JointPositionActionCfg(
        asset_name=asset_name,
        joint_names=joint_names,
        preserve_order=True,
        use_default_offset=False,
        scale=1.0,
        offset=0.0,
        clip={
            re.escape(name): tuple(
                sorted(
                    (
                        gripper.closed_positions[name],
                        gripper.open_positions[name],
                    )
                )
            )
            for name in joint_names
        },
    )


# Compatibility with the previous factory name.
__all__ = ["ActionsCfg", "ArmActionMode", "build_actions_cfg"]
