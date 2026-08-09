"""Compile robot profiles into an Isaac Lab Action Manager configuration."""

from __future__ import annotations

import re
from dataclasses import MISSING
from typing import Literal

from isaaclab.envs.mdp.actions import JointPositionActionCfg
from isaaclab.utils.configclass import configclass

from scale_bench.robots import RobotProfile


ArmActionMode = Literal["joint_position"]


@configclass
class ActionsCfg:
    """Action terms consumed directly by the Isaac Lab Action Manager."""

    left_arm: JointPositionActionCfg = MISSING
    left_gripper: JointPositionActionCfg = MISSING
    right_arm: JointPositionActionCfg = MISSING
    right_gripper: JointPositionActionCfg = MISSING


def create_actions_cfg(
    *,
    left_robot_profile: RobotProfile,
    right_robot_profile: RobotProfile,
    arm_action_mode: ArmActionMode,
) -> ActionsCfg:
    """Build action term configs from the selected profiles and control mode."""

    if arm_action_mode != "joint_position":
        raise ValueError(f"unsupported arm action mode: {arm_action_mode}")
    return ActionsCfg(
        left_arm=_arm_action_cfg("left_robot", left_robot_profile),
        left_gripper=_gripper_action_cfg("left_robot", left_robot_profile),
        right_arm=_arm_action_cfg("right_robot", right_robot_profile),
        right_gripper=_gripper_action_cfg("right_robot", right_robot_profile),
    )


def _arm_action_cfg(
    asset_name: str,
    profile: RobotProfile,
) -> JointPositionActionCfg:
    return JointPositionActionCfg(
        asset_name=asset_name,
        joint_names=list(profile.kinematics.arm_joint_names),
        preserve_order=True,
        use_default_offset=False,
        scale=1.0,
        offset=0.0,
    )


def _gripper_action_cfg(
    asset_name: str,
    profile: RobotProfile,
) -> JointPositionActionCfg:
    gripper = profile.gripper
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


__all__ = ["ActionsCfg", "ArmActionMode", "create_actions_cfg"]
