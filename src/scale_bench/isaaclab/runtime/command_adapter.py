"""Build pure command action layouts from initialized Isaac adapters."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from isaaclab.envs import ManagerBasedEnv

from scale_bench.config.models.robot import RobotConfig
from scale_bench.skills.executor import CommandActionLayout


def build_command_action_layout(
    env: ManagerBasedEnv,
    *,
    left_robot_config: RobotConfig,
    right_robot_config: RobotConfig,
) -> CommandActionLayout:
    """Resolve dual-arm slices and semantic gripper targets."""

    descriptors = env.get_IO_descriptors["actions"]
    by_name = {
        descriptor["name"]: descriptor
        for descriptor in descriptors
    }
    expected_names = {
        "left_arm",
        "left_gripper",
        "right_arm",
        "right_gripper",
    }
    if set(by_name) != expected_names:
        raise RuntimeError("action descriptors do not match the dual-arm contract")
    _validate_descriptor_joints(by_name["left_arm"], left_robot_config, arm=True)
    _validate_descriptor_joints(by_name["left_gripper"], left_robot_config, arm=False)
    _validate_descriptor_joints(by_name["right_arm"], right_robot_config, arm=True)
    _validate_descriptor_joints(by_name["right_gripper"], right_robot_config, arm=False)
    return CommandActionLayout(
        action_dim=env.action_manager.total_action_dim,
        left_arm=tuple(by_name["left_arm"]["slice"]),
        left_gripper=tuple(by_name["left_gripper"]["slice"]),
        right_arm=tuple(by_name["right_arm"]["slice"]),
        right_gripper=tuple(by_name["right_gripper"]["slice"]),
        left_gripper_open=_gripper_target(left_robot_config, closed=False),
        left_gripper_closed=_gripper_target(left_robot_config, closed=True),
        right_gripper_open=_gripper_target(right_robot_config, closed=False),
        right_gripper_closed=_gripper_target(right_robot_config, closed=True),
    )


def _validate_descriptor_joints(
    descriptor: Mapping[str, Any],
    config: RobotConfig,
    *,
    arm: bool,
) -> None:
    expected = (
        config.kinematics.arm_joint_names
        if arm
        else config.gripper.command_joint_names
    )
    if tuple(descriptor["joint_names"]) != expected:
        raise RuntimeError(
            f"action descriptor joints do not match robot profile: "
            f"{descriptor['joint_names']} != {list(expected)}"
        )


def _gripper_target(
    config: RobotConfig,
    *,
    closed: bool,
) -> tuple[float, ...]:
    positions = (
        config.gripper.closed_positions
        if closed
        else config.gripper.open_positions
    )
    return tuple(
        positions[name] for name in config.gripper.command_joint_names
    )


__all__ = ["build_command_action_layout"]
