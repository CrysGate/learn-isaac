"""Build and validate IO descriptors from initialized runtime objects."""

from __future__ import annotations

import math
from typing import Any

from isaaclab.envs import ManagerBasedEnv
from isaaclab.envs.mdp.actions import JointPositionAction
from isaaclab.sensors import CameraCfg


def build_io_descriptors(
    env: ManagerBasedEnv,
    native_descriptors: dict[str, Any],
) -> dict[str, Any]:
    """Supplement native descriptors with resolved ScaleBench semantics."""

    _annotate_action_descriptors(env, native_descriptors["actions"])
    _annotate_observation_descriptors(env, native_descriptors["observations"])
    camera_update_periods = {
        name: sensor.cfg.update_period
        for name, sensor in env.scene.sensors.items()
        if isinstance(sensor.cfg, CameraCfg)
    }
    render_dt = env.physics_dt * env.cfg.sim.render_interval
    native_descriptors["runtime"] = {
        "physics_dt": env.physics_dt,
        "step_dt": env.step_dt,
        "render_dt": render_dt,
        "physics_frequency_hz": 1.0 / env.physics_dt,
        "step_frequency_hz": 1.0 / env.step_dt,
        "render_frequency_hz": 1.0 / render_dt,
        "control_decimation": env.cfg.decimation,
        "arm_action_mode": env.cfg.arm_action_mode,
        "camera_update_periods": camera_update_periods,
    }
    return native_descriptors


def validate_io_descriptors(
    env: ManagerBasedEnv,
    descriptors: dict[str, Any],
) -> None:
    """Fail when initialized managers or sensors violate the public contract."""

    expected_action_terms = [
        "left_arm",
        "left_gripper",
        "right_arm",
        "right_gripper",
    ]
    if env.action_manager.active_terms != expected_action_terms:
        raise RuntimeError(
            "action term order does not match the public contract: "
            f"{env.action_manager.active_terms}"
        )
    if env.observation_manager.group_obs_concatenate["policy"]:
        raise RuntimeError("policy observation terms must not be concatenated")

    action_dims = [item["shape"][0] for item in descriptors["actions"]]
    if action_dims != env.action_manager.action_term_dim:
        raise RuntimeError("action descriptors do not match manager dimensions")

    observation_descriptors = descriptors["observations"]["policy"]
    observation_dims = env.observation_manager.group_obs_term_dim["policy"]
    for descriptor, actual_shape in zip(
        observation_descriptors,
        observation_dims,
        strict=True,
    ):
        if tuple(descriptor["shape"]) != tuple(actual_shape):
            raise RuntimeError(
                f"observation descriptor {descriptor['name']!r} has shape "
                f"{descriptor['shape']}, manager resolved {actual_shape}"
            )
        name = descriptor["name"]
        if name.endswith("_camera_rgb"):
            _validate_camera_descriptor(
                descriptor,
                dtype="torch.uint8",
                channels=3,
            )
        elif name.endswith("_camera_depth"):
            _validate_camera_descriptor(
                descriptor,
                dtype="torch.float32",
                channels=1,
            )

    runtime = descriptors["runtime"]
    if not math.isclose(
        runtime["render_dt"],
        runtime["step_dt"],
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise RuntimeError("runtime render_dt must equal environment step_dt")
    mismatched_cameras = [
        f"{name}={period:g}s"
        for name, period in runtime["camera_update_periods"].items()
        if not math.isclose(
            period,
            runtime["step_dt"],
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
    ]
    if mismatched_cameras:
        raise RuntimeError(
            "runtime camera update periods must equal environment step_dt; "
            f"mismatched: {', '.join(mismatched_cameras)}"
        )


def _annotate_action_descriptors(
    env: ManagerBasedEnv,
    descriptors: list[dict[str, Any]],
) -> None:
    term_names = env.action_manager.active_terms
    term_dims = env.action_manager.action_term_dim
    if len(descriptors) != len(term_names):
        raise RuntimeError("action descriptor count does not match active terms")

    start = 0
    for name, dim, descriptor in zip(
        term_names,
        term_dims,
        descriptors,
        strict=True,
    ):
        descriptor["name"] = name
        descriptor["shape"] = [dim]
        descriptor["slice"] = [start, start + dim]
        term = env.action_manager.get_term(name)
        if isinstance(term, JointPositionAction):
            _annotate_joint_position_descriptor(env, descriptor, term)
        start += dim


def _annotate_joint_position_descriptor(
    env: ManagerBasedEnv,
    descriptor: dict[str, Any],
    term: JointPositionAction,
) -> None:
    asset = env.scene[term.cfg.asset_name]
    joint_ids, joint_names = asset.find_joints(
        term.cfg.joint_names,
        preserve_order=term.cfg.preserve_order,
    )
    if joint_names != term.cfg.joint_names:
        raise RuntimeError(
            f"{term.cfg.asset_name} joints resolved as {joint_names}, "
            f"expected profile order {term.cfg.joint_names}"
        )
    descriptor["action_type"] = "AbsoluteJointPosition"
    descriptor["joint_names"] = list(joint_names)
    descriptor.setdefault("extras", {}).update(
        units="rad or m, joint-dependent",
        limits=asset.data.joint_pos_limits.torch[0, joint_ids]
        .detach()
        .cpu()
        .tolist(),
    )


def _annotate_observation_descriptors(
    env: ManagerBasedEnv,
    descriptors: dict[str, list[dict[str, Any]]],
) -> None:
    for group_name, group_descriptors in descriptors.items():
        term_names = env.observation_manager.active_terms[group_name]
        if len(group_descriptors) != len(term_names):
            raise RuntimeError(
                f"{group_name} observation descriptor count does not match "
                "active terms"
            )
        for name, descriptor in zip(term_names, group_descriptors, strict=True):
            descriptor["name"] = name


def _validate_camera_descriptor(
    descriptor: dict[str, Any],
    *,
    dtype: str,
    channels: int,
) -> None:
    extras = descriptor["extras"]
    if descriptor["dtype"] != dtype:
        raise RuntimeError(
            f"camera observation {descriptor['name']!r} must use {dtype}, "
            f"got {descriptor['dtype']}"
        )
    if descriptor["shape"][-1] != channels or extras.get("layout") != "HWC":
        raise RuntimeError(
            f"camera observation {descriptor['name']!r} has an invalid "
            "layout or channel count"
        )


__all__ = ["build_io_descriptors", "validate_io_descriptors"]
