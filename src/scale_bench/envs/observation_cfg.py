"""Compile profiles and scene sensors into an Observation Manager configuration."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import MISSING

import isaaclab.envs.mdp as mdp
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg, SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.utils.configclass import configclass

from scale_bench.robots import RobotProfile

from .mdp.observations import camera_image, gripper_joint_pos


@configclass
class ObservationsCfg:
    """Observation groups consumed by the Isaac Lab Observation Manager."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Named policy inputs with no privileged task state."""

        left_arm_joint_pos: ObservationTermCfg = MISSING
        left_gripper_joint_pos: ObservationTermCfg = MISSING
        right_arm_joint_pos: ObservationTermCfg = MISSING
        right_gripper_joint_pos: ObservationTermCfg = MISSING
        left_robot_camera_rgb: ObservationTermCfg | None = None
        left_robot_camera_depth: ObservationTermCfg | None = None
        right_robot_camera_rgb: ObservationTermCfg | None = None
        right_robot_camera_depth: ObservationTermCfg | None = None
        overhead_camera_rgb: ObservationTermCfg | None = None
        overhead_camera_depth: ObservationTermCfg | None = None

        def __post_init__(self) -> None:
            self.concatenate_terms = False
            self.enable_corruption = False

    policy: PolicyCfg = MISSING


def create_observations_cfg(
    *,
    left_robot_profile: RobotProfile,
    right_robot_profile: RobotProfile,
    scene_cfg: InteractiveSceneCfg,
) -> ObservationsCfg:
    """Build policy observation term configs in their public output order."""

    terms = {
        **_robot_observation_terms("left", left_robot_profile),
        **_robot_observation_terms("right", right_robot_profile),
    }
    camera_names = []
    if left_robot_profile.camera is not None:
        camera_names.append("left_robot_camera")
    if right_robot_profile.camera is not None:
        camera_names.append("right_robot_camera")
    camera_names.append("overhead_camera")
    terms.update(_camera_observation_terms(scene_cfg, camera_names))
    return ObservationsCfg(policy=ObservationsCfg.PolicyCfg(**terms))


def _robot_observation_terms(
    side: str,
    profile: RobotProfile,
) -> dict[str, ObservationTermCfg]:
    asset_name = f"{side}_robot"
    arm_cfg = SceneEntityCfg(
        asset_name,
        joint_names=list(profile.kinematics.arm_joint_names),
        preserve_order=True,
    )
    gripper_cfg = SceneEntityCfg(
        asset_name,
        joint_names=list(profile.gripper.joint_names),
        preserve_order=True,
    )
    return {
        f"{side}_arm_joint_pos": ObservationTermCfg(
            func=mdp.joint_pos,
            params={"asset_cfg": arm_cfg},
        ),
        f"{side}_gripper_joint_pos": ObservationTermCfg(
            func=gripper_joint_pos,
            params={"asset_cfg": gripper_cfg},
        ),
    }


def _camera_observation_terms(
    scene_cfg: InteractiveSceneCfg,
    camera_names: Sequence[str],
) -> dict[str, ObservationTermCfg]:
    terms: dict[str, ObservationTermCfg] = {}
    for camera_name in camera_names:
        camera_cfg = getattr(scene_cfg, camera_name, None)
        _validate_policy_camera(camera_name, camera_cfg)
        terms[f"{camera_name}_rgb"] = _camera_term(camera_name, "rgb")
        terms[f"{camera_name}_depth"] = _camera_term(
            camera_name,
            "distance_to_image_plane",
        )
    return terms


def _camera_term(camera_name: str, data_type: str) -> ObservationTermCfg:
    return ObservationTermCfg(
        func=camera_image,
        params={
            "sensor_cfg": SceneEntityCfg(camera_name),
            "data_type": data_type,
        },
    )


def _validate_policy_camera(
    camera_name: str,
    camera_cfg: CameraCfg | None,
) -> None:
    if not isinstance(camera_cfg, CameraCfg):
        raise ValueError(f"policy camera {camera_name!r} is missing from the scene")
    required = {"rgb", "distance_to_image_plane"}
    missing = required - set(camera_cfg.data_types)
    if missing:
        raise ValueError(
            f"policy camera {camera_name!r} must enable RGB-D; missing "
            f"{sorted(missing)}"
        )


__all__ = ["ObservationsCfg", "create_observations_cfg"]
