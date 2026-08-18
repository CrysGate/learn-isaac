"""Runtime control surface for an assembled evaluation gripper."""

from __future__ import annotations

from dataclasses import dataclass

from isaacsim.replicator.grasping import grasping_utils
from pxr import Usd

from grasp_data_gen.isaac.geometry import Pose
from grasp_data_gen.isaac.prim_poses import relative_world_pose
from scale_bench.config.models.robot import RobotConfig


@dataclass(frozen=True)
class DynamicGripper:
    """The assembled gripper prims plus its small control surface."""

    stage: Usd.Stage
    root_prim: Usd.Prim
    base_prim: Usd.Prim
    link_prims: dict[str, Usd.Prim]
    joint_prims: dict[str, Usd.Prim]
    base_to_tcp: Pose
    base_to_camera: Pose | None
    robot: RobotConfig

    def set_joint_positions(self, positions: dict[str, float]) -> None:
        for joint_name, position in positions.items():
            joint = self.joint_prims[joint_name]
            if not grasping_utils.set_joint_state(
                joint,
                position_value=position,
                velocity_value=0.0,
            ):
                raise RuntimeError(f"failed to set joint state: {joint.GetPath()}")

    def set_drive_targets(self, targets: dict[str, float]) -> None:
        for joint_name, target in targets.items():
            grasping_utils.set_joint_drive_parameters(
                self.joint_prims[joint_name],
                target_value=target,
                target_type="position",
            )

    def joint_positions(self) -> dict[str, float]:
        return {
            name: float(grasping_utils.get_joint_state(prim)[0])
            for name, prim in self.joint_prims.items()
        }

    def command_openness(self) -> dict[str, float]:
        config = self.robot.gripper
        positions = self.joint_positions()
        return {
            name: (positions[name] - config.closed_positions[name])
            / (config.open_positions[name] - config.closed_positions[name])
            for name in config.command_joint_names
        }

    def aperture(self) -> float:
        config = self.robot.gripper
        openness = self.command_openness()
        mean_openness = sum(openness.values()) / len(openness)
        return config.min_aperture_m + mean_openness * (
            config.max_aperture_m - config.min_aperture_m
        )

    def open(self) -> None:
        config = self.robot.gripper
        positions = {
            name: self.robot.initial_joint_positions[name]
            for name in config.joint_names
        }
        positions.update(config.open_positions)
        self.set_drive_targets(config.open_positions)
        self.set_joint_positions(positions)

    def close(self) -> None:
        self.set_drive_targets(self.robot.gripper.closed_positions)

    def link_pose_base(self, link_name: str) -> Pose:
        return relative_world_pose(self.base_prim, self.link_prims[link_name])

    def link_poses_base(self) -> dict[str, Pose]:
        return {
            name: self.link_pose_base(name)
            for name in self.link_prims
        }
