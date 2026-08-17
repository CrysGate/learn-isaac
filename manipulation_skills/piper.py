"""Piper runtime adapter and factories for the atomic skill library."""

from __future__ import annotations

from typing import Any

import torch
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.sim.views import FrameView
from isaaclab.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    quat_from_matrix,
    quat_inv,
    subtract_frame_transforms,
)

from scale_bench.config.models.robot import RobotConfig

from .pick import (
    GraspCandidate,
    PickConfig,
    PickSkill,
    Pose,
    ResolvedGrasp,
)


DEFAULT_MAX_JOINT_STEP_RAD = 0.04


class _ActionCodec:
    """Build full environment actions from named manager terms."""

    def __init__(self, env: Any) -> None:
        self.env = env
        self._terms = {
            item["name"]: item for item in env.get_IO_descriptors["actions"]
        }

    def hold(self) -> torch.Tensor:
        action = torch.empty(
            (self.env.num_envs, self.env.action_manager.total_action_dim),
            device=self.env.device,
            dtype=torch.float32,
        )
        for name, descriptor in self._terms.items():
            term = self.env.action_manager.get_term(name)
            asset = self.env.scene[term.cfg.asset_name]
            joint_ids, joint_names = asset.find_joints(
                descriptor["joint_names"],
                preserve_order=True,
            )
            if joint_names != descriptor["joint_names"]:
                raise RuntimeError(f"action term {name!r} changed joint order")
            start, stop = descriptor["slice"]
            action[:, start:stop] = asset.data.joint_pos.torch[:, joint_ids]
        return action

    def set(
        self,
        action: torch.Tensor,
        term_name: str,
        env_id: int,
        values: torch.Tensor,
    ) -> None:
        descriptor = self._terms[term_name]
        start, stop = descriptor["slice"]
        if values.shape != (stop - start,):
            raise ValueError(
                f"{term_name} expects {stop - start} values, got {tuple(values.shape)}"
            )
        action[env_id, start:stop] = values


class PiperRuntime:
    """Read Piper state and convert TCP targets to manager joint actions."""

    def __init__(
        self,
        env: Any,
        profile: RobotConfig,
        robot: str,
        env_id: int,
        max_joint_step_rad: float,
    ) -> None:
        if profile.name.lower() != "piper":
            raise ValueError(f"PiperRuntime requires a Piper profile, got {profile.name!r}")
        if env_id < 0 or env_id >= env.num_envs:
            raise IndexError(f"env_id must be in [0, {env.num_envs})")

        self.env = env
        self.profile = profile
        self.env_id = env_id
        self.step_dt = float(env.step_dt)
        self.max_gripper_aperture_m = float(profile.gripper.max_aperture_m)
        self.max_joint_step_rad = max_joint_step_rad
        self.robot_name = _robot_name(robot)
        self.side = self.robot_name.removesuffix("_robot")
        self.robot = env.scene[self.robot_name]
        self.arm_term = f"{self.side}_arm"
        self.gripper_term = f"{self.side}_gripper"
        self.codec = _ActionCodec(env)

        self.arm_joint_ids, arm_names = self.robot.find_joints(
            list(profile.kinematics.arm_joint_names),
            preserve_order=True,
        )
        if arm_names != list(profile.kinematics.arm_joint_names):
            raise RuntimeError("Piper arm joints do not match RobotConfig order")
        body_ids, body_names = self.robot.find_bodies(profile.kinematics.ee_body)
        if len(body_ids) != 1:
            raise RuntimeError(
                f"expected one Piper EE body, found {body_names}"
            )
        self.ee_body_id = body_ids[0]
        self.jacobian_body_id = (
            self.ee_body_id - 1 if self.robot.is_fixed_base else self.ee_body_id
        )
        self.jacobian_joint_ids = [
            joint_id + self.robot.num_base_dofs for joint_id in self.arm_joint_ids
        ]
        self.ik = DifferentialIKController(
            DifferentialIKControllerCfg(
                command_type="pose",
                use_relative_mode=False,
                ik_method="dls",
            ),
            num_envs=1,
            device=env.device,
        )
        self.ee_to_tcp = self._resolve_ee_to_tcp()

    def hold_action(self, gripper_open: bool | None = None) -> torch.Tensor:
        action = self.codec.hold()
        if gripper_open is not None:
            self.codec.set(
                action,
                self.gripper_term,
                self.env_id,
                self._gripper_target(gripper_open),
            )
        return action

    def move_action(
        self,
        target_tcp_pose_w: Pose,
        gripper_open: bool | None,
    ) -> torch.Tensor:
        action = self.codec.hold()
        self.codec.set(
            action,
            self.arm_term,
            self.env_id,
            self._arm_target(target_tcp_pose_w),
        )
        if gripper_open is not None:
            self.codec.set(
                action,
                self.gripper_term,
                self.env_id,
                self._gripper_target(gripper_open),
            )
        return action

    def arm_joint_positions(self) -> torch.Tensor:
        return self.robot.data.joint_pos.torch[
            self.env_id,
            self.arm_joint_ids,
        ].clone()

    def home_joint_positions(self) -> torch.Tensor:
        return torch.tensor(
            [
                self.profile.initial_joint_positions[name]
                for name in self.profile.kinematics.arm_joint_names
            ],
            device=self.env.device,
            dtype=torch.float32,
        )

    def joint_action(
        self,
        joint_positions: torch.Tensor,
        gripper_open: bool | None = None,
    ) -> torch.Tensor:
        if joint_positions.shape != (len(self.arm_joint_ids),):
            raise ValueError(
                f"expected {len(self.arm_joint_ids)} arm joints, "
                f"got {tuple(joint_positions.shape)}"
            )
        current = self.arm_joint_positions()
        target = current + (joint_positions.to(current) - current).clamp(
            -self.max_joint_step_rad,
            self.max_joint_step_rad,
        )
        limits = self.robot.data.soft_joint_pos_limits.torch[
            self.env_id,
            self.arm_joint_ids,
        ]
        target = target.clamp(limits[:, 0], limits[:, 1])
        action = self.codec.hold()
        self.codec.set(action, self.arm_term, self.env_id, target)
        if gripper_open is not None:
            self.codec.set(
                action,
                self.gripper_term,
                self.env_id,
                self._gripper_target(gripper_open),
            )
        return action

    def tcp_pose_w(self) -> Pose:
        ee_pose = self.robot.data.body_pose_w.torch[
            self.env_id : self.env_id + 1,
            self.ee_body_id,
        ]
        tcp_pos, tcp_quat = combine_frame_transforms(
            ee_pose[:, :3],
            ee_pose[:, 3:7],
            self.ee_to_tcp[0],
            self.ee_to_tcp[1],
        )
        return tcp_pos[0], tcp_quat[0]

    def object_pose_w(self, object_name: str) -> Pose:
        try:
            pose = self.env.scene[object_name].data.root_pose_w.torch[self.env_id]
        except (KeyError, AttributeError) as error:
            raise ValueError(f"scene object {object_name!r} is not a rigid asset") from error
        return pose[:3], pose[3:7]

    def gripper_aperture_m(self) -> float:
        gripper = self.profile.gripper
        ids, _ = self.robot.find_joints(
            list(gripper.command_joint_names),
            preserve_order=True,
        )
        positions = self.robot.data.joint_pos.torch[self.env_id, ids]
        fractions = []
        for index, name in enumerate(gripper.command_joint_names):
            closed = gripper.closed_positions[name]
            opened = gripper.open_positions[name]
            fractions.append((positions[index] - closed) / (opened - closed))
        openness = torch.stack(fractions).mean().clamp(0.0, 1.0).item()
        return float(
            gripper.min_aperture_m
            + openness * (gripper.max_aperture_m - gripper.min_aperture_m)
        )

    def default_grasp(
        self,
        object_name: str,
        height_offset_m: float,
    ) -> ResolvedGrasp:
        object_pos, _ = self.object_pose_w(object_name)
        base_pos = self.robot.data.root_pos_w.torch[self.env_id]
        horizontal = object_pos - base_pos
        horizontal[2] = 0.0
        norm = torch.linalg.vector_norm(horizontal)
        if norm < 1.0e-6:
            raise RuntimeError("cannot infer a horizontal approach direction")
        world_up = torch.tensor((0.0, 0.0, 1.0), device=self.env.device)
        x_axis = 0.67 * horizontal / norm - 0.74 * world_up
        x_axis /= torch.linalg.vector_norm(x_axis)
        y_axis = torch.linalg.cross(world_up, x_axis)
        y_axis /= torch.linalg.vector_norm(y_axis)
        z_axis = torch.linalg.cross(x_axis, y_axis)
        rotation = torch.stack((x_axis, y_axis, z_axis), dim=1)
        orientation = quat_from_matrix(rotation.unsqueeze(0))[0]
        position = object_pos.clone()
        position[2] += height_offset_m
        return ResolvedGrasp((position, orientation), x_axis)

    def _arm_target(self, target_tcp_pose_w: Pose) -> torch.Tensor:
        tcp_to_ee = subtract_frame_transforms(self.ee_to_tcp[0], self.ee_to_tcp[1])
        desired_ee_pos_w, desired_ee_quat_w = combine_frame_transforms(
            target_tcp_pose_w[0].unsqueeze(0),
            target_tcp_pose_w[1].unsqueeze(0),
            tcp_to_ee[0],
            tcp_to_ee[1],
        )
        root_pose = self.robot.data.root_pose_w.torch[
            self.env_id : self.env_id + 1
        ]
        desired_ee_pos_b, desired_ee_quat_b = subtract_frame_transforms(
            root_pose[:, :3],
            root_pose[:, 3:7],
            desired_ee_pos_w,
            desired_ee_quat_w,
        )
        ee_pose = self.robot.data.body_pose_w.torch[
            self.env_id : self.env_id + 1,
            self.ee_body_id,
        ]
        current_ee_pos_b, current_ee_quat_b = subtract_frame_transforms(
            root_pose[:, :3],
            root_pose[:, 3:7],
            ee_pose[:, :3],
            ee_pose[:, 3:7],
        )

        jacobian = self.robot.data.body_link_jacobian_w.torch[
            self.env_id : self.env_id + 1,
            self.jacobian_body_id,
        ][:, :, self.jacobian_joint_ids].clone()
        base_rotation = matrix_from_quat(quat_inv(root_pose[:, 3:7]))
        jacobian[:, :3] = torch.bmm(base_rotation, jacobian[:, :3])
        jacobian[:, 3:] = torch.bmm(base_rotation, jacobian[:, 3:])

        joint_pos = self.robot.data.joint_pos.torch[
            self.env_id : self.env_id + 1,
            self.arm_joint_ids,
        ]
        command = torch.cat((desired_ee_pos_b, desired_ee_quat_b), dim=1)
        self.ik.set_command(command)
        target = self.ik.compute(
            current_ee_pos_b,
            current_ee_quat_b,
            jacobian,
            joint_pos,
        )
        target = joint_pos + (target - joint_pos).clamp(
            -self.max_joint_step_rad,
            self.max_joint_step_rad,
        )
        limits = self.robot.data.soft_joint_pos_limits.torch[
            self.env_id,
            self.arm_joint_ids,
        ]
        return target[0].clamp(limits[:, 0], limits[:, 1])

    def _gripper_target(self, opened: bool) -> torch.Tensor:
        positions = (
            self.profile.gripper.open_positions
            if opened
            else self.profile.gripper.closed_positions
        )
        return torch.tensor(
            [positions[name] for name in self.profile.gripper.command_joint_names],
            device=self.env.device,
            dtype=torch.float32,
        )

    def _resolve_ee_to_tcp(self) -> Pose:
        frame_path = (
            f"{self.robot.cfg.prim_path}/"
            f"{self.profile.kinematics.ee_body}/"
            f"{self.profile.kinematics.tcp.parent_frame}"
        )
        frame = FrameView(frame_path, device=self.env.device)
        if frame.count != self.env.num_envs:
            raise RuntimeError(
                f"TCP parent frame {frame_path!r} matched {frame.count} prims; "
                f"expected {self.env.num_envs}"
            )
        parent_pos_w, parent_quat_w = frame.get_world_poses()
        ee_pose_w = self.robot.data.body_pose_w.torch[:, self.ee_body_id]
        ee_to_parent = subtract_frame_transforms(
            ee_pose_w[:, :3],
            ee_pose_w[:, 3:7],
            parent_pos_w.torch,
            parent_quat_w.torch,
        )
        tcp = self.profile.kinematics.tcp
        tcp_pos = ee_pose_w.new_tensor(tcp.position_m).repeat(self.env.num_envs, 1)
        tcp_quat = ee_pose_w.new_tensor(tcp.orientation_xyzw).repeat(
            self.env.num_envs, 1
        )
        ee_to_tcp = combine_frame_transforms(
            ee_to_parent[0],
            ee_to_parent[1],
            tcp_pos,
            tcp_quat,
        )
        return (
            ee_to_tcp[0][self.env_id : self.env_id + 1].clone(),
            ee_to_tcp[1][self.env_id : self.env_id + 1].clone(),
        )


def pick(
    env: Any,
    robot_profile: RobotConfig,
    *,
    robot: str,
    object_name: str,
    env_id: int = 0,
    grasp: GraspCandidate | None = None,
    config: PickConfig | None = None,
) -> PickSkill:
    """Create a tick-driven Piper pick skill for an initialized environment."""

    resolved_config = config or PickConfig()
    runtime = PiperRuntime(
        env,
        robot_profile,
        robot,
        env_id,
        resolved_config.max_joint_step_rad,
    )
    return PickSkill(
        runtime,
        object_name,
        grasp=grasp,
        config=resolved_config,
    )


def move_to_pose(
    env: Any,
    robot_profile: RobotConfig,
    *,
    robot: str,
    target_pose_w: Pose,
    env_id: int = 0,
    gripper_open: bool | None = None,
    config: Any = None,
    max_joint_step_rad: float = DEFAULT_MAX_JOINT_STEP_RAD,
) -> Any:
    """Create a Cartesian move skill for one Piper arm."""

    from .motion import MoveToPoseSkill

    runtime = PiperRuntime(
        env,
        robot_profile,
        robot,
        env_id,
        max_joint_step_rad,
    )
    return MoveToPoseSkill(
        runtime,
        target_pose_w,
        gripper_open=gripper_open,
        config=config,
    )


def open_gripper(
    env: Any,
    robot_profile: RobotConfig,
    *,
    robot: str,
    env_id: int = 0,
    config: Any = None,
) -> Any:
    """Create an open-gripper skill for one Piper arm."""

    from .gripper import OpenGripperSkill

    runtime = PiperRuntime(
        env,
        robot_profile,
        robot,
        env_id,
        DEFAULT_MAX_JOINT_STEP_RAD,
    )
    return OpenGripperSkill(runtime, config=config)


def close_gripper(
    env: Any,
    robot_profile: RobotConfig,
    *,
    robot: str,
    env_id: int = 0,
    require_contact: bool = False,
    config: Any = None,
) -> Any:
    """Create a close-gripper skill for one Piper arm."""

    from .gripper import CloseGripperSkill

    runtime = PiperRuntime(
        env,
        robot_profile,
        robot,
        env_id,
        DEFAULT_MAX_JOINT_STEP_RAD,
    )
    return CloseGripperSkill(
        runtime,
        require_contact=require_contact,
        config=config,
    )


def place(
    env: Any,
    robot_profile: RobotConfig,
    *,
    robot: str,
    object_name: str,
    target_object_pose_w: Pose,
    env_id: int = 0,
    placement_direction_w: torch.Tensor | None = None,
    config: Any = None,
    max_joint_step_rad: float = DEFAULT_MAX_JOINT_STEP_RAD,
) -> Any:
    """Create a place skill for an object currently held by one Piper arm."""

    from .place import PlaceSkill

    runtime = PiperRuntime(
        env,
        robot_profile,
        robot,
        env_id,
        max_joint_step_rad,
    )
    return PlaceSkill(
        runtime,
        object_name,
        target_object_pose_w,
        placement_direction_w=placement_direction_w,
        config=config,
    )


def insert(
    env: Any,
    robot_profile: RobotConfig,
    *,
    robot: str,
    object_name: str,
    target_object_pose_w: Pose,
    insertion_direction_w: torch.Tensor,
    env_id: int = 0,
    config: Any = None,
    max_joint_step_rad: float = DEFAULT_MAX_JOINT_STEP_RAD,
) -> Any:
    """Create a straight-line insertion skill for one Piper arm."""

    from .insert import InsertSkill

    runtime = PiperRuntime(
        env,
        robot_profile,
        robot,
        env_id,
        max_joint_step_rad,
    )
    return InsertSkill(
        runtime,
        object_name,
        target_object_pose_w,
        insertion_direction_w,
        config=config,
    )


def rotate(
    env: Any,
    robot_profile: RobotConfig,
    *,
    robot: str,
    object_name: str,
    target_object_orientation_wxyz: torch.Tensor,
    env_id: int = 0,
    config: Any = None,
    max_joint_step_rad: float = DEFAULT_MAX_JOINT_STEP_RAD,
) -> Any:
    """Create an in-hand object rotation skill for one Piper arm."""

    from .rotate import RotateSkill

    runtime = PiperRuntime(
        env,
        robot_profile,
        robot,
        env_id,
        max_joint_step_rad,
    )
    return RotateSkill(
        runtime,
        object_name,
        target_object_orientation_wxyz,
        config=config,
    )


def home(
    env: Any,
    robot_profile: RobotConfig,
    *,
    robot: str,
    env_id: int = 0,
    gripper_open: bool | None = None,
    config: Any = None,
    max_joint_step_rad: float = DEFAULT_MAX_JOINT_STEP_RAD,
) -> Any:
    """Create a joint-space home skill for one Piper arm."""

    from .motion import HomeSkill

    runtime = PiperRuntime(
        env,
        robot_profile,
        robot,
        env_id,
        max_joint_step_rad,
    )
    return HomeSkill(runtime, gripper_open=gripper_open, config=config)


def _robot_name(robot: str) -> str:
    aliases = {
        "left": "left_robot",
        "left_robot": "left_robot",
        "right": "right_robot",
        "right_robot": "right_robot",
    }
    try:
        return aliases[robot]
    except KeyError as error:
        raise ValueError("robot must be left, right, left_robot, or right_robot") from error


__all__ = [
    "DEFAULT_MAX_JOINT_STEP_RAD",
    "PiperRuntime",
    "close_gripper",
    "home",
    "insert",
    "move_to_pose",
    "open_gripper",
    "pick",
    "place",
    "rotate",
]
