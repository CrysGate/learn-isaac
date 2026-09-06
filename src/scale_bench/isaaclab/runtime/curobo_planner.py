"""CuRobo single-segment planning over explicit ScaleBench scene facts."""

from __future__ import annotations

import itertools
import logging
import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Any

import torch
import yaml
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.scene import Cuboid
from curobo.scene import Scene as SceneCfg
from curobo.trajectory_optimizer import TrajectoryOptimizerResult
from curobo.types import DeviceCfg, GoalToolPose, ToolPoseCriteria
from curobo.types import JointState as CuroboJointState
from curobo.types import Pose as CuroboPose
from torch import Tensor

from scale_bench.config.models.robot import RobotConfig, TcpConfig
from scale_bench.config.models.scene import RobotMountConfig, SceneConfig
from scale_bench.skills.context import (
    EmptyTool,
    HeldObject,
    JointState,
    JointTrajectory,
    PlanningScene,
    SceneObject,
)
from scale_bench.skills.errors import PlanningError
from scale_bench.skills.geometry import (
    compose_pose,
    conjugate_quaternion_xyzw,
    inverse_pose,
    relative_pose,
    rotate_vector_xyzw,
)
from scale_bench.skills.models import Arm, Pose
from scale_bench.skills.planner import (
    MotionPlanner as MotionPlannerProtocol,
)
from scale_bench.skills.planner import (
    PlanningStage,
)

if TYPE_CHECKING:
    from scale_bench.isaaclab.runtime.curobo_visualization import (
        CuroboPlanningVisualizer,
    )

START_JOINT_LIMIT_TOLERANCE_RAD = 1.0e-5
TCP_FRAME = "scale_bench_tcp"
LOGGER = logging.getLogger(__name__)


class CuroboMotionPlanner(MotionPlannerProtocol):
    """Plan one environment-frame TCP target for one fixed robot mount."""

    def __init__(
        self,
        planner: MotionPlanner,
        mount: RobotMountConfig,
        other_mount: RobotMountConfig,
        tcp: TcpConfig,
        arm: Arm,
        table_top_z_m: float,
        arm_joint_names: tuple[str, ...],
        visualizer: CuroboPlanningVisualizer | None,
    ) -> None:
        self._planner = planner
        self._arm: Arm = arm
        self._arm_base_pose_env = Pose(
            (*mount.position_xy_m, table_top_z_m),
            mount.orientation_xyzw,
        )
        self._other_arm_base_pose_env = Pose(
            (*other_mount.position_xy_m, table_top_z_m),
            other_mount.orientation_xyzw,
        )
        self._tcp_pose_parent = Pose(tcp.position_m, tcp.orientation_xyzw)
        self._joint_names = arm_joint_names
        self._visualizer = visualizer
        attached_indices = (
            planner.kinematics.config.kinematics_config.get_sphere_index_from_link_name(
                "attached_object"
            )
        )
        self._attached_sphere_indices = set(attached_indices.cpu().tolist())

    @property
    def arm(self) -> Arm:
        return self._arm

    def plan_pose(
        self,
        start: JointState,
        target_tcp_pose_env: Pose,
        scene: PlanningScene,
        stage: PlanningStage,
        linear_axis_env: tuple[float, float, float] | None,
    ) -> JointTrajectory:
        """Normalize contact directions; transit uses None and may reorient."""
        if linear_axis_env is not None:
            linear_axis_norm = math.hypot(*linear_axis_env)
            linear_axis_env = tuple(
                component_env / linear_axis_norm for component_env in linear_axis_env
            )
        planning_start = self._planning_start(start.positions, stage)
        collision_cuboids_base = self._sync_scene(scene)
        self._capture_visualization(
            stage,
            planning_start,
            collision_cuboids_base,
        )
        violations = self._configuration_violations(scene, planning_start)
        if violations:
            raise PlanningError(
                self._arm,
                stage,
                f"start state is infeasible: {'; '.join(violations)}",
            )
        current = self._joint_state(planning_start)
        goal = self._goal_from_env_pose(target_tcp_pose_env)
        criteria = self._motion_criteria(target_tcp_pose_env, linear_axis_env)
        try:
            self._planner.update_tool_pose_criteria(
                {frame: criteria for frame in self._planner.tool_frames}
            )
            result = self._planner.plan_pose(goal, current)
            trajectory = self._trajectory(result, stage)
            if linear_axis_env is not None:
                self._validate_tcp_path(
                    trajectory, target_tcp_pose_env, stage, linear_axis_env
                )
            return trajectory
        finally:
            self._planner.update_tool_pose_criteria(
                {
                    frame: ToolPoseCriteria(device_cfg=self._planner.device_cfg)
                    for frame in self._planner.tool_frames
                }
            )

    def _motion_criteria(
        self,
        target_tcp_pose_env: Pose,
        linear_axis_env: tuple[float, float, float] | None,
    ) -> ToolPoseCriteria:
        if linear_axis_env is None:
            return ToolPoseCriteria(device_cfg=self._planner.device_cfg)

        motion_axis_base = rotate_vector_xyzw(
            conjugate_quaternion_xyzw(self._arm_base_pose_env.orientation_xyzw),
            linear_axis_env,
        )
        motion_axis_tcp = rotate_vector_xyzw(
            conjugate_quaternion_xyzw(target_tcp_pose_env.orientation_xyzw),
            linear_axis_env,
        )
        _, axis_index, project_to_goal = max(
            (abs(component), index, project)
            for project, components in (
                (False, motion_axis_base), (True, motion_axis_tcp)
            )
            for index, component in enumerate(components)
        )
        non_terminal_axes_weight = [1.0] * 6
        non_terminal_axes_weight[axis_index] = 0.0
        return ToolPoseCriteria(
            non_terminal_pose_axes_weight_factor=non_terminal_axes_weight,
            project_distance_to_goal=project_to_goal,
            device_cfg=self._planner.device_cfg,
        )

    def _validate_tcp_path(
        self,
        trajectory: JointTrajectory,
        target_tcp_pose_env: Pose,
        stage: PlanningStage,
        linear_axis_env: tuple[float, float, float],
    ) -> None:
        """Check the executed interpolation: pose costs alone are soft constraints."""
        kinematics = self._planner.compute_kinematics(
            self._joint_state(trajectory.positions)
        )
        tcp_poses_base = kinematics.tool_poses.get_link_pose(TCP_FRAME)
        tcp_poses_env = self._curobo_pose(self._arm_base_pose_env).multiply(
            tcp_poses_base
        )
        target_tcp_pose_env_curobo = self._curobo_pose(target_tcp_pose_env)
        position_tolerance_m = self._planner.trajopt_solver.config.position_tolerance
        orientation_tolerance_rad = (
            self._planner.trajopt_solver.config.orientation_tolerance
        )
        motion_axis_env = trajectory.positions.new_tensor(linear_axis_env)
        tcp_offsets_env_m = tcp_poses_env.position - target_tcp_pose_env_curobo.position
        remaining_m = -(tcp_offsets_env_m @ motion_axis_env)
        lateral_error_m = torch.linalg.vector_norm(
            tcp_offsets_env_m + remaining_m[:, None] * motion_axis_env, dim=-1
        ).max()
        overshoot_m = torch.maximum(
            -remaining_m.min(), remaining_m.max() - remaining_m[0]
        )
        reversal_m = (remaining_m - remaining_m.cummin(dim=0).values).max()
        orientation_error_rad = 2.0 * torch.acos(
            (tcp_poses_env.quaternion * target_tcp_pose_env_curobo.quaternion)
            .sum(dim=-1)
            .abs()
            .clamp(max=1.0)
        ).max()
        endpoint_error_m = torch.linalg.vector_norm(tcp_offsets_env_m[-1])
        metrics = {
            "lateral_error_m": float(lateral_error_m),
            "overshoot_m": float(overshoot_m),
            "reversal_m": float(reversal_m),
            "endpoint_error_m": float(endpoint_error_m),
            "orientation_error_rad": float(orientation_error_rad),
        }
        if (
            max(lateral_error_m, overshoot_m, reversal_m, endpoint_error_m)
            > position_tolerance_m
            or orientation_error_rad > orientation_tolerance_rad
        ):
            raise PlanningError(self._arm, stage, f"TCP path constraint failed: {metrics}")
        LOGGER.debug(
            "%s %s TCP path: %s",
            self._arm,
            stage,
            metrics,
            extra={
                "event": "PATH",
                "event_fields": {"arm": self._arm, "stage": stage, **metrics},
            },
        )

    def _curobo_pose(self, frame_pose_parent: Pose) -> CuroboPose:
        return CuroboPose.from_list(
            [*frame_pose_parent.position_m, *frame_pose_parent.orientation_xyzw],
            device_cfg=self._planner.device_cfg,
            q_xyzw=True,
        )

    def plan_joints(
        self,
        start: JointState,
        target_joint_state: JointState,
        scene: PlanningScene,
        stage: PlanningStage,
    ) -> JointTrajectory:
        planning_start = self._planning_start(start.positions, stage)
        collision_cuboids_base = self._sync_scene(scene)
        self._capture_visualization(
            stage,
            planning_start,
            collision_cuboids_base,
        )
        violations = self._configuration_violations(scene, planning_start)
        if violations:
            raise PlanningError(
                self._arm,
                stage,
                f"start state is infeasible: {'; '.join(violations)}",
            )
        result = self._planner.plan_cspace(
            self._joint_state(target_joint_state.positions),
            self._joint_state(planning_start),
        )
        return self._trajectory(result, stage)

    def commit_inspection_stages(
        self,
        stages: tuple[PlanningStage, ...],
    ) -> None:
        if self._visualizer is not None:
            self._visualizer.commit(stages)

    def browse_captured_stages(self) -> None:
        if self._visualizer is not None:
            self._visualizer.browse()

    def _planning_start(self, start: Tensor, stage: str) -> Tensor:
        graph_planner = self._planner.graph_planner
        clipped = start.clamp(
            min=graph_planner.action_bound_lows,
            max=graph_planner.action_bound_highs,
        )
        excess = (start - clipped).abs()
        invalid = excess > START_JOINT_LIMIT_TOLERANCE_RAD
        if invalid.any().item():
            details = ", ".join(
                f"{name}={float(start[index]):.9g} exceeds limit by "
                f"{float(excess[index]):.9g} rad"
                for index, name in enumerate(self._joint_names)
                if invalid[index].item()
            )
            raise PlanningError(
                self._arm,
                stage,
                f"start state is outside joint limits: {details}",
            )
        return clipped

    def _sync_scene(
        self,
        scene: PlanningScene,
    ) -> tuple[Cuboid, list[Cuboid], list[Cuboid], list[Cuboid]]:
        self._sync_tool(scene)
        table, camera_stand, objects, other_robot = self._scene_cuboids(scene)
        self._planner.update_world(
            SceneCfg(cuboid=[table, *camera_stand, *objects, *other_robot])
        )
        return table, camera_stand, objects, other_robot

    def _capture_visualization(
        self,
        stage: PlanningStage,
        planning_start: Tensor,
        collision_cuboids_base: tuple[
            Cuboid,
            list[Cuboid],
            list[Cuboid],
            list[Cuboid],
        ],
    ) -> None:
        if self._visualizer is not None:
            active_robot_spheres_base = self._planner.kinematics.get_robot_as_spheres(
                planning_start.unsqueeze(0),
                filter_valid=False,
            )[0]
            (
                table_cuboid_base,
                camera_stand_cuboids_base,
                object_cuboids_base,
                other_arm_cuboids_base,
            ) = collision_cuboids_base
            self._visualizer.capture(
                stage,
                self._arm,
                active_robot_spheres_base,
                self._attached_sphere_indices,
                self._arm_base_pose_env,
                table_cuboid_base,
                camera_stand_cuboids_base,
                object_cuboids_base,
                other_arm_cuboids_base,
            )

    def _scene_cuboids(
        self,
        scene: PlanningScene,
    ) -> tuple[Cuboid, list[Cuboid], list[Cuboid], list[Cuboid]]:
        table = self._scene_object_cuboid(scene.table, 0.0)
        camera_stand = [
            self._scene_object_cuboid(collision_object_env, 0.0)
            for collision_object_env in scene.camera_stand
        ]
        objects = [
            self._scene_object_cuboid(scene_object, -0.001)
            for scene_object in scene.objects
        ]
        other_robot = self._other_robot_cuboids(scene.other_robot.joints)
        return table, camera_stand, objects, other_robot

    def _configuration_violations(
        self,
        scene: PlanningScene,
        joint_positions: Tensor,
    ) -> tuple[str, ...]:
        """Identify the constraints that reject a joint configuration."""

        table, camera_stand, objects, other_robot = self._scene_cuboids(scene)
        graph_planner = self._planner.graph_planner
        feasible = graph_planner.check_samples_feasibility(joint_positions.unsqueeze(0))
        if feasible.all().item():
            return ()

        below = joint_positions < graph_planner.action_bound_lows
        above = joint_positions > graph_planner.action_bound_highs
        outside = below | above
        if outside.any().item():
            details = ", ".join(
                f"{name}={float(joint_positions[index]):.9g} not in "
                f"[{float(graph_planner.action_bound_lows[index]):.9g}, "
                f"{float(graph_planner.action_bound_highs[index]):.9g}]"
                for index, name in enumerate(self._joint_names)
                if outside[index].item()
            )
            return (f"joint_limits({details})",)

        self._planner.update_world(SceneCfg(cuboid=[]))
        feasible = graph_planner.check_samples_feasibility(joint_positions.unsqueeze(0))
        if not feasible.all().item():
            full_world = [table, *camera_stand, *objects, *other_robot]
            self._planner.update_world(SceneCfg(cuboid=full_world))
            return ("self_collision",)

        groups = [
            ("table", [table]),
            ("camera_stand", camera_stand),
            *(
                (f"object:{scene_object.name}", [cuboid])
                for scene_object, cuboid in zip(
                    scene.objects,
                    objects,
                    strict=True,
                )
            ),
            ("other_arm", other_robot),
        ]
        invalid = []
        for name, cuboids in groups:
            self._planner.update_world(SceneCfg(cuboid=cuboids))
            feasible = graph_planner.check_samples_feasibility(
                joint_positions.unsqueeze(0)
            )
            if not feasible.all().item():
                collision_source = name
                if isinstance(scene.tool, HeldObject):
                    self._planner.disable_link_collision(["attached_object"])
                    robot_feasible = graph_planner.check_samples_feasibility(
                        joint_positions.unsqueeze(0)
                    )
                    self._planner.enable_link_collision(["attached_object"])
                    prefix = "held_object" if robot_feasible.all().item() else "robot"
                    collision_source = f"{prefix}->{name}"
                invalid.append(collision_source)
        full_world = [table, *camera_stand, *objects, *other_robot]
        self._planner.update_world(SceneCfg(cuboid=full_world))
        if not invalid:
            invalid.append("combined_world")
        return tuple(invalid)

    def _sync_tool(self, scene: PlanningScene) -> None:
        if isinstance(scene.tool, EmptyTool):
            self._planner.disable_link_collision(["attached_object"])
            return

        object_spheres_object_m = _cuboid_collision_spheres(
            scene.tool.object.size_m,
            max_spheres=len(self._attached_sphere_indices),
        )
        object_pose_tcp = inverse_pose(scene.tool.tcp_pose_object)
        object_pose_parent = compose_pose(
            self._tcp_pose_parent,
            object_pose_tcp,
        )
        sphere_centers_parent_m = tuple(
            compose_pose(
                object_pose_parent,
                Pose(
                    tuple(float(value) for value in sphere[:3]),
                    (0.0, 0.0, 0.0, 1.0),
                ),
            ).position_m
            for sphere in object_spheres_object_m
        )
        object_spheres_parent_m = self._planner.device_cfg.to_device(
            tuple(
                (*center, float(sphere[3]))
                for center, sphere in zip(
                    sphere_centers_parent_m,
                    object_spheres_object_m,
                    strict=True,
                )
            )
        )
        kinematics = self._planner.kinematics.config.kinematics_config
        # Clear unused reserved slots; writing current positive radii activates them.
        self._planner.disable_link_collision(["attached_object"])
        kinematics.update_link_spheres("attached_object", object_spheres_parent_m)

    def _scene_object_cuboid(
        self,
        scene_object: SceneObject,
        padding_m: float,
    ) -> Cuboid:
        object_pose_base = relative_pose(
            self._arm_base_pose_env,
            scene_object.pose_env,
        )
        return Cuboid(
            name=scene_object.name,
            pose=_curobo_pose(
                object_pose_base.position_m,
                object_pose_base.orientation_xyzw,
            ),
            dims=[dimension + 2.0 * padding_m for dimension in scene_object.size_m],
        )

    def _other_robot_cuboids(self, joints: JointState) -> list[Cuboid]:
        spheres = self._planner.kinematics.get_robot_as_spheres(
            joints.positions.unsqueeze(0),
            filter_valid=False,
        )[0]
        other_base_pose_base = relative_pose(
            self._arm_base_pose_env,
            self._other_arm_base_pose_env,
        )
        cuboids = []
        for index, sphere in enumerate(spheres):
            radius = float(sphere.radius)
            if index in self._attached_sphere_indices or radius <= 0.0:
                continue
            sphere_pose_base = compose_pose(
                other_base_pose_base,
                Pose(
                    tuple(float(value) for value in sphere.pose[:3]),
                    (0.0, 0.0, 0.0, 1.0),
                ),
            )
            cuboids.append(
                Cuboid(
                    name=f"other_arm/{index:03d}",
                    pose=[*sphere_pose_base.position_m, 1.0, 0.0, 0.0, 0.0],
                    dims=[2.0 * radius, 2.0 * radius, 2.0 * radius],
                )
            )
        return cuboids

    def _goal_from_env_pose(
        self,
        target_tcp_pose_env: Pose,
    ) -> GoalToolPose:
        tcp_pose_base = relative_pose(
            self._arm_base_pose_env,
            target_tcp_pose_env,
        )
        return GoalToolPose.from_poses(
            {TCP_FRAME: self._curobo_pose(tcp_pose_base)},
            ordered_tool_frames=self._planner.tool_frames,
        )

    def _joint_state(self, positions: Tensor) -> CuroboJointState:
        return CuroboJointState.from_position(
            positions.unsqueeze(0),
            joint_names=list(self._joint_names),
        )

    def _trajectory(
        self,
        result: TrajectoryOptimizerResult | None,
        stage: str,
    ) -> JointTrajectory:
        """Convert a plan; None means planning produced no trajectory optimizer result."""
        if result is None:
            raise PlanningError(
                self._arm,
                stage,
                "CUROBO_NO_RESULT: planning returned no trajectory optimization "
                "result; failure details are unavailable",
            )
        if result.success is None:
            raise PlanningError(
                self._arm,
                stage,
                "CUROBO_MISSING_SUCCESS: the returned trajectory optimization "
                "result has no success status",
            )
        successful_count = int(result.success.count_nonzero().item())
        if successful_count == 0:
            raise PlanningError(
                self._arm,
                stage,
                "CUROBO_NO_SUCCESSFUL_TRAJECTORY: the latest trajectory "
                "optimization result contains no successful trajectory "
                f"(successful_candidates={successful_count}/{result.success.numel()}); "
                "attempt history is unavailable",
            )
        interpolated = result.get_interpolated_plan()
        indices = tuple(interpolated.joint_names.index(name) for name in self._joint_names)
        positions = (
            interpolated.position.reshape(
                -1,
                interpolated.position.shape[-1],
            )[:, indices].contiguous()
        )
        return JointTrajectory(positions)


def build_curobo_motion_planners(
    *,
    left_robot_config: RobotConfig,
    right_robot_config: RobotConfig,
    scene_config: SceneConfig,
    device: str,
    dtype: torch.dtype,
    interpolation_dt_s: float,
    visualize: bool,
    env_origin_world_m: tuple[float, float, float],
) -> Mapping[Arm, CuroboMotionPlanner]:
    """Share one statelessly synchronized backend per kinematic profile."""

    backends: dict[tuple[Path, str, TcpConfig, tuple[str, ...]], MotionPlanner] = {}
    visualizer = None
    if visualize:
        from scale_bench.isaaclab.runtime.curobo_visualization import (
            CuroboPlanningVisualizer,
        )

        visualizer = CuroboPlanningVisualizer(env_origin_world_m)
    planners = {}
    for arm, robot_config, mount, other_mount in (
        (
            "left",
            left_robot_config,
            scene_config.robot_mounts.left,
            scene_config.robot_mounts.right,
        ),
        (
            "right",
            right_robot_config,
            scene_config.robot_mounts.right,
            scene_config.robot_mounts.left,
        ),
    ):
        joint_names = tuple(robot_config.kinematics.arm_joint_names)
        key = (
            Path(robot_config.urdf_path).resolve(),
            robot_config.kinematics.base_body,
            robot_config.kinematics.tcp,
            joint_names,
        )
        backend = backends.get(key)
        if backend is None:
            backend = _build_backend(
                robot_config,
                device=device,
                dtype=dtype,
                interpolation_dt_s=interpolation_dt_s,
            )
            backends[key] = backend
        planners[arm] = CuroboMotionPlanner(
            backend,
            mount,
            other_mount,
            robot_config.kinematics.tcp,
            arm,
            scene_config.table_top_z_m,
            joint_names,
            visualizer,
        )
    return planners


def _build_backend(
    robot_config: RobotConfig,
    *,
    device: str,
    dtype: torch.dtype,
    interpolation_dt_s: float,
) -> MotionPlanner:
    device_cfg = DeviceCfg(device=device, dtype=dtype)
    cfg = MotionPlannerCfg.create(
        _load_collision_robot_config(robot_config),
        device_cfg=device_cfg,
        collision_cache={"cuboid": 96},
        self_collision_check=True,
        num_ik_seeds=64,
        num_trajopt_seeds=4,
        interpolation_dt=interpolation_dt_s,
        interpolation_buffer_size=500,
    )
    return MotionPlanner(cfg)


def _load_collision_robot_config(robot_config: RobotConfig) -> dict[str, Any]:
    path = Path(robot_config.curobo_config_path).resolve()
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    kinematics = document["kinematics"]
    # required = {
    #     "collision_spheres",
    #     "self_collision_ignore",
    #     "self_collision_buffer",
    #     "cspace",
    #     "extra_collision_spheres",
    #     "extra_links",
    # }
    urdf_path = Path(robot_config.urdf_path).resolve()
    asset_root = Path(kinematics.get("asset_root_path", ""))
    if not asset_root.is_absolute():
        asset_root = (path.parent / asset_root).resolve()
    kinematics["urdf_path"] = str(urdf_path)
    kinematics["asset_root_path"] = str(asset_root)
    kinematics["base_link"] = robot_config.kinematics.base_body
    
    tcp = robot_config.kinematics.tcp
    kinematics["tool_frames"] = [TCP_FRAME]
    kinematics["extra_links"][TCP_FRAME] = {
        "link_name": TCP_FRAME,
        "parent_link_name": tcp.parent_frame,
        "joint_name": f"{TCP_FRAME}_joint",
        "joint_type": "FIXED",
        "fixed_transform": [
            *tcp.position_m,
            tcp.orientation_xyzw[3],
            *tcp.orientation_xyzw[:3],
        ],
    }

    arm_joints = tuple(robot_config.kinematics.arm_joint_names)
    cspace = kinematics["cspace"]
    cspace["default_joint_position"] = list(
        _urdf_reference_positions(urdf_path, arm_joints)
    )
    kinematics["lock_joints"] = {
        name: robot_config.initial_joint_positions[name]
        for name in robot_config.gripper.command_joint_names
    }
    return document


def _urdf_reference_positions(
    urdf_path: Path,
    joint_names: tuple[str, ...],
) -> tuple[float, ...]:
    root = ET.parse(urdf_path).getroot()
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    references = []
    for joint_name in joint_names:
        joint = joints.get(joint_name)
        limit = None if joint is None else joint.find("limit")
        lower = float(limit.get("lower", "nan"))
        upper = float(limit.get("upper", "nan"))
        references.append((lower + upper) / 2.0)
    return tuple(references)


def _cuboid_collision_spheres(
    dims: tuple[float, float, float],
    *,
    max_spheres: int,
) -> tuple[tuple[float, float, float, float], ...]:
    subdivisions = min(
        (
            counts
            for counts in itertools.product(range(1, max_spheres + 1), repeat=3)
            if math.prod(counts) <= max_spheres
        ),
        key=lambda counts: math.sqrt(
            sum(
                (dimension / count) ** 2
                for dimension, count in zip(dims, counts, strict=True)
            )
        ),
    )
    cell_dims = tuple(
        dimension / count for dimension, count in zip(dims, subdivisions, strict=True)
    )
    radius = min(cell_dims) * 0.45
    axes = [
        tuple(-dimension / 2.0 + cell / 2.0 + index * cell for index in range(count))
        for dimension, cell, count in zip(
            dims,
            cell_dims,
            subdivisions,
            strict=True,
        )
    ]
    bottom_sphere_center_object_z_m = (
        -dims[2] / 2.0 + radius + 0.01
    )
    top_sphere_center_object_z_m = dims[2] / 2.0 - radius
    vertical_count = subdivisions[2]
    if vertical_count == 1:
        axes[2] = (
            (bottom_sphere_center_object_z_m + top_sphere_center_object_z_m) / 2.0,
        )
    else:
        axes[2] = tuple(
            bottom_sphere_center_object_z_m
            + (
                top_sphere_center_object_z_m
                - bottom_sphere_center_object_z_m
            )
            * index
            / (vertical_count - 1)
            for index in range(vertical_count)
        )
    return tuple((*center, radius) for center in itertools.product(*axes))


def _curobo_pose(
    position_base_m: tuple[float, float, float],
    orientation_base_xyzw: tuple[float, float, float, float],
) -> list[float]:
    return [
        *position_base_m,
        orientation_base_xyzw[3],
        orientation_base_xyzw[0],
        orientation_base_xyzw[1],
        orientation_base_xyzw[2],
    ]


__all__ = ["CuroboMotionPlanner", "build_curobo_motion_planners"]
