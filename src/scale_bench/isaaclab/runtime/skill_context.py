"""Read-only Isaac Lab geometry used by manipulation skill programs."""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET
from collections import Counter, deque
from collections.abc import Mapping
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from isaaclab.sensors import Camera
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from scale_bench.config.loader import load_config
from scale_bench.config.models.grasp import GraspCatalogConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.skills.context import (
    GraspCandidate,
    GraspState,
    JointState,
    RobotState,
    SceneObject,
    SceneSnapshot,
)
from scale_bench.skills.errors import SkillError
from scale_bench.skills.geometry import (
    compose_pose,
    conjugate_quaternion_xyzw,
    inverse_pose,
    multiply_quaternions_xyzw,
    normalize_quaternion_xyzw,
    quaternion_xyzw_from_rpy,
    relative_pose,
    rotate_vector_xyzw,
)
from scale_bench.skills.models import (
    Arm,
    Pose,
)
from scale_bench.tasks.common.rigid_object import RigidObjectTask

from .anygrasp import AnyGraspClient, AnyGraspDetection, AnyGraspServiceError
from .anygrasp_diagnostics import (
    AnyGraspCandidateStatus,
    AnyGraspDiagnostics,
    AnyGraspPoseDiagnostic,
)
from .environment import ScaleBenchEnv

LOGGER = logging.getLogger(__name__)

UPPER_HALF_GRASP_TASK_ID = "single_object_pick_and_place"
UPPER_HALF_GRASP_MIN_TCP_OBJECT_Z_M = 0.0


@dataclass(frozen=True, slots=True)
class _ValidAnyGraspCandidate:
    detection_index: int
    score: float
    width_m: float
    tcp_position_object_m: tuple[float, float, float]
    candidate: GraspCandidate


@dataclass(frozen=True, slots=True)
class _AnyGraspCapture:
    rgb: np.ndarray
    depth_m: np.ndarray
    intrinsic_matrix_px: np.ndarray
    camera_position_env_m: tuple[float, float, float]
    camera_orientation_env_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class _AnyGraspInference:
    capture: _AnyGraspCapture
    target_points_env_m: tuple[tuple[float, float, float], ...]
    detections: tuple[AnyGraspDetection, ...]


class IsaacLabSkillContext:
    """Resolve live geometry without advancing physics or episode recording."""

    def __init__(
        self,
        env: ScaleBenchEnv,
        task: RigidObjectTask,
        scene_config: SceneConfig,
        robot_configs: Mapping[Arm, RobotConfig],
        *,
        env_id: int,
    ) -> None:
        if env_id < 0 or env_id >= env.num_envs:
            raise ValueError(f"env_id must be in [0, {env.num_envs})")
        self._env = env
        self._task = task
        self._env_id = env_id
        self._table = SceneObject(
            "table",
            Pose(scene_config.table.position_m, (0.0, 0.0, 0.0, 1.0)),
            scene_config.table.size_m,
        )
        self._camera_stand = _camera_stand_collision_objects_env(scene_config)
        self._scene_table_top_z_m = scene_config.table_top_z_m
        self._tcp_body_indices = {}
        self._arm_joint_indices = {}
        self._gripper_joint_indices = {}
        self._gripper_aperture_multipliers = {}
        self._minimum_grasp_apertures_m = {}
        self._tcp_poses_ee_body = {}
        self._camera_positions_tcp_m = {}
        for arm in ("left", "right"):
            robot_config = robot_configs[arm]
            kinematics = robot_config.kinematics
            tcp = kinematics.tcp
            robot = env.scene[f"{arm}_robot"]
            body_indices, body_names = robot.find_bodies(kinematics.ee_body)
            if len(body_indices) != 1 or body_names != [kinematics.ee_body]:
                raise ValueError(
                    f"{arm} robot EE body {kinematics.ee_body!r} did not "
                    "resolve to exactly one articulation body"
                )
            tcp_parent_pose_ee_body = _fixed_urdf_frame_pose(
                robot_config.urdf_path,
                kinematics.ee_body,
                tcp.parent_frame,
            )
            self._tcp_body_indices[arm] = body_indices[0]
            joint_indices, joint_names = robot.find_joints(
                kinematics.arm_joint_names,
                preserve_order=True,
            )
            if tuple(joint_names) != kinematics.arm_joint_names:
                raise ValueError(f"{arm} robot arm joints do not match its profile")
            self._arm_joint_indices[arm] = joint_indices
            gripper = robot_config.gripper
            gripper_indices, gripper_names = robot.find_joints(
                gripper.joint_names,
                preserve_order=True,
            )
            if tuple(gripper_names) != gripper.joint_names:
                raise ValueError(f"{arm} robot gripper joints do not match its profile")
            self._gripper_joint_indices[arm] = gripper_indices
            self._gripper_aperture_multipliers[arm] = tuple(
                gripper.aperture_joint_multipliers[name] for name in gripper.joint_names
            )
            self._minimum_grasp_apertures_m[arm] = gripper.minimum_grasp_aperture_m
            self._tcp_poses_ee_body[arm] = compose_pose(
                tcp_parent_pose_ee_body,
                Pose(tcp.position_m, tcp.orientation_xyzw),
            )
            self._camera_positions_tcp_m[arm] = _camera_position_tcp_m(
                robot_config,
                inverse_pose(self._tcp_poses_ee_body[arm]),
            )
        self._arm_base_poses_env = {
            "left": Pose(
                (
                    *scene_config.robot_mounts.left.position_xy_m,
                    scene_config.table_top_z_m,
                ),
                scene_config.robot_mounts.left.orientation_xyzw,
            ),
            "right": Pose(
                (
                    *scene_config.robot_mounts.right.position_xy_m,
                    scene_config.table_top_z_m,
                ),
                scene_config.robot_mounts.right.orientation_xyzw,
            ),
        }
        self._anygrasp_config = scene_config.anygrasp
        self._anygrasp_client = (
            AnyGraspClient(scene_config.anygrasp)
            if scene_config.anygrasp is not None
            else None
        )
        self._gripper_apertures_m = {
            arm: robot_configs[arm].gripper.max_aperture_m for arm in ("left", "right")
        }
        self._grasp_catalogs = {}
        if self._anygrasp_client is None:
            self._grasp_catalogs = {
                arm: _load_grasp_catalog(robot_configs[arm])
                for arm in ("left", "right")
            }
            for arm, catalog in self._grasp_catalogs.items():
                missing = set(task.metadata) - set(catalog.objects)
                if missing:
                    raise ValueError(
                        f"{arm} grasp catalog is missing task objects: "
                        f"{sorted(missing)}"
                    )

    def snapshot(self) -> SceneSnapshot:
        """Read current robot, static-scene, and task-object geometry."""

        objects = tuple(
            SceneObject(
                object_name,
                Pose(*self._object_pose_env(object_name)),
                metadata.size,
            )
            for object_name, metadata in self._task.metadata.items()
        )
        return SceneSnapshot(
            left_robot=self._robot_state("left"),
            right_robot=self._robot_state("right"),
            table=self._table,
            camera_stand=self._camera_stand,
            objects=objects,
        )

    def grasp_candidates(
        self,
        object_name: str,
        arm: Arm,
    ) -> tuple[GraspCandidate, ...]:
        """Return score-ordered object-local TCP candidates for one arm."""

        if object_name not in self._task.metadata:
            raise ValueError(f"unknown task object: {object_name!r}")
        object_position_env_m, object_orientation_env_xyzw = self._object_pose_env(
            object_name
        )
        if self._anygrasp_client is not None:
            inference = self._infer_anygrasp(
                object_name,
                arm,
                object_position_env_m,
                object_orientation_env_xyzw,
            )
            diagnostics = self._analyze_anygrasp(
                object_name,
                arm,
                object_position_env_m,
                object_orientation_env_xyzw,
                inference,
            )
            candidates = diagnostics.candidates
            if not candidates:
                raise SkillError(
                    f"AnyGrasp returned no valid {object_name!r} candidate "
                    f"for {arm} arm "
                    f"(detections={len(inference.detections)}, "
                    f"target_points={len(inference.target_points_env_m)})"
                )
        else:
            catalog = self._grasp_catalogs[arm]
            approach_distance_m = catalog.approach_distance_m
            candidates = tuple(
                GraspCandidate(
                    tcp_pose_object=Pose(
                        candidate.position_object_m,
                        candidate.orientation_object_xyzw,
                    ),
                    approach_axis_tcp=candidate.approach_axis_tcp,
                    approach_distance_m=approach_distance_m,
                    score=candidate.score,
                )
                for candidate in catalog.objects[object_name]
            ) + self._procedural_side_grasps(
                arm,
                object_position_env_m,
                object_orientation_env_xyzw,
            )
        return tuple(sorted(candidates, key=lambda item: item.score, reverse=True))

    def analyze_anygrasp(
        self,
        object_name: str,
        arm: Arm,
    ) -> AnyGraspDiagnostics:
        """Return every raw pose and filter outcome for the diagnostic viewer."""

        if object_name not in self._task.metadata:
            raise ValueError(f"unknown task object: {object_name!r}")
        if self._anygrasp_client is None:
            raise ValueError("AnyGrasp diagnostics require an AnyGrasp scene source")
        object_position_env_m, object_orientation_env_xyzw = self._object_pose_env(
            object_name
        )
        inference = self._infer_anygrasp(
            object_name,
            arm,
            object_position_env_m,
            object_orientation_env_xyzw,
        )
        return self._analyze_anygrasp(
            object_name,
            arm,
            object_position_env_m,
            object_orientation_env_xyzw,
            inference,
        )

    def measure_grasp(self, object_name: str, arm: Arm) -> GraspState:
        """Measure the live object-to-TCP relation after gripper settling."""

        aperture_m = self._gripper_aperture_m(arm)
        minimum_aperture_m = self._minimum_grasp_apertures_m[arm]
        if aperture_m < minimum_aperture_m:
            raise SkillError(
                f"{arm} gripper does not hold {object_name!r}: "
                f"aperture={aperture_m:.6g} m, "
                f"minimum={minimum_aperture_m:.6g} m"
            )
        object_pose_env = Pose(*self._object_pose_env(object_name))
        tcp_pose_env = self._tcp_pose_env(arm)
        tcp_pose_object = relative_pose(object_pose_env, tcp_pose_env)
        return GraspState(
            object_name,
            arm,
            aperture_m,
            object_pose_env,
            tcp_pose_env,
            tcp_pose_object,
        )

    def _gripper_aperture_m(self, arm: Arm) -> float:
        robot = self._env.scene[f"{arm}_robot"]
        positions = robot.data.joint_pos.torch[
            self._env_id,
            self._gripper_joint_indices[arm],
        ]
        multipliers = positions.new_tensor(self._gripper_aperture_multipliers[arm])
        return float((positions * multipliers).sum().item())

    def _robot_state(self, arm: Arm) -> RobotState:
        robot = self._env.scene[f"{arm}_robot"]
        joints = (
            robot.data.joint_pos.torch[
                self._env_id,
                self._arm_joint_indices[arm],
            ]
            .detach()
            .clone()
        )
        return RobotState(
            JointState(joints),
            self._tcp_pose_env(arm),
            self._camera_positions_tcp_m[arm],
        )

    def _tcp_pose_env(self, arm: Arm) -> Pose:
        robot = self._env.scene[f"{arm}_robot"]
        body_index = self._tcp_body_indices[arm]
        env_origin = self._env.scene.env_origins[self._env_id]
        ee_body_position_env_m = (
            robot.data.body_pos_w.torch[self._env_id, body_index] - env_origin
        )
        ee_body_orientation_env_xyzw = robot.data.body_quat_w.torch[
            self._env_id, body_index
        ]
        ee_body_pose_env = Pose(
            tuple(ee_body_position_env_m.detach().cpu().tolist()),
            tuple(ee_body_orientation_env_xyzw.detach().cpu().tolist()),
        )
        return compose_pose(ee_body_pose_env, self._tcp_poses_ee_body[arm])

    def _object_pose_env(
        self,
        object_name: str,
    ) -> tuple[
        tuple[float, float, float],
        tuple[float, float, float, float],
    ]:
        object_position_env_m = (
            self._env.scene[object_name].data.root_pos_w.torch[self._env_id]
            - self._env.scene.env_origins[self._env_id]
        )
        object_orientation_env_xyzw = self._env.scene[
            object_name
        ].data.root_quat_w.torch[self._env_id]
        return (
            tuple(object_position_env_m.detach().cpu().tolist()),
            tuple(object_orientation_env_xyzw.detach().cpu().tolist()),
        )

    def _infer_anygrasp(
        self,
        object_name: str,
        arm: Arm,
        object_position_env_m: tuple[float, float, float],
        object_orientation_env_xyzw: tuple[float, float, float, float],
    ) -> _AnyGraspInference:
        """Capture the task-directed RGB-D view and submit one inference request."""

        config = self._anygrasp_config
        client = self._anygrasp_client
        assert config is not None and client is not None
        capture = self._capture_anygrasp_input(
            object_position_env_m,
            arm,
            config.capture_distance_m,
        )
        masked_depth, target_points_env_m = self._mask_anygrasp_target(
            object_name,
            object_position_env_m,
            object_orientation_env_xyzw,
            capture,
        )
        capture_view = "arm_side_oblique"
        oblique_target_point_count = len(target_points_env_m)
        if oblique_target_point_count < config.minimum_target_points:
            LOGGER.warning(
                "oblique view has %d target points; retrying from overhead",
                oblique_target_point_count,
                extra={
                    "event": "CAMERA",
                    "event_fields": {
                        "env_id": self._env_id,
                        "object": object_name,
                        "arm": arm,
                        "view": capture_view,
                        "target_point_count": oblique_target_point_count,
                        "minimum_target_point_count": (
                            config.minimum_target_points
                        ),
                        "fallback_view": "overhead",
                    },
                },
            )
            capture = self._capture_overhead_anygrasp_input(
                object_position_env_m,
                config.capture_distance_m,
            )
            masked_depth, target_points_env_m = self._mask_anygrasp_target(
                object_name,
                object_position_env_m,
                object_orientation_env_xyzw,
                capture,
            )
            capture_view = "overhead_fallback"
        valid_depth = capture.depth_m[
            np.isfinite(capture.depth_m) & (capture.depth_m > 0.0)
        ]
        depth_range_m = (
            "empty"
            if not len(valid_depth)
            else f"{float(valid_depth.min()):.4f}..{float(valid_depth.max()):.4f}"
        )
        object_position_env_m_rounded = tuple(
            round(value, 4) for value in object_position_env_m
        )
        camera_position_env_m_rounded = tuple(
            round(value, 4) for value in capture.camera_position_env_m
        )
        camera_orientation_env_xyzw_rounded = tuple(
            round(value, 4) for value in capture.camera_orientation_env_xyzw
        )
        center_depth_m = float(
            capture.depth_m[
                capture.depth_m.shape[0] // 2,
                capture.depth_m.shape[1] // 2,
            ]
        )
        LOGGER.debug(
            "view=%s object_position_env_m=%s camera_position_env_m=%s "
            "camera_orientation_env_xyzw=%s center_depth_m=%.4f valid_depth=%d "
            "depth_range_m=%s",
            capture_view,
            object_position_env_m_rounded,
            camera_position_env_m_rounded,
            camera_orientation_env_xyzw_rounded,
            center_depth_m,
            len(valid_depth),
            depth_range_m,
            extra={
                "event": "CAMERA",
                "event_fields": {
                    "env_id": self._env_id,
                    "object": object_name,
                    "arm": arm,
                    "view": capture_view,
                    "object_position_env_m": object_position_env_m_rounded,
                    "camera_position_env_m": camera_position_env_m_rounded,
                    "camera_orientation_env_xyzw": (
                        camera_orientation_env_xyzw_rounded
                    ),
                    "center_depth_m": center_depth_m,
                    "valid_depth_count": len(valid_depth),
                    "depth_range_m": depth_range_m,
                },
            },
        )
        target_point_count = len(target_points_env_m)
        if target_point_count < config.minimum_target_points:
            raise SkillError(
                f"AnyGrasp overhead fallback crop for {object_name!r} contains "
                f"only {target_point_count} valid depth points after the "
                f"arm-side view contained {oblique_target_point_count}; "
                f"expected at least {config.minimum_target_points}"
            )
        try:
            detections = client.detect(
                capture.rgb,
                capture.depth_m,
                capture.intrinsic_matrix_px,
                masked_depth > 0.0,
            )
        except AnyGraspServiceError as error:
            raise SkillError(str(error)) from error
        return _AnyGraspInference(capture, target_points_env_m, detections)

    def _analyze_anygrasp(
        self,
        object_name: str,
        arm: Arm,
        object_position_env_m: tuple[float, float, float],
        object_orientation_env_xyzw: tuple[float, float, float, float],
        inference: _AnyGraspInference,
    ) -> AnyGraspDiagnostics:
        """Transform and classify one shared detection batch for an arm."""

        config = self._anygrasp_config
        assert config is not None
        upper_half_task = self._task.task_id == UPPER_HALF_GRASP_TASK_ID
        capture = inference.capture
        detections = inference.detections
        target_points_env_m = inference.target_points_env_m
        object_pose_env = Pose(
            object_position_env_m,
            object_orientation_env_xyzw,
        )
        camera_pose_env = Pose(
            capture.camera_position_env_m,
            capture.camera_orientation_env_xyzw,
        )
        aperture_m = self._gripper_apertures_m[arm]
        diagnostics = []
        valid_candidates = []
        for detection_index, detection in enumerate(
            sorted(
                detections,
                key=lambda item: item.score,
                reverse=True,
            )
        ):
            grasp_orientation_camera_xyzw = _quaternion_xyzw_from_matrix(
                detection.rotation_camera
            )
            tcp_pose_camera = Pose(
                tuple(detection.translation_camera_m.tolist()),
                grasp_orientation_camera_xyzw,
            )
            tcp_pose_env = compose_pose(camera_pose_env, tcp_pose_camera)
            anygrasp_tip_pose_camera = Pose(
                tuple(detection.tip_position_camera_m.tolist()),
                grasp_orientation_camera_xyzw,
            )
            anygrasp_tip_pose_env = compose_pose(
                camera_pose_env,
                anygrasp_tip_pose_camera,
            )
            anygrasp_tip_pose_object = relative_pose(
                object_pose_env,
                anygrasp_tip_pose_env,
            )
            tcp_pose_object = relative_pose(object_pose_env, tcp_pose_env)
            approach_axis_env = rotate_vector_xyzw(
                tcp_pose_env.orientation_xyzw,
                (1.0, 0.0, 0.0),
            )
            finger_open_axis_env = rotate_vector_xyzw(
                tcp_pose_env.orientation_xyzw,
                (0.0, 1.0, 0.0),
            )
            candidate = GraspCandidate(
                tcp_pose_object=tcp_pose_object,
                approach_axis_tcp=(1.0, 0.0, 0.0),
                approach_distance_m=config.approach_distance_m,
                score=detection.score,
            )
            status = _anygrasp_candidate_status(
                score=detection.score,
                minimum_score=config.min_score,
                width_m=detection.width_m,
                aperture_m=aperture_m,
                open_axis_vertical_dot=abs(finger_open_axis_env[2]),
                maximum_open_axis_vertical_dot=(config.maximum_open_axis_vertical_dot),
                table_clearance_m=(
                    tcp_pose_env.position_m[2] - self._scene_table_top_z_m
                ),
                minimum_tcp_height_above_table_m=(
                    config.minimum_tcp_height_above_table_m
                ),
                tcp_inside_target_box=_point_inside_box(
                    tcp_pose_object.position_m,
                    self._task.metadata[object_name].size,
                    config.target_margin_m,
                ),
            )
            if (
                status.is_valid
                and upper_half_task
                and tcp_pose_object.position_m[2]
                < UPPER_HALF_GRASP_MIN_TCP_OBJECT_Z_M
            ):
                status = AnyGraspCandidateStatus.REJECTED_TCP_HEIGHT
            diagnostics.append(
                AnyGraspPoseDiagnostic(
                    detection_index=detection_index,
                    score=detection.score,
                    width_m=detection.width_m,
                    height_m=detection.height_m,
                    depth_m=detection.depth_m,
                    translation_camera_m=tuple(detection.translation_camera_m.tolist()),
                    rotation_camera=tuple(
                        tuple(float(value) for value in row)
                        for row in detection.rotation_camera
                    ),
                    object_id=detection.object_id,
                    grasp_origin_env_m=tcp_pose_env.position_m,
                    anygrasp_tip_pose_env=anygrasp_tip_pose_env,
                    anygrasp_tip_position_object_m=(
                        anygrasp_tip_pose_object.position_m
                    ),
                    tcp_pose_env=tcp_pose_env,
                    tcp_position_object_m=tcp_pose_object.position_m,
                    tcp_axes_env=tuple(
                        rotate_vector_xyzw(tcp_pose_env.orientation_xyzw, axis)
                        for axis in (
                            (1.0, 0.0, 0.0),
                            (0.0, 1.0, 0.0),
                            (0.0, 0.0, 1.0),
                        )
                    ),
                    approach_axis_env=approach_axis_env,
                    finger_open_axis_env=finger_open_axis_env,
                    open_axis_vertical_dot=abs(finger_open_axis_env[2]),
                    table_clearance_m=(
                        tcp_pose_env.position_m[2] - self._scene_table_top_z_m
                    ),
                    status=status,
                )
            )
            if status.is_valid:
                valid_candidates.append(
                    _ValidAnyGraspCandidate(
                        detection_index=detection_index,
                        score=detection.score,
                        width_m=detection.width_m,
                        tcp_position_object_m=tcp_pose_object.position_m,
                        candidate=candidate,
                    )
                )
        selected_candidates = (
            [max(valid_candidates, key=lambda item: item.score)]
            if valid_candidates
            else []
        )
        selected_detection_indices = {
            item.detection_index for item in selected_candidates
        }
        diagnostics = [
            replace(diagnostic, status=AnyGraspCandidateStatus.SELECTED)
            if diagnostic.detection_index in selected_detection_indices
            else diagnostic
            for diagnostic in diagnostics
        ]
        candidates = tuple(item.candidate for item in valid_candidates)
        accepted_diagnostics = [
            {
                "detection_index": item.detection_index,
                "score": round(item.score, 4),
                "width_m": round(item.width_m, 4),
                "tcp_position_object_m": tuple(
                    round(value, 4) for value in item.tcp_position_object_m
                ),
            }
            for item in valid_candidates
        ]
        LOGGER.debug(
            "geometry-valid candidates=%s",
            accepted_diagnostics,
            extra={
                "event": "CANDIDATE",
                "event_fields": {
                    "env_id": self._env_id,
                    "object": object_name,
                    "arm": arm,
                    "candidates": accepted_diagnostics,
                },
            },
        )
        status_counts = Counter(diagnostic.status.value for diagnostic in diagnostics)
        ordered_status_counts = dict(sorted(status_counts.items()))
        rejected_statuses = {
            status.removeprefix("rejected_"): count
            for status, count in ordered_status_counts.items()
            if status.startswith("rejected_")
        }
        filtered_summary = (
            ",".join(
                f"{status}:{count}"
                for status, count in rejected_statuses.items()
            )
            or "none"
        )
        LOGGER.info(
            "points=%d returned=%d valid=%d filtered=%s",
            len(target_points_env_m),
            len(detections),
            len(candidates),
            filtered_summary,
            extra={
                "event": "DETECT",
                "event_fields": {
                    "env_id": self._env_id,
                    "object": object_name,
                    "arm": arm,
                    "target_point_count": len(target_points_env_m),
                    "detection_count": len(detections),
                    "valid_candidate_count": len(candidates),
                    "status_counts": ordered_status_counts,
                },
            },
        )
        return AnyGraspDiagnostics(
            object_name=object_name,
            arm=arm,
            env_id=self._env_id,
            object_pose_env=Pose(
                object_position_env_m,
                object_orientation_env_xyzw,
            ),
            object_size_m=self._task.metadata[object_name].size,
            camera_pose_env=Pose(
                capture.camera_position_env_m,
                capture.camera_orientation_env_xyzw,
            ),
            input_rgb=capture.rgb,
            input_depth_m=capture.depth_m,
            intrinsic_matrix_px=capture.intrinsic_matrix_px,
            depth_trunc_m=config.depth_trunc_m,
            target_points_env_m=target_points_env_m,
            table_top_z_m=self._scene_table_top_z_m,
            gripper_aperture_m=aperture_m,
            minimum_score=config.min_score,
            maximum_open_axis_vertical_dot=(config.maximum_open_axis_vertical_dot),
            minimum_tcp_height_above_table_m=(config.minimum_tcp_height_above_table_m),
            detections=tuple(diagnostics),
            candidates=candidates,
        )

    def _capture_anygrasp_input(
        self,
        object_position_env_m: tuple[float, float, float],
        arm: Arm,
        capture_distance_m: float,
    ) -> _AnyGraspCapture:
        """Capture from the selected arm's side at a 45-degree elevation."""

        arm_base_position_env_m = self._arm_base_poses_env[arm].position_m
        horizontal_x = arm_base_position_env_m[0] - object_position_env_m[0]
        horizontal_y = arm_base_position_env_m[1] - object_position_env_m[1]
        horizontal_norm = math.hypot(horizontal_x, horizontal_y)
        if horizontal_norm <= 1.0e-9:
            raise SkillError(
                f"cannot build an arm-side AnyGrasp view for {arm}: "
                "object and robot base have the same XY position"
            )
        diagonal_offset_m = capture_distance_m / math.sqrt(2.0)
        eye_position_env_m = (
            object_position_env_m[0]
            + diagonal_offset_m * horizontal_x / horizontal_norm,
            object_position_env_m[1]
            + diagonal_offset_m * horizontal_y / horizontal_norm,
            object_position_env_m[2] + diagonal_offset_m,
        )
        return self._capture_anygrasp_view(
            object_position_env_m,
            eye_position_env_m,
        )

    def _capture_overhead_anygrasp_input(
        self,
        object_position_env_m: tuple[float, float, float],
        capture_distance_m: float,
    ) -> _AnyGraspCapture:
        """Capture the single fallback view directly above the target."""

        eye_position_env_m = (
            object_position_env_m[0],
            object_position_env_m[1],
            object_position_env_m[2] + capture_distance_m,
        )
        return self._capture_anygrasp_view(
            object_position_env_m,
            eye_position_env_m,
        )

    def _capture_anygrasp_view(
        self,
        object_position_env_m: tuple[float, float, float],
        eye_position_env_m: tuple[float, float, float],
    ) -> _AnyGraspCapture:
        """Render one target-centered view without stepping or recording."""

        camera = self._env.scene.sensors["overhead_camera"]
        if not isinstance(camera, Camera):
            raise SkillError("AnyGrasp requires an Isaac Lab Camera sensor")
        original_position_world = (
            camera.data.pos_w.torch[self._env_id].detach().cpu().numpy().copy()
        )
        original_orientation_ros = (
            camera.data.quat_w_ros.torch[self._env_id].detach().cpu().numpy().copy()
        )
        env_origin = self._env.scene.env_origins[self._env_id].detach().cpu().numpy()
        target_world = env_origin + np.asarray(object_position_env_m, dtype=np.float64)
        eye_world = env_origin + np.asarray(eye_position_env_m, dtype=np.float64)
        camera.set_world_poses_from_view(
            eyes=eye_world.reshape(1, 3),
            targets=target_world.reshape(1, 3),
            env_ids=[self._env_id],
        )
        try:
            self._refresh_camera_without_recording(camera)
            output = camera.data.output
            if output is None or not {
                "rgb",
                "distance_to_image_plane",
            }.issubset(output):
                raise SkillError(
                    "AnyGrasp requires overhead_camera aligned RGB-D output"
                )
            return _AnyGraspCapture(
                rgb=(output["rgb"].torch[self._env_id].detach().cpu().numpy().copy()),
                depth_m=(
                    output["distance_to_image_plane"]
                    .torch[self._env_id]
                    .detach()
                    .cpu()
                    .numpy()
                    .copy()
                ),
                intrinsic_matrix_px=(
                    camera.data.intrinsic_matrices.torch[self._env_id]
                    .detach()
                    .cpu()
                    .numpy()
                    .copy()
                ),
                camera_position_env_m=tuple(
                    float(value)
                    for value in (
                        camera.data.pos_w.torch[self._env_id].detach().cpu().numpy()
                        - env_origin
                    )
                ),
                camera_orientation_env_xyzw=tuple(
                    float(value)
                    for value in camera.data.quat_w_ros.torch[self._env_id]
                    .detach()
                    .cpu()
                    .tolist()
                ),
            )
        finally:
            camera.set_world_poses(
                positions=original_position_world.reshape(1, 3),
                orientations=original_orientation_ros.reshape(1, 4),
                env_ids=[self._env_id],
                convention="ros",
            )
            self._refresh_camera_without_recording(camera)

    def _mask_anygrasp_target(
        self,
        object_name: str,
        object_position_env_m: tuple[float, float, float],
        object_orientation_env_xyzw: tuple[float, float, float, float],
        capture: _AnyGraspCapture,
    ) -> tuple[np.ndarray, tuple[tuple[float, float, float], ...]]:
        """Extract visible target points from one captured scene frame."""

        config = self._anygrasp_config
        assert config is not None
        return _mask_depth_to_target_box(
            capture.depth_m,
            capture.intrinsic_matrix_px,
            capture.camera_position_env_m,
            capture.camera_orientation_env_xyzw,
            object_position_env_m,
            object_orientation_env_xyzw,
            self._task.metadata[object_name].size,
            config.target_margin_m,
            config.depth_trunc_m,
            minimum_env_z_m=(
                self._scene_table_top_z_m + config.minimum_point_height_above_table_m
            ),
        )

    def _refresh_camera_without_recording(self, camera: Camera) -> None:
        """Refresh one camera without environment or video recorder hooks."""

        import omni.kit.app

        self._env.sim.forward()
        # Pump Kit once so RTX observes the changed USD camera transform. This
        # bypasses SimulationContext.render(), whose final stage runs the
        # video-recording hooks.
        omni.kit.app.get_app().update()
        self._env.sim.render_context.reset_transform_cadence()
        camera.reset(env_ids=[self._env_id])
        camera.update(0.0, force_recompute=True)

    def _procedural_side_grasps(
        self,
        arm: Arm,
        object_position_env_m: tuple[float, float, float],
        object_orientation_env_xyzw: tuple[float, float, float, float],
    ) -> tuple[GraspCandidate, ...]:
        """Generate arm-reachable side grasps for full-arm CuRobo filtering."""

        arm_base_pose_env = self._arm_base_poses_env[arm]
        object_pose_env = Pose(
            object_position_env_m,
            object_orientation_env_xyzw,
        )
        object_pose_base = relative_pose(arm_base_pose_env, object_pose_env)
        radial_yaw_base_rad = math.atan2(
            object_pose_base.position_m[1],
            object_pose_base.position_m[0],
        )
        yaw_offsets_base_rad = (
            0.0,
            math.pi / 2.0,
            -math.pi / 2.0,
            math.pi,
            math.pi / 4.0,
            -math.pi / 4.0,
            3.0 * math.pi / 4.0,
            -3.0 * math.pi / 4.0,
        )
        env_orientation_object_xyzw = conjugate_quaternion_xyzw(
            object_orientation_env_xyzw
        )
        candidates = []
        for tilt_base_rad in (math.radians(30.0), math.radians(45.0)):
            for yaw_offset_base_rad in yaw_offsets_base_rad:
                yaw_base_rad = radial_yaw_base_rad + yaw_offset_base_rad
                tcp_orientation_base_xyzw = quaternion_xyzw_from_rpy(
                    0.0,
                    tilt_base_rad,
                    yaw_base_rad,
                )
                tcp_orientation_env_xyzw = multiply_quaternions_xyzw(
                    arm_base_pose_env.orientation_xyzw,
                    tcp_orientation_base_xyzw,
                )
                candidates.append(
                    GraspCandidate(
                        tcp_pose_object=Pose(
                            (0.0, 0.0, 0.0),
                            normalize_quaternion_xyzw(
                                multiply_quaternions_xyzw(
                                    env_orientation_object_xyzw,
                                    tcp_orientation_env_xyzw,
                                )
                            ),
                        ),
                        approach_axis_tcp=(1.0, 0.0, 0.0),
                        approach_distance_m=(
                            self._grasp_catalogs[arm].approach_distance_m
                        ),
                        score=0.0,
                    )
                )
        return tuple(candidates)


def _camera_position_tcp_m(
    robot_config: RobotConfig,
    ee_body_pose_tcp: Pose,
) -> tuple[float, float, float]:
    """Read the mounted sensor's fixed position for upright grasp filtering."""
    camera = robot_config.camera
    robot_stage = Usd.Stage.Open(robot_config.usd_path)
    robot_prim = robot_stage.GetDefaultPrim()
    camera_mount_prim = robot_stage.GetPrimAtPath(
        robot_prim.GetPath().AppendPath(camera.parent_prim_path)
    )
    ee_body_prim = robot_prim.GetChild(robot_config.kinematics.ee_body)
    camera_offset_tcp_m = rotate_vector_xyzw(
        ee_body_pose_tcp.orientation_xyzw,
        tuple(
            UsdGeom.XformCache().ComputeRelativeTransform(
                camera_mount_prim, ee_body_prim
            )[0].Transform(Gf.Vec3d(*camera.position_m))
        ),
    )
    return tuple(
        coordinate_tcp_m + offset_tcp_m
        for coordinate_tcp_m, offset_tcp_m in zip(
            ee_body_pose_tcp.position_m, camera_offset_tcp_m, strict=True
        )
    )


def _camera_stand_collision_objects_env(
    scene_config: SceneConfig,
) -> tuple[SceneObject, ...]:
    camera_stand_usd_path = scene_config.camera.stand_usd_path
    camera_stand_stage = Usd.Stage.Open(camera_stand_usd_path)
    if camera_stand_stage is None:
        raise ValueError(f"could not open camera stand USD: {camera_stand_usd_path}")

    camera_stand_prim = camera_stand_stage.GetDefaultPrim()
    if not camera_stand_prim.IsValid():
        raise ValueError(
            f"camera stand USD has no default prim: {camera_stand_usd_path}"
        )

    camera_stand_pose_env = Pose(
        (
            *scene_config.camera.stand_position_xy_m,
            scene_config.table_top_z_m,
        ),
        scene_config.camera.stand_orientation_xyzw,
    )
    bounds_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [
            UsdGeom.Tokens.default_,
            UsdGeom.Tokens.render,
            UsdGeom.Tokens.proxy,
        ],
    )
    collision_objects_env = []
    for collision_prim in camera_stand_stage.Traverse():
        if not collision_prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if (
            UsdPhysics.CollisionAPI(collision_prim)
            .GetCollisionEnabledAttr()
            .Get()
            is False
        ):
            continue

        collision_bounds_object_m = bounds_cache.ComputeRelativeBound(
            collision_prim,
            camera_stand_prim,
        ).ComputeAlignedRange()
        if collision_bounds_object_m.IsEmpty():
            continue
        minimum_object_m = collision_bounds_object_m.GetMin()
        maximum_object_m = collision_bounds_object_m.GetMax()
        collision_prim_position_object_m = tuple(
            float((minimum_object_m[axis] + maximum_object_m[axis]) / 2.0)
            for axis in range(3)
        )
        collision_prim_size_m = tuple(
            float(maximum_object_m[axis] - minimum_object_m[axis])
            for axis in range(3)
        )
        collision_prim_pose_env = compose_pose(
            camera_stand_pose_env,
            Pose(
                collision_prim_position_object_m,
                (0.0, 0.0, 0.0, 1.0),
            ),
        )
        collision_objects_env.append(
            SceneObject(
                name=f"camera_stand/{len(collision_objects_env):03d}",
                pose_env=collision_prim_pose_env,
                size_m=collision_prim_size_m,
            )
        )

    if not collision_objects_env:
        raise ValueError(
            "camera stand USD has no enabled collision geometry: "
            f"{camera_stand_usd_path}"
        )
    return tuple(collision_objects_env)


def _load_grasp_catalog(robot_config: RobotConfig) -> GraspCatalogConfig:
    if robot_config.grasp_catalog_path is None:
        raise ValueError(
            f"robot {robot_config.name!r} requires a grasp candidate catalog"
        )
    catalog = load_config(
        Path(robot_config.grasp_catalog_path),
        GraspCatalogConfig,
    )
    tcp = robot_config.kinematics.tcp
    if (
        catalog.robot_name != robot_config.name
        or catalog.tcp_parent_frame != tcp.parent_frame
        or catalog.tcp_position_m != tcp.position_m
        or catalog.tcp_orientation_xyzw != tcp.orientation_xyzw
    ):
        raise ValueError("grasp catalog TCP does not match RobotConfig")
    return catalog


def _matrix_from_quaternion_xyzw(
    value: tuple[float, float, float, float],
) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(value)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _quaternion_xyzw_from_matrix(
    matrix: np.ndarray,
) -> tuple[float, float, float, float]:
    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("rotation matrix must have shape (3, 3)")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = (
                math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            )
            quaternion = (
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            )
        elif index == 1:
            scale = (
                math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            )
            quaternion = (
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            )
        else:
            scale = (
                math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            )
            quaternion = (
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            )
    return normalize_quaternion_xyzw(tuple(float(value) for value in quaternion))


def _point_inside_box(
    point_object_m: tuple[float, float, float],
    size_m: tuple[float, float, float],
    margin_m: float,
) -> bool:
    return all(
        abs(coordinate) <= dimension / 2.0 + margin_m
        for coordinate, dimension in zip(point_object_m, size_m, strict=True)
    )


def _anygrasp_candidate_status(
    *,
    score: float,
    minimum_score: float,
    width_m: float,
    aperture_m: float,
    open_axis_vertical_dot: float,
    maximum_open_axis_vertical_dot: float,
    table_clearance_m: float,
    minimum_tcp_height_above_table_m: float,
    tcp_inside_target_box: bool,
) -> AnyGraspCandidateStatus:
    """Return the first failed filter in the runtime's fixed filter order."""

    if score < minimum_score:
        return AnyGraspCandidateStatus.REJECTED_SCORE
    if width_m > aperture_m + 1.0e-6:
        return AnyGraspCandidateStatus.REJECTED_WIDTH
    if not tcp_inside_target_box:
        return AnyGraspCandidateStatus.REJECTED_TARGET_BOX
    if open_axis_vertical_dot > maximum_open_axis_vertical_dot:
        return AnyGraspCandidateStatus.REJECTED_OPEN_AXIS
    if table_clearance_m < minimum_tcp_height_above_table_m:
        return AnyGraspCandidateStatus.REJECTED_TABLE_CLEARANCE
    return AnyGraspCandidateStatus.VALID_NOT_SELECTED


def _fixed_urdf_frame_pose(
    urdf_path: str | None,
    source_frame: str,
    target_frame: str,
) -> Pose:
    """Resolve a fixed-frame transform without depending on merged USD links."""

    identity_pose = Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    if source_frame == target_frame:
        return identity_pose
    if urdf_path is None:
        raise ValueError(
            f"cannot resolve {source_frame!r} to {target_frame!r} without a URDF"
        )
    try:
        root = ET.parse(Path(urdf_path)).getroot()
    except (OSError, ET.ParseError) as error:
        raise ValueError(f"could not parse robot URDF {urdf_path}: {error}") from error

    graph: dict[str, list[tuple[str, Pose]]] = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        if parent_node is None or child_node is None:
            raise ValueError("URDF fixed joint is missing parent or child")
        parent = parent_node.get("link")
        child = child_node.get("link")
        if not parent or not child:
            raise ValueError("URDF fixed joint has an empty parent or child")
        origin = joint.find("origin")
        child_position_parent_m = _urdf_vector(origin, "xyz")
        rpy = _urdf_vector(origin, "rpy")
        child_pose_parent = Pose(
            child_position_parent_m,
            quaternion_xyzw_from_rpy(*rpy),
        )
        graph.setdefault(parent, []).append((child, child_pose_parent))
        graph.setdefault(child, []).append((parent, inverse_pose(child_pose_parent)))

    pending = deque([(source_frame, identity_pose)])
    visited = {source_frame}
    while pending:
        frame, frame_pose_source = pending.popleft()
        for neighbor, neighbor_pose_frame in graph.get(frame, ()):
            if neighbor in visited:
                continue
            neighbor_pose_source = compose_pose(frame_pose_source, neighbor_pose_frame)
            if neighbor == target_frame:
                return neighbor_pose_source
            visited.add(neighbor)
            pending.append((neighbor, neighbor_pose_source))
    raise ValueError(
        f"URDF has no fixed-frame path from {source_frame!r} to {target_frame!r}"
    )


def _urdf_vector(
    origin: ET.Element | None,
    attribute: str,
) -> tuple[float, float, float]:
    text = None if origin is None else origin.get(attribute)
    values = (0.0, 0.0, 0.0) if text is None else tuple(map(float, text.split()))
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"URDF origin {attribute} must contain three finite values")
    return values


def _mask_depth_to_target_box(
    depth_m: np.ndarray,
    intrinsic_matrix_px: np.ndarray,
    camera_position_env_m: tuple[float, float, float],
    camera_orientation_env_xyzw: tuple[float, float, float, float],
    object_position_env_m: tuple[float, float, float],
    object_orientation_env_xyzw: tuple[float, float, float, float],
    object_size_m: tuple[float, float, float],
    margin_m: float,
    depth_trunc_m: float,
    *,
    minimum_env_z_m: float,
) -> tuple[
    np.ndarray,
    tuple[tuple[float, float, float], ...],
]:
    """Keep depth points inside the target's expanded oriented bounding box."""

    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    if depth.ndim != 2:
        raise SkillError(f"overhead depth has unexpected shape {depth.shape}")
    intrinsics = np.asarray(intrinsic_matrix_px, dtype=np.float64)
    if intrinsics.shape != (3, 3):
        raise SkillError(
            f"overhead camera intrinsics have unexpected shape {intrinsics.shape}"
        )
    fx, fy = float(intrinsics[0, 0]), float(intrinsics[1, 1])
    cx, cy = float(intrinsics[0, 2]), float(intrinsics[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise SkillError("overhead camera focal lengths must be positive")

    valid = np.isfinite(depth) & (depth > 0.0) & (depth < depth_trunc_m)
    rows, columns = np.nonzero(valid)
    masked = np.zeros_like(depth, dtype=np.float32)
    if not len(rows):
        return masked, ()
    z = depth[rows, columns].astype(np.float64, copy=False)
    points_camera = np.column_stack(
        (
            (columns.astype(np.float64) - cx) / fx * z,
            (rows.astype(np.float64) - cy) / fy * z,
            z,
        )
    )
    camera_to_env = _matrix_from_quaternion_xyzw(camera_orientation_env_xyzw)
    object_to_env = _matrix_from_quaternion_xyzw(object_orientation_env_xyzw)
    points_env = points_camera @ camera_to_env.T + np.asarray(
        camera_position_env_m,
        dtype=np.float64,
    )
    # Row-vector form of R_object_env.T @ (point_env - object_position_env).
    points_object = (
        points_env - np.asarray(object_position_env_m, dtype=np.float64)
    ) @ object_to_env
    half_extents = np.asarray(object_size_m, dtype=np.float64) / 2.0 + margin_m
    inside = np.all(np.abs(points_object) <= half_extents, axis=1) & (
        points_env[:, 2] >= minimum_env_z_m
    )
    masked[rows[inside], columns[inside]] = z[inside]
    target_points_env_m = tuple(
        tuple(float(value) for value in point) for point in points_env[inside]
    )
    return masked, target_points_env_m


__all__ = ["IsaacLabSkillContext"]
