"""Explicit AnyGrasp filtering records and their Isaac Kit viewer."""

from __future__ import annotations

import json
from collections.abc import Sequence
from dataclasses import asdict, dataclass
from enum import StrEnum
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from scale_bench.skills.context import GraspCandidate
from scale_bench.skills.models import Arm, Pose

if TYPE_CHECKING:
    import open3d as o3d


class AnyGraspCandidateStatus(StrEnum):
    """The first concrete filter outcome for one returned detection."""

    SELECTED = "selected"
    VALID_NOT_SELECTED = "valid_not_selected"
    REJECTED_SCORE = "rejected_score"
    REJECTED_WIDTH = "rejected_width"
    REJECTED_OPEN_AXIS = "rejected_open_axis"
    REJECTED_TCP_HEIGHT = "rejected_tcp_height"
    REJECTED_TABLE_CLEARANCE = "rejected_table_clearance"
    REJECTED_TARGET_BOX = "rejected_target_box"

    @property
    def is_valid(self) -> bool:
        return self in {
            self.SELECTED,
            self.VALID_NOT_SELECTED,
        }


@dataclass(frozen=True, slots=True)
class AnyGraspPoseDiagnostic:
    """One raw service pose after camera-to-environment transformation."""

    detection_index: int
    score: float
    width_m: float
    height_m: float
    depth_m: float
    translation_camera_m: tuple[float, float, float]
    rotation_camera: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    object_id: int
    grasp_origin_env_m: tuple[float, float, float]
    anygrasp_tip_pose_env: Pose
    anygrasp_tip_position_object_m: tuple[float, float, float]
    tcp_pose_env: Pose
    tcp_position_object_m: tuple[float, float, float]
    tcp_axes_env: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    approach_axis_env: tuple[float, float, float]
    finger_open_axis_env: tuple[float, float, float]
    open_axis_vertical_dot: float
    table_clearance_m: float
    status: AnyGraspCandidateStatus


@dataclass(frozen=True, slots=True)
class AnyGraspDiagnostics:
    """All evidence used to select candidates for one object and arm."""

    object_name: str
    arm: Arm
    env_id: int
    object_pose_env: Pose
    object_size_m: tuple[float, float, float]
    camera_pose_env: Pose
    input_rgb: np.ndarray
    input_depth_m: np.ndarray
    intrinsic_matrix_px: np.ndarray
    depth_trunc_m: float
    target_points_env_m: tuple[tuple[float, float, float], ...]
    table_top_z_m: float
    gripper_aperture_m: float
    minimum_score: float
    maximum_open_axis_vertical_dot: float
    minimum_tcp_height_above_table_m: float
    detections: tuple[AnyGraspPoseDiagnostic, ...]
    candidates: tuple[GraspCandidate, ...]

    def write_json(self, path: Path) -> None:
        """Write a portable record when a persistent diagnostic was requested."""

        document = {
            "object_name": self.object_name,
            "arm": self.arm,
            "env_id": self.env_id,
            "object_pose_env": asdict(self.object_pose_env),
            "object_size_m": self.object_size_m,
            "camera_pose_env": asdict(self.camera_pose_env),
            "target_points_env_m": self.target_points_env_m,
            "table_top_z_m": self.table_top_z_m,
            "gripper_aperture_m": self.gripper_aperture_m,
            "minimum_score": self.minimum_score,
            "maximum_open_axis_vertical_dot": (self.maximum_open_axis_vertical_dot),
            "minimum_tcp_height_above_table_m": (self.minimum_tcp_height_above_table_m),
            "detections": [
                {
                    **asdict(detection),
                    "status": detection.status.value,
                }
                for detection in self.detections
            ],
        }
        path.write_text(
            json.dumps(document, indent=2) + "\n",
            encoding="utf-8",
        )


@dataclass(frozen=True, slots=True)
class AnyGraspOpen3DGrasp:
    """Raw GraspNet fields required by the official Open3D geometry."""

    score: float
    width_m: float
    height_m: float
    depth_m: float
    translation_camera_m: tuple[float, float, float]
    rotation_camera: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    object_id: int
    status: AnyGraspCandidateStatus
    open_axis_vertical_dot: float


@dataclass(frozen=True, slots=True)
class AnyGraspOpen3DFrame:
    """Portable raw RGB-D inference result for a clean viewer process."""

    input_rgb: np.ndarray
    input_depth_m: np.ndarray
    intrinsic_matrix_px: np.ndarray
    depth_trunc_m: float
    grasps: tuple[AnyGraspOpen3DGrasp, ...]

    @classmethod
    def from_diagnostics(
        cls,
        diagnostics: AnyGraspDiagnostics,
    ) -> AnyGraspOpen3DFrame:
        return cls(
            input_rgb=diagnostics.input_rgb,
            input_depth_m=diagnostics.input_depth_m,
            intrinsic_matrix_px=diagnostics.intrinsic_matrix_px,
            depth_trunc_m=diagnostics.depth_trunc_m,
            grasps=tuple(
                AnyGraspOpen3DGrasp(
                    score=detection.score,
                    width_m=detection.width_m,
                    height_m=detection.height_m,
                    depth_m=detection.depth_m,
                    translation_camera_m=detection.translation_camera_m,
                    rotation_camera=detection.rotation_camera,
                    object_id=detection.object_id,
                    status=detection.status,
                    open_axis_vertical_dot=detection.open_axis_vertical_dot,
                )
                for detection in diagnostics.detections
            ),
        )

    def write(self, path: Path) -> None:
        np.savez_compressed(
            path,
            input_rgb=self.input_rgb,
            input_depth_m=self.input_depth_m,
            intrinsic_matrix_px=self.intrinsic_matrix_px,
            depth_trunc_m=np.asarray(self.depth_trunc_m, dtype=np.float64),
            scores=np.asarray([grasp.score for grasp in self.grasps]),
            widths_m=np.asarray([grasp.width_m for grasp in self.grasps]),
            heights_m=np.asarray([grasp.height_m for grasp in self.grasps]),
            grasp_depths_m=np.asarray([grasp.depth_m for grasp in self.grasps]),
            translations_camera_m=np.asarray(
                [grasp.translation_camera_m for grasp in self.grasps]
            ),
            rotations_camera=np.asarray(
                [grasp.rotation_camera for grasp in self.grasps]
            ),
            object_ids=np.asarray(
                [grasp.object_id for grasp in self.grasps],
                dtype=np.int64,
            ),
            statuses=np.asarray([grasp.status for grasp in self.grasps]),
            open_axis_vertical_dots=np.asarray(
                [grasp.open_axis_vertical_dot for grasp in self.grasps]
            ),
        )

    @classmethod
    def read(cls, path: Path) -> AnyGraspOpen3DFrame:
        with np.load(path, allow_pickle=False) as archive:
            scores = archive["scores"]
            widths_m = archive["widths_m"]
            heights_m = archive["heights_m"]
            grasp_depths_m = archive["grasp_depths_m"]
            translations = archive["translations_camera_m"]
            rotations = archive["rotations_camera"]
            object_ids = archive["object_ids"]
            statuses = archive["statuses"]
            open_axis_vertical_dots = archive["open_axis_vertical_dots"]
            grasp_count = len(scores)
            if not all(
                len(values) == grasp_count
                for values in (
                    widths_m,
                    heights_m,
                    grasp_depths_m,
                    translations,
                    rotations,
                    object_ids,
                    statuses,
                    open_axis_vertical_dots,
                )
            ):
                raise ValueError("Open3D bundle grasp arrays have different lengths")
            return cls(
                input_rgb=archive["input_rgb"],
                input_depth_m=archive["input_depth_m"],
                intrinsic_matrix_px=archive["intrinsic_matrix_px"],
                depth_trunc_m=float(archive["depth_trunc_m"]),
                grasps=tuple(
                    AnyGraspOpen3DGrasp(
                        score=float(scores[index]),
                        width_m=float(widths_m[index]),
                        height_m=float(heights_m[index]),
                        depth_m=float(grasp_depths_m[index]),
                        translation_camera_m=tuple(
                            float(value) for value in translations[index]
                        ),
                        rotation_camera=tuple(
                            tuple(float(value) for value in row)
                            for row in rotations[index]
                        ),
                        object_id=int(object_ids[index]),
                        status=AnyGraspCandidateStatus(str(statuses[index])),
                        open_axis_vertical_dot=float(open_axis_vertical_dots[index]),
                    )
                    for index in range(grasp_count)
                ),
            )


def _open3d_input_cloud(
    frame: AnyGraspOpen3DFrame,
) -> o3d.geometry.PointCloud:
    """Reconstruct the exact metric color cloud used by AnyGrasp."""

    import open3d as o3d

    depth = _request_depth_image(frame)
    colors = np.asarray(frame.input_rgb)
    if colors.shape != (*depth.shape, 3) or colors.dtype != np.uint8:
        raise ValueError("diagnostic RGB-D frame is not aligned uint8 RGB-D")
    intrinsics = np.asarray(
        frame.intrinsic_matrix_px,
        dtype=np.float64,
    )
    fx = float(intrinsics[0, 0])
    fy = float(intrinsics[1, 1])
    cx = float(intrinsics[0, 2])
    cy = float(intrinsics[1, 2])
    columns, rows = np.meshgrid(
        np.arange(depth.shape[1], dtype=np.float32),
        np.arange(depth.shape[0], dtype=np.float32),
    )
    valid = depth > 0.0
    z = depth[valid]
    points = np.column_stack(
        (
            (columns[valid] - cx) / fx * z,
            (rows[valid] - cy) / fy * z,
            z,
        )
    ).astype(np.float32)
    point_colors = colors[valid].astype(np.float32) / 255.0
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(point_colors)
    return cloud


def _request_depth_image(frame: AnyGraspOpen3DFrame) -> np.ndarray:
    """Return the metric depth array serialized in the AnyGrasp request."""

    depth = np.asarray(frame.input_depth_m, dtype=np.float32)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    if depth.ndim != 2:
        raise ValueError("diagnostic depth frame must have shape (H, W) or (H, W, 1)")
    valid = np.isfinite(depth) & (depth > 0.0) & (depth < frame.depth_trunc_m)
    return np.where(valid, depth, 0.0).astype(np.float32, copy=False)


def _depth_preview_rgb(depth_m: np.ndarray) -> tuple[np.ndarray, float, float]:
    """Colorize valid metric depth from red (near) to blue (far)."""

    valid = depth_m > 0.0
    if not valid.any():
        raise ValueError("AnyGrasp request depth contains no valid pixels")
    minimum_m = float(depth_m[valid].min())
    maximum_m = float(depth_m[valid].max())
    normalized = np.zeros_like(depth_m, dtype=np.float32)
    if maximum_m > minimum_m:
        normalized[valid] = (depth_m[valid] - minimum_m) / (maximum_m - minimum_m)

    preview = np.zeros((*depth_m.shape, 3), dtype=np.uint8)
    distance_from_middle = np.abs(2.0 * normalized - 1.0)
    preview[..., 0][valid] = np.asarray(
        255.0 * (1.0 - normalized[valid]),
        dtype=np.uint8,
    )
    preview[..., 1][valid] = np.asarray(
        255.0 * (1.0 - distance_from_middle[valid]),
        dtype=np.uint8,
    )
    preview[..., 2][valid] = np.asarray(
        255.0 * normalized[valid],
        dtype=np.uint8,
    )
    return preview, minimum_m, maximum_m


def _show_request_rgbd(frame: AnyGraspOpen3DFrame) -> None:
    """Show the two image arrays sent to AnyGrasp in one blocking window."""

    import open3d as o3d
    from open3d.visualization import gui

    colors = np.asarray(frame.input_rgb)
    depth_m = _request_depth_image(frame)
    if colors.shape != (*depth_m.shape, 3) or colors.dtype != np.uint8:
        raise ValueError("diagnostic RGB-D frame is not aligned uint8 RGB-D")
    depth_preview, minimum_m, maximum_m = _depth_preview_rgb(depth_m)

    application = gui.Application.instance
    application.initialize()
    window = application.create_window(
        "AnyGrasp request RGB-D",
        width=1600,
        height=900,
    )
    em = window.theme.font_size
    root = gui.Vert(0.5 * em, gui.Margins(em, em, em, em))
    root.add_child(
        gui.Label(
            "Exact client request images. Depth is in metres; black pixels "
            "are serialized as 0."
        )
    )
    images = gui.Horiz(0.5 * em)
    rgb_column = gui.Vert(0.25 * em)
    rgb_column.add_child(gui.Label(f"RGB uint8  shape={colors.shape}"))
    rgb_column.add_child(gui.ImageWidget(o3d.geometry.Image(colors)))
    depth_column = gui.Vert(0.25 * em)
    depth_column.add_child(
        gui.Label(
            f"Depth float32  valid range={minimum_m:.4f}..{maximum_m:.4f} m "
            "(near red, far blue)"
        )
    )
    depth_column.add_child(gui.ImageWidget(o3d.geometry.Image(depth_preview)))
    images.add_child(rgb_column)
    images.add_child(depth_column)
    root.add_child(images)
    window.add_child(root)
    application.run()


def _official_mesh_box(
    width: float,
    height: float,
    depth: float,
) -> o3d.geometry.TriangleMesh:
    """Build the box primitive used by graspnetAPI's official viewer."""

    import open3d as o3d

    vertices = np.asarray(
        (
            (0.0, 0.0, 0.0),
            (width, 0.0, 0.0),
            (0.0, 0.0, depth),
            (width, 0.0, depth),
            (0.0, height, 0.0),
            (width, height, 0.0),
            (0.0, height, depth),
            (width, height, depth),
        ),
        dtype=np.float64,
    )
    triangles = np.asarray(
        (
            (4, 7, 5),
            (4, 6, 7),
            (0, 2, 4),
            (2, 6, 4),
            (0, 1, 2),
            (1, 3, 2),
            (1, 5, 7),
            (1, 7, 3),
            (2, 3, 7),
            (2, 7, 6),
            (0, 4, 1),
            (1, 4, 5),
        ),
        dtype=np.int32,
    )
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    return mesh


def _official_gripper_mesh(
    grasp: AnyGraspOpen3DGrasp,
) -> o3d.geometry.TriangleMesh:
    """Reproduce graspnetAPI.plot_gripper_pro_max for one raw grasp."""

    import open3d as o3d

    mesh_height = 0.004
    finger_width = 0.004
    tail_length = 0.04
    depth_base = 0.02
    left = _official_mesh_box(
        grasp.depth_m + depth_base + finger_width,
        finger_width,
        mesh_height,
    )
    right = _official_mesh_box(
        grasp.depth_m + depth_base + finger_width,
        finger_width,
        mesh_height,
    )
    bottom = _official_mesh_box(
        finger_width,
        grasp.width_m,
        mesh_height,
    )
    tail = _official_mesh_box(tail_length, finger_width, mesh_height)

    left_points = np.asarray(left.vertices)
    right_points = np.asarray(right.vertices)
    bottom_points = np.asarray(bottom.vertices)
    tail_points = np.asarray(tail.vertices)
    left_points[:, 0] -= depth_base + finger_width
    left_points[:, 1] -= grasp.width_m / 2.0 + finger_width
    left_points[:, 2] -= mesh_height / 2.0
    right_points[:, 0] -= depth_base + finger_width
    right_points[:, 1] += grasp.width_m / 2.0
    right_points[:, 2] -= mesh_height / 2.0
    bottom_points[:, 0] -= finger_width + depth_base
    bottom_points[:, 1] -= grasp.width_m / 2.0
    bottom_points[:, 2] -= mesh_height / 2.0
    tail_points[:, 0] -= tail_length + finger_width + depth_base
    tail_points[:, 1] -= finger_width / 2.0
    tail_points[:, 2] -= mesh_height / 2.0

    vertices = np.concatenate(
        (left_points, right_points, bottom_points, tail_points),
        axis=0,
    )
    rotation = np.asarray(grasp.rotation_camera, dtype=np.float64)
    translation = np.asarray(
        grasp.translation_camera_m,
        dtype=np.float64,
    )
    vertices = vertices @ rotation.T + translation
    triangles = np.concatenate(
        (
            np.asarray(left.triangles),
            np.asarray(right.triangles) + 8,
            np.asarray(bottom.triangles) + 16,
            np.asarray(tail.triangles) + 24,
        ),
        axis=0,
    )
    vertex_color = (
        grasp.score,
        0.0,
        1.0 - grasp.score,
    )
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    mesh.vertex_colors = o3d.utility.Vector3dVector(
        np.tile(vertex_color, (len(vertices), 1))
    )
    return mesh


def _show_open3d_window(
    geometries: Sequence[o3d.geometry.Geometry],
    window_name: str,
) -> None:
    """Show point-cloud and triangle-mesh geometry in one blocking window."""

    import open3d as o3d

    visualizer = o3d.visualization.Visualizer()
    if not visualizer.create_window(
        window_name=window_name,
        width=1280,
        height=720,
    ):
        raise RuntimeError(
            "Open3D could not create an OpenGL window; verify DISPLAY and "
            "X11/OpenGL availability"
        )
    try:
        for geometry in geometries:
            visualizer.add_geometry(geometry)
        visualizer.run()
    finally:
        visualizer.destroy_window()


def show_anygrasp_open3d(frame: AnyGraspOpen3DFrame) -> None:
    """Show the raw camera-frame result using AnyGrasp's official style."""

    print(
        "[anygrasp-open3d] showing the exact request RGB-D images; close the "
        "window to inspect the returned grasps",
        flush=True,
    )
    _show_request_rgbd(frame)
    if not frame.grasps:
        print("[anygrasp-open3d] service returned no grasps", flush=True)
        return
    cloud = _open3d_input_cloud(frame)
    grippers = [_official_gripper_mesh(grasp) for grasp in frame.grasps]
    view_transform = np.diag((1.0, 1.0, -1.0, 1.0))
    cloud.transform(view_transform)
    for gripper in grippers:
        gripper.transform(view_transform)
    print(
        "[anygrasp-open3d] showing all raw grasps; close the window "
        "to inspect the top raw grasp",
        flush=True,
    )
    _show_open3d_window(
        [*grippers, cloud],
        window_name="AnyGrasp raw detections",
    )
    raw_top = frame.grasps[0]
    print(
        f"[anygrasp-open3d] raw top score={raw_top.score:.6f} "
        f"status={raw_top.status} "
        f"open_axis_vertical_dot={raw_top.open_axis_vertical_dot:.6f}",
        flush=True,
    )
    _show_open3d_window(
        [grippers[0], cloud],
        window_name="AnyGrasp raw top detection",
    )
    for index, grasp in enumerate(frame.grasps):
        if grasp.status is not AnyGraspCandidateStatus.SELECTED:
            continue
        print(
            f"[anygrasp-open3d] selected top score={grasp.score:.6f} "
            f"open_axis_vertical_dot={grasp.open_axis_vertical_dot:.6f}",
            flush=True,
        )
        _show_open3d_window(
            [grippers[index], cloud],
            window_name="AnyGrasp selected top detection",
        )
        break
    else:
        print("[anygrasp-open3d] no locally selected grasp", flush=True)


__all__ = [
    "AnyGraspCandidateStatus",
    "AnyGraspDiagnostics",
    "AnyGraspOpen3DFrame",
    "AnyGraspOpen3DGrasp",
    "AnyGraspPoseDiagnostic",
    "show_anygrasp_open3d",
]
