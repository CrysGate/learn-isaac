"""Measure and apply task-space tabletop grasp constraints."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from pxr import Gf, Usd, UsdGeom

from grasp_data_gen.config import TabletopGraspFilterConfig
from grasp_data_gen.isaac.geometry import Pose
from grasp_data_gen.models import TaskFilterResult, Vector3

if TYPE_CHECKING:
    from grasp_data_gen.isaac.scene import EvaluationScene


FILTER_EPSILON = 1.0e-6
DIRECTION_EPSILON = 1.0e-9
BOUND_PURPOSES = (
    UsdGeom.Tokens.default_,
    UsdGeom.Tokens.render,
    UsdGeom.Tokens.proxy,
)


@dataclass(frozen=True)
class TabletopGeometry:
    """Scene measurements reused while filtering a batch of candidates."""

    gripper_points_base: tuple[Gf.Vec3d, ...]
    support_height_m: float
    camera_position_base: Gf.Vec3d | None


def _bbox_corners(bounds: Gf.Range3d) -> tuple[Gf.Vec3d, ...]:
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    return tuple(
        Gf.Vec3d(x, y, z)
        for x in (minimum[0], maximum[0])
        for y in (minimum[1], maximum[1])
        for z in (minimum[2], maximum[2])
    )


def _gripper_points_base(scene: EvaluationScene) -> tuple[Gf.Vec3d, ...]:
    bounds_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), list(BOUND_PURPOSES))
    points = []
    for link_name, link in scene.gripper.link_prims.items():
        bounds = bounds_cache.ComputeUntransformedBound(link).ComputeAlignedRange()
        if bounds.IsEmpty():
            raise RuntimeError(
                f"configured link has no bounded geometry: {link.GetPath()}"
            )
        position, orientation = scene.gripper.link_pose_base(link_name)
        rotation = Gf.Rotation(orientation)
        points.extend(
            position + rotation.TransformDir(point)
            for point in _bbox_corners(bounds)
        )
    return tuple(points)


def _support_height(
    object_prim: Usd.Prim,
    up_axis_object: Vector3,
) -> float:
    bounds = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        list(BOUND_PURPOSES),
    ).ComputeUntransformedBound(object_prim).ComputeAlignedRange()
    if bounds.IsEmpty():
        raise RuntimeError("object USD has no bounded geometry")
    up = Gf.Vec3d(*up_axis_object)
    return min(float(Gf.Dot(point, up)) for point in _bbox_corners(bounds))


def measure_tabletop_geometry(
    scene: EvaluationScene,
    config: TabletopGraspFilterConfig,
) -> TabletopGeometry:
    """Measure geometry that remains constant across one candidate batch."""

    camera_pose = scene.gripper.base_to_camera
    return TabletopGeometry(
        gripper_points_base=_gripper_points_base(scene),
        support_height_m=_support_height(scene.object_prim, config.up_axis_object),
        camera_position_base=None if camera_pose is None else camera_pose[0],
    )


def evaluate_tabletop_candidate(
    base_pose: Pose,
    tcp_pose: Pose,
    geometry: TabletopGeometry,
    config: TabletopGraspFilterConfig,
    approach_axis_tcp: Vector3,
) -> TaskFilterResult:
    """Evaluate one candidate against support-plane and camera constraints."""

    approach = Gf.Rotation(tcp_pose[1]).TransformDir(Gf.Vec3d(*approach_axis_tcp))
    up = Gf.Vec3d(*config.up_axis_object)
    approach_up_dot = float(Gf.Dot(approach, up))

    base_position, base_orientation = base_pose
    rotation = Gf.Rotation(base_orientation)
    camera_height_above_tcp = None
    camera_side_up_dot = None
    if geometry.camera_position_base is not None:
        camera_position = base_position + rotation.TransformDir(
            geometry.camera_position_base
        )
        camera_offset = camera_position - tcp_pose[0]
        camera_height_above_tcp = float(Gf.Dot(camera_offset, up))
        camera_side = camera_offset - approach * Gf.Dot(camera_offset, approach)
        camera_side_length = camera_side.GetLength()
        if camera_side_length > DIRECTION_EPSILON:
            camera_side_up_dot = float(
                Gf.Dot(camera_side / camera_side_length, up)
            )

    pregrasp_position = base_position - approach * config.approach_distance_m
    minimum_height = min(
        float(Gf.Dot(position + rotation.TransformDir(point), up))
        for position in (base_position, pregrasp_position)
        for point in geometry.gripper_points_base
    )
    clearance = minimum_height - geometry.support_height_m

    failures = []
    if config.enabled:
        if approach_up_dot > config.maximum_approach_up_dot + FILTER_EPSILON:
            failures.append("approach_from_below")
        if clearance < config.gripper_clearance_m:
            failures.append("support_plane_collision")
        if config.require_camera_above_tcp:
            if camera_height_above_tcp is None:
                raise RuntimeError("camera-above-TCP filtering requires a mounted camera pose")
            if (camera_height_above_tcp < config.minimum_camera_height_above_tcp_m - FILTER_EPSILON):
                failures.append("camera_below_tcp")
            if camera_side_up_dot is None:
                raise RuntimeError(
                    "mounted camera must have an offset perpendicular to the "
                    "approach axis"
                )
            if (camera_side_up_dot < config.minimum_camera_side_up_dot - FILTER_EPSILON):
                failures.append("camera_not_on_upper_side")

    return TaskFilterResult(
        accepted=not failures,
        failures=tuple(failures),
        approach_direction_object=tuple(float(value) for value in approach),
        approach_up_dot=approach_up_dot,
        minimum_gripper_clearance_m=clearance,
        camera_height_above_tcp_m=camera_height_above_tcp,
        camera_side_up_dot=camera_side_up_dot,
    )
