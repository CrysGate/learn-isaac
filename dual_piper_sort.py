#!/usr/bin/env python3
"""Dual-Piper matryoshka sorting simulation and expert-data collection.

The module deliberately keeps imports that require a running Isaac Sim
application inside the integration entry points.  Pure geometry, metadata,
layout, and schema helpers therefore remain cheap to import from
``test_dual_piper_sort.py``.

World convention:

* lengths are metres;
* the world is Z-up;
* quaternions are always ``[qw, qx, qy, qz]``.
"""

from __future__ import annotations

import argparse
import datetime
import importlib.metadata
import json
import math
import os
import platform
import re
import selectors
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Final, Sequence


REPOSITORY_ROOT: Final = Path(__file__).resolve().parent
ASSET_ROOT: Final = REPOSITORY_ROOT / "Assets"

PIPER_USD: Final = ASSET_ROOT / "Robots/piper/Piper.usd"
PIPER_URDF: Final = ASSET_ROOT / "Robots/piper/piper_description/urdf/piper.urdf"
PIPER_REFERENCE_CONFIG: Final = ASSET_ROOT / "Robots/piper/robot_config.yml"
ROOM_USD: Final = ASSET_ROOT / "Room/Simple_Room_nolight/simple_room_nolight.usd"
HDR_TEXTURE: Final = ASSET_ROOT / "Background/brown_photostudio_02_4k.hdr"
CAMERA_STAND_USD: Final = (
    ASSET_ROOT / "Object/RoboDojo/Geometry/camera_stand/00000/object.usd"
)
MATRYOSHKA_ROOT: Final = ASSET_ROOT / "Object/RoboDojo/Rigid/matryoshka_dolls"
TABLE_MDL: Final = ASSET_ROOT / "Material/material_0122/Mahogany_Planks.mdl"
GROUND_MDL: Final = ASSET_ROOT / "Material/material_0564/Wood_Tiles_Fineline.mdl"

WORLD_METERS_PER_UNIT: Final = 1.0
WORLD_UP_AXIS: Final = "Z"
QUATERNION_ORDER: Final = "wxyz"
IDENTITY_QUATERNION: Final = (1.0, 0.0, 0.0, 0.0)

TABLE_PRIM_PATH: Final = "/World/Scene/Table"
TABLE_POSITION: Final = (0.0, -0.05, 0.74)
TABLE_ORIENTATION: Final = IDENTITY_QUATERNION
TABLE_SIZE: Final = (1.4, 1.1, 0.05)
TABLE_X_RANGE: Final = (-0.70, 0.70)
TABLE_Y_RANGE: Final = (-0.60, 0.50)
TABLE_TOP_Z: Final = 0.765
TABLE_MATERIAL_ENTRY: Final = "Mahogany_Planks"

GROUND_PRIM_PATH: Final = "/World/Scene/Ground"
GROUND_POSITION: Final = (0.0, 0.0, -0.05)
GROUND_ORIENTATION: Final = IDENTITY_QUATERNION
GROUND_SIZE: Final = (6.0, 6.0, 0.1)
GROUND_MATERIAL_ENTRY: Final = "Wood_Tiles_Fineline"
TABLE_VISUAL_MATERIAL_PATH: Final = "/World/Looks/TableMahogany"
GROUND_VISUAL_MATERIAL_PATH: Final = "/World/Looks/GroundWoodTiles"

ROOM_PRIM_PATH: Final = "/World/Scene/Room"
ROOM_RESIDUAL_LIGHT_REL_PATH: Final = "simple_room/RectLight"
HDR_DOME_PRIM_PATH: Final = "/World/Scene/EnvironmentLight"
HDR_DOME_INTENSITY: Final = 1_000.0
HDR_DOME_ROTATION_DEGREES: Final = 0.0
HDR_DOME_VISIBLE_IN_PRIMARY_RAYS: Final = True

CAMERA_STAND_PRIM_PATH: Final = "/World/Scene/CameraStand"
# The authored boom extends along local -Y.  Rotating it -90 degrees about
# world X makes that axis vertical; the resulting base lies on the table
# between the two Piper mounts.
CAMERA_STAND_POSITION: Final = (0.0, -0.47, TABLE_TOP_Z)
CAMERA_STAND_ORIENTATION_RAW: Final = (0.707, -0.707, 0.0, 0.0)
CAMERA_STAND_SOURCE_BBOX_MIN: Final = (-0.1500001, -0.613153, -0.045001)
CAMERA_STAND_SOURCE_BBOX_MAX: Final = (0.1500001, 0.000001, 0.067434)

LEFT_PIPER_PRIM_PATH: Final = "/World/Robots/LeftPiper"
RIGHT_PIPER_PRIM_PATH: Final = "/World/Robots/RightPiper"
PIPER_BASE_ORIENTATION_RAW: Final = (0.707, 0.0, 0.0, 0.707)
PIPER_ARM_JOINT_NAMES: Final = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)
PIPER_GRIPPER_JOINT_NAMES: Final = ("gripper_joint", "joint8")
PIPER_DOF_NAMES: Final = PIPER_ARM_JOINT_NAMES + PIPER_GRIPPER_JOINT_NAMES
PIPER_COMMAND_JOINT_NAMES: Final = PIPER_ARM_JOINT_NAMES + ("gripper_joint",)
PIPER_BASE_LINK: Final = "base_link"
PIPER_URDF_TOOL_LINK: Final = "gripper_center"
PIPER_TOOL_LINK: Final = "finger_center"
PIPER_WRIST_LINK: Final = "link6"
PIPER_CAMERA_MOUNT_REL_PATH: Final = "link6/camera"
PIPER_TOOL_REL_PATH: Final = "link6/gripper_center"
# The asset and repository Piper example define the six-axis zero state as the
# retracted waiting pose.  Its measured tool position and collision clearance
# are verified in simulation rather than inferred from the joint values.
PIPER_HOME_JOINT_POSITION: Final = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
PIPER_OPEN_GRIPPER_POSITION: Final = 0.04
PIPER_CLOSED_GRIPPER_POSITION: Final = 0.0
PIPER_HOME_DOF_POSITION: Final = PIPER_HOME_JOINT_POSITION + (
    PIPER_OPEN_GRIPPER_POSITION,
    PIPER_OPEN_GRIPPER_POSITION,
)
PIPER_HOME_TOLERANCE_RAD: Final = 0.02
PIPER_GRIPPER_TOLERANCE_M: Final = 0.001
PIPER_OPEN_FINGER_SEPARATION_M: Final = 0.08
PIPER_HOME_TOOL_FORWARD_RANGE_M: Final = (0.15, 0.25)
PIPER_HOME_SETTLE_STEPS: Final = 60

PHYSICS_FREQUENCY_HZ: Final = 120
CONTROL_FREQUENCY_HZ: Final = 30
RENDER_FREQUENCY_HZ: Final = 30
CAMERA_FREQUENCY_HZ: Final = 30
PHYSICS_DT: Final = 1.0 / PHYSICS_FREQUENCY_HZ
CONTROL_DT: Final = 1.0 / CONTROL_FREQUENCY_HZ
RENDER_DT: Final = 1.0 / RENDER_FREQUENCY_HZ

CAMERA_RESOLUTION: Final = (640, 480)
CAMERA_RGB_DTYPE: Final = "uint8"
CAMERA_RGB_CHANNEL_ORDER: Final = "RGB"
CAMERA_DEPTH_DTYPE: Final = "float32"
CAMERA_DEPTH_DEFINITION: Final = "distance_to_image_plane"
CAMERA_DEPTH_UNIT: Final = "m"
CAMERA_INVALID_DEPTH: Final = "NaN"
CAMERA_FOCAL_LENGTH_MM: Final = 1.93
CAMERA_HORIZONTAL_APERTURE_MM: Final = 2.65
WRIST_CAMERA_CLIPPING_RANGE: Final = (0.03, 3.0)
OVERHEAD_CAMERA_CLIPPING_RANGE: Final = (0.10, 6.0)
LEFT_WRIST_CAMERA_NAME: Final = "left_wrist_camera"
RIGHT_WRIST_CAMERA_NAME: Final = "right_wrist_camera"
OVERHEAD_CAMERA_NAME: Final = "overhead_camera"
CAMERA_SENSOR_PRIM_NAME: Final = "D435Sensor"
LEFT_WRIST_CAMERA_PRIM_PATH: Final = (
    f"{LEFT_PIPER_PRIM_PATH}/{PIPER_CAMERA_MOUNT_REL_PATH}/"
    f"{CAMERA_SENSOR_PRIM_NAME}"
)
RIGHT_WRIST_CAMERA_PRIM_PATH: Final = (
    f"{RIGHT_PIPER_PRIM_PATH}/{PIPER_CAMERA_MOUNT_REL_PATH}/"
    f"{CAMERA_SENSOR_PRIM_NAME}"
)
OVERHEAD_CAMERA_PRIM_PATH: Final = (
    f"{CAMERA_STAND_PRIM_PATH}/{CAMERA_SENSOR_PRIM_NAME}"
)
# The Piper helper is a ROS optical frame.  A local 180 degree X rotation
# converts its +Z optical direction to the USD camera's -Z viewing direction.
WRIST_CAMERA_LOCAL_USD_ORIENTATION: Final = (0.0, 1.0, 0.0, 0.0)
# Fixed world pose supplied for the sensor at the top of the stand.  This is a
# USD camera-frame orientation: +Y is image-up and -Z is optical-forward.
OVERHEAD_CAMERA_POSITION: Final = (0.0, -0.41, 1.308)
OVERHEAD_CAMERA_USD_ORIENTATION_RAW: Final = (0.9659258, 0.2588190, 0.0, 0.0)
OVERHEAD_CAMERA_TARGET: Final = (0.0, -0.05, TABLE_TOP_Z)
CAMERA_RENDER_WARMUP_STEPS: Final = 24
CAMERA_TARGET_PIXEL_MARGIN: Final = 2.0
SCENE_PREVIEW_EYE: Final = (2.1, -2.3, 1.9)
SCENE_PREVIEW_TARGET: Final = (0.0, -0.05, 0.75)

MATRYOSHKA_UUID: Final = "a5a44251-6d8c-4cee-a8d7-90c443e47e53"
MATRYOSHKA_SORT_ORDER: Final = ("00004", "00003", "00002", "00001", "00000")
MATRYOSHKA_PICK_ORDER: Final = MATRYOSHKA_SORT_ORDER
MATRYOSHKA_TARGET_GAP: Final = 0.025
MATRYOSHKA_INITIAL_GAP: Final = 0.04
MATRYOSHKA_POSITION_TOLERANCE: Final = 0.02
MATRYOSHKA_UPRIGHT_TOLERANCE_DEGREES: Final = 10.0
MATRYOSHKA_LINEAR_SPEED_TOLERANCE: Final = 0.01
MATRYOSHKA_ANGULAR_SPEED_TOLERANCE: Final = 0.10
MATRYOSHKA_PRIM_ROOT: Final = "/World/Objects"
MATRYOSHKA_PHYSICS_MATERIAL_PATH: Final = "/World/Looks/MatryoshkaPhysics"
MATRYOSHKA_PHYSICS_RESTITUTION: Final = 0.05
MATRYOSHKA_LINEAR_DAMPING: Final = 0.10
MATRYOSHKA_ANGULAR_DAMPING: Final = 0.10
MATRYOSHKA_SLEEP_THRESHOLD: Final = 0.005
MATRYOSHKA_STABILIZATION_THRESHOLD: Final = 0.001
# This central area remains inside the overhead view and leaves generous room
# for grasp approach, the table edges, and the rear robot bases.
# Keep complete doll geometry inside the fixed overhead D435 frustum, including
# the largest doll at either sampling boundary.
MATRYOSHKA_RANDOM_X_RANGE: Final = (-0.22, 0.22)
MATRYOSHKA_RANDOM_Y_RANGE: Final = (-0.22, 0.17)
MATRYOSHKA_TABLE_EDGE_CLEARANCE: Final = 0.08
MATRYOSHKA_ROBOT_BASE_EXCLUSION_RADIUS: Final = 0.16
MATRYOSHKA_LAYOUT_SAMPLES_PER_OBJECT: Final = 500
MATRYOSHKA_SPAWN_CLEARANCE: Final = 0.003
MATRYOSHKA_TABLE_HEIGHT_TOLERANCE: Final = 0.008
MATRYOSHKA_SETTLE_MAX_STEPS: Final = 1_440
MATRYOSHKA_STABLE_CONSECUTIVE_STEPS: Final = 30

# cuRobo owns only the six arm joints.  The active gripper joint is locked
# open in its kinematic model (the URDF mimic parser follows joint8
# automatically) and is commanded explicitly in simulation.
CUROBO_DEVICE: Final = "cuda:0"
CUROBO_NUM_IK_SEEDS: Final = 64
CUROBO_NUM_TRAJOPT_SEEDS: Final = 4
CUROBO_MAX_PLAN_ATTEMPTS: Final = 5
CUROBO_POSITION_TOLERANCE_M: Final = 0.008
CUROBO_ORIENTATION_TOLERANCE_RAD: Final = 0.08
CUROBO_COLLISION_ACTIVATION_DISTANCE_M: Final = 0.005
CUROBO_CURRENT_STATE_LIMIT_MARGIN_RAD: Final = 1.0e-5
CUROBO_MAX_CURRENT_STATE_PROJECTION_RAD: Final = 1.0e-3
CUROBO_COLLISION_CACHE: Final = {"cuboid": 32, "sphere": 160}
CUROBO_ATTACHED_OBJECT_LINK: Final = "attached_object"
CUROBO_ATTACHED_OBJECT_SPHERES: Final = 4
CUROBO_ATTACHED_OBJECT_INSET_M: Final = 0.009
CUROBO_MAX_TRAJECTORY_STEPS: Final = 1_000
CUROBO_FINAL_EXECUTION_TOLERANCE_RAD: Final = 0.01
CUROBO_MAX_EXECUTION_ERROR_RAD: Final = 0.080
CUROBO_FINAL_SETTLE_MAX_CONTROL_FRAMES: Final = 30
CUROBO_GRASP_EXECUTION_TOLERANCE_RAD: Final = 0.003
CUROBO_GRASP_SETTLE_MAX_CONTROL_FRAMES: Final = 60
CUROBO_WORKER_RESPONSE_PREFIX: Final = "CUROBO_WORKER_RESPONSE "
CUROBO_WORKER_TIMEOUT_S: Final = 60.0
GRASP_JOINT_ROOT: Final = "/World/GraspJoints"
PICK_SMOKE_ASSET_ID: Final = "00001"
PICK_GRIPPER_CLOSE_STEPS: Final = 120
PICK_RELEASE_SETTLE_STEPS: Final = 60
PICK_RELEASE_FINGER_MARGIN_M: Final = 0.006

EPISODE_SCHEMA_VERSION: Final = "1.0.0"
EPISODE_HDF5_COMPRESSION: Final = "gzip"
EPISODE_HDF5_COMPRESSION_LEVEL: Final = 1
EPISODE_DATASET_ALLOCATION_BLOCK: Final = 32
EPISODE_MAX_RUNTIME_S: Final = 1_800.0
EPISODE_OBJECT_IDS: Final = MATRYOSHKA_SORT_ORDER
EPISODE_ROBOT_NAMES: Final = ("left", "right")
EPISODE_CAMERA_NAMES: Final = (
    LEFT_WRIST_CAMERA_NAME,
    RIGHT_WRIST_CAMERA_NAME,
    OVERHEAD_CAMERA_NAME,
)
GRASP_EVENT_NONE: Final = 0
GRASP_EVENT_ATTACH: Final = 1
GRASP_EVENT_DETACH: Final = 2
GRASP_EVENT_NAMES: Final = {
    GRASP_EVENT_NONE: "none",
    GRASP_EVENT_ATTACH: "attach_fixed_joint",
    GRASP_EVENT_DETACH: "detach_fixed_joint",
}
EPISODE_WORKER_SUCCESS_MARKERS: Final = {
    "collect-worker": "HDF5_EXPERT_RECORDING_OK",
    "replay-worker": "HDF5_ACTION_REPLAY_OK",
}

# The URDF's ``gripper_center`` origin is at the distal finger plane.  Both
# finger meshes extend along its local -X axis, so their longitudinal centre
# is about 40 mm at local X=-0.040 rather than +0.040.  cuRobo plans the
# virtual ``finger_center`` frame authored below at exactly that offset.
# In this nominal top-down orientation, local +X points toward the fingertips
# and local +Y lies along world +X, the gripper closing direction.
PIPER_TOP_DOWN_TOOL_WORLD_ORIENTATION: Final = (
    0.5,
    0.5,
    0.5,
    -0.5,
)
PIPER_FINGER_CENTER_BELOW_TOOL_M: Final = -0.040
PIPER_FINGER_CENTER_OFFSET_IN_TOOL_M: Final = (
    PIPER_FINGER_CENTER_BELOW_TOOL_M,
    0.0,
    0.0,
)
PIPER_GRASP_TOOL_HEIGHT_ABOVE_TABLE_M: Final = 0.082
PIPER_GRASP_CENTER_TOLERANCE_M: Final = 0.008
PIPER_GRASP_MAX_CENTER_TOLERANCE_M: Final = 0.012
PIPER_GRASP_CENTER_DIAMETER_FRACTION: Final = 0.25
PIPER_GRASP_CORRECTION_TRIGGER_M: Final = 0.004
PIPER_NEAR_GRASP_ALIGNMENT_TOLERANCE_M: Final = 0.005
PIPER_GRASP_MAX_AXIAL_ERROR_M: Final = 0.035
PIPER_GRASP_MIN_SEPARATION_M: Final = 0.005
PIPER_GRASP_MIN_DIAMETER_FRACTION: Final = 0.25
PIPER_GRASP_MIN_DOWNWARD_AXIS_COMPONENT: Final = 0.45
PIPER_NOMINAL_GRASP_HEIGHT_M: Final = 0.035
PIPER_LARGE_DOLL_DIAMETER_THRESHOLD_M: Final = 0.070
PIPER_LARGE_DOLL_GRASP_HEIGHT_M: Final = 0.085
PIPER_LARGE_DOLL_CENTER_ABOVE_TOP_M: Final = 0.020
PIPER_LARGE_DOLL_MIN_DOWNWARD_AXIS_COMPONENT: Final = 0.75
PIPER_REGULAR_DOLL_MAX_CLOSING_AXIS_WORLD_Z: Final = 0.50
PIPER_LARGE_DOLL_MAX_CLOSING_AXIS_WORLD_Z: Final = 0.10
PIPER_LARGE_DOLL_GRASP_SEED_CANDIDATES: Final = 32
PIPER_APPROACH_MAX_DOLL_DISPLACEMENT_M: Final = 0.004
PIPER_PREGRASP_CLEARANCE_M: Final = 0.110
PIPER_NEAR_GRASP_CLEARANCE_M: Final = 0.040
PIPER_FINAL_APPROACH_CLEARANCES_M: Final = (
    0.030,
    0.020,
    0.010,
    0.0,
)
PIPER_LARGE_DOLL_FINAL_APPROACH_CLEARANCES_M: Final = (
    0.030,
    0.020,
    0.015,
)
PIPER_GRASP_SEED_CANDIDATES: Final = 6
PIPER_PREGRASP_CLEARANCE_CANDIDATES_M: Final = (
    PIPER_PREGRASP_CLEARANCE_M,
    0.090,
    0.070,
    0.055,
)
PIPER_LIFT_CLEARANCE_M: Final = 0.130
PIPER_PREPLACE_CLEARANCE_M: Final = 0.120
PIPER_LARGE_DOLL_PREPLACE_CLEARANCE_M: Final = 0.050
PIPER_RETREAT_CLEARANCE_M: Final = 0.130
PIPER_LARGE_DOLL_RETREAT_CLEARANCE_M: Final = 0.0
PIPER_RELEASE_AXIS_CLEARANCES_M: Final = (0.020, 0.040, 0.060)
PIPER_UPRIGHT_YAW_OFFSETS_RAD: Final = (
    0.0,
    math.pi / 4.0,
    -math.pi / 4.0,
    math.pi / 2.0,
    -math.pi / 2.0,
    math.pi,
)
PIPER_TRANSPORT_UPRIGHT_TOLERANCE_DEGREES: Final = 9.0
PIPER_POST_GRASP_UPRIGHT_TOLERANCE_DEGREES: Final = 2.0
PIPER_PLANNED_UPRIGHT_TOLERANCE_DEGREES: Final = 1.5
PIPER_CONSTRAINED_PLACE_TOLERANCE_M: Final = 0.004
# cuRobo must not terminate a collision-avoidance trajectory at exact support
# contact.  Hold the still-attached doll just above the tabletop, open both
# fingers, then detach and let full PhysX geometry settle this small gap.
PIPER_PLANNED_PLACE_SUPPORT_CLEARANCE_M: Final = 0.002
PIPER_PLACE_APPROACH_CLEARANCES_M: Final = (
    0.030,
    0.015,
    PIPER_PLANNED_PLACE_SUPPORT_CLEARANCE_M,
)


@dataclass(frozen=True)
class PoseSpec:
    """A pose using metres and a wxyz quaternion."""

    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


@dataclass(frozen=True)
class RobotSpec:
    """Static configuration for one Piper instance."""

    name: str
    prim_path: str
    base_pose: PoseSpec


@dataclass(frozen=True)
class DollSpec:
    """Authoritative metadata for one of the five selected dolls."""

    asset_id: str
    asset_path: Path
    metadata_path: Path
    height: float
    size: tuple[float, float, float]
    footprint_radius: float
    mass: float
    friction: float
    uuid: str


@dataclass(frozen=True)
class DollPlacement:
    """A deterministic upright doll placement."""

    asset_id: str
    pose: PoseSpec
    yaw_rad: float


@dataclass(frozen=True)
class CameraSpec:
    """Shared pinhole approximation for one logical D435 RGB-D camera."""

    name: str
    resolution: tuple[int, int]
    frequency_hz: int
    focal_length_mm: float
    horizontal_aperture_mm: float
    clipping_range: tuple[float, float]
    depth_definition: str = CAMERA_DEPTH_DEFINITION


def normalize_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    """Return a normalized wxyz quaternion and reject a zero norm."""

    if len(quaternion) != 4:
        raise ValueError(f"Expected four quaternion components, got {len(quaternion)}")
    norm = math.sqrt(sum(float(component) ** 2 for component in quaternion))
    if norm <= 0.0:
        raise ValueError("Cannot normalize a zero quaternion")
    return tuple(float(component) / norm for component in quaternion)  # type: ignore[return-value]


def quaternion_conjugate(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    """Return the conjugate of a normalized-or-unnormalized wxyz quaternion."""

    w, x, y, z = (float(value) for value in quaternion)
    return (w, -x, -y, -z)


def quaternion_multiply(
    first: Sequence[float],
    second: Sequence[float],
) -> tuple[float, float, float, float]:
    """Compose two wxyz quaternions as ``first * second``."""

    aw, ax, ay, az = (float(value) for value in first)
    bw, bx, by, bz = (float(value) for value in second)
    return normalize_quaternion(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )
    )


def upright_yaw_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    """Keep an object's world yaw while removing its roll and pitch."""

    w, x, y, z = normalize_quaternion(quaternion)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return (
        math.cos(0.5 * yaw),
        0.0,
        0.0,
        math.sin(0.5 * yaw),
    )


def attached_object_upright_tilt_degrees(
    tool_world_orientation: Sequence[float],
    tool_to_object_orientation: Sequence[float],
) -> float:
    """Predict a rigidly attached object's world roll/pitch magnitude."""

    object_world_orientation = quaternion_multiply(
        tool_world_orientation,
        tool_to_object_orientation,
    )
    object_up = _quaternion_rotate_vector(
        object_world_orientation,
        (0.0, 0.0, 1.0),
    )
    return math.degrees(
        math.acos(min(1.0, max(-1.0, float(object_up[2]))))
    )


def world_pose_to_robot_base(spec: RobotSpec, pose: PoseSpec) -> PoseSpec:
    """Express a world pose in one Piper base frame."""

    inverse_base_orientation = quaternion_conjugate(
        normalize_quaternion(spec.base_pose.quaternion)
    )
    world_offset = tuple(
        pose.position[index] - spec.base_pose.position[index]
        for index in range(3)
    )
    base_position = _quaternion_rotate_vector(
        inverse_base_orientation,
        world_offset,
    )
    base_orientation = quaternion_multiply(
        inverse_base_orientation,
        pose.quaternion,
    )
    return PoseSpec(base_position, base_orientation)


def robot_base_pose_to_world(spec: RobotSpec, pose: PoseSpec) -> PoseSpec:
    """Express a pose from one Piper base frame in world coordinates."""

    world_offset = _quaternion_rotate_vector(
        spec.base_pose.quaternion,
        pose.position,
    )
    world_position = tuple(
        spec.base_pose.position[index] + world_offset[index]
        for index in range(3)
    )
    world_orientation = quaternion_multiply(
        spec.base_pose.quaternion,
        pose.quaternion,
    )
    return PoseSpec(world_position, world_orientation)


def pose_relative_to(parent: PoseSpec, child: PoseSpec) -> PoseSpec:
    """Express ``child`` in ``parent`` coordinates."""

    inverse_parent_orientation = quaternion_conjugate(
        normalize_quaternion(parent.quaternion)
    )
    offset = tuple(
        child.position[index] - parent.position[index] for index in range(3)
    )
    return PoseSpec(
        _quaternion_rotate_vector(inverse_parent_orientation, offset),
        quaternion_multiply(inverse_parent_orientation, child.quaternion),
    )


def tool_pose_for_attached_object_pose(
    tool_pose: PoseSpec,
    object_pose: PoseSpec,
    desired_object_pose: PoseSpec,
) -> PoseSpec:
    """Solve a tool pose from a measured rigid tool-to-object transform."""

    tool_to_object = pose_relative_to(tool_pose, object_pose)
    desired_tool_orientation = quaternion_multiply(
        desired_object_pose.quaternion,
        quaternion_conjugate(tool_to_object.quaternion),
    )
    desired_world_offset = _quaternion_rotate_vector(
        desired_tool_orientation,
        tool_to_object.position,
    )
    return PoseSpec(
        tuple(
            desired_object_pose.position[index] - desired_world_offset[index]
            for index in range(3)
        ),  # type: ignore[arg-type]
        desired_tool_orientation,
    )


def tool_pose_for_attached_object_orientation(
    tool_pose: PoseSpec,
    object_pose: PoseSpec,
    desired_object_orientation: Sequence[float],
) -> PoseSpec:
    """Rotate a grasped object about its centre to a requested orientation."""

    return tool_pose_for_attached_object_pose(
        tool_pose,
        object_pose,
        PoseSpec(
            object_pose.position,
            normalize_quaternion(desired_object_orientation),
        ),
    )


def piper_finger_center_pose(tool_pose: PoseSpec) -> PoseSpec:
    """Transform the authored distal tool pose to the physical finger centre."""

    offset = _quaternion_rotate_vector(
        tool_pose.quaternion,
        PIPER_FINGER_CENTER_OFFSET_IN_TOOL_M,
    )
    return PoseSpec(
        tuple(
            tool_pose.position[index] + offset[index] for index in range(3)
        ),  # type: ignore[arg-type]
        tool_pose.quaternion,
    )


def piper_axis_approach_pose(
    grasp_pose: PoseSpec,
    clearance_m: float,
) -> PoseSpec:
    """Back the physical finger centre away along the grasp tool axis."""

    if clearance_m < 0.0:
        raise ValueError("Grasp approach clearance cannot be negative")
    approach_axis = _quaternion_rotate_vector(
        grasp_pose.quaternion,
        (1.0, 0.0, 0.0),
    )
    return PoseSpec(
        tuple(
            grasp_pose.position[index]
            - clearance_m * approach_axis[index]
            for index in range(3)
        ),  # type: ignore[arg-type]
        grasp_pose.quaternion,
    )


def piper_horizontal_closing_orientation_candidates(
    quaternion: Sequence[float],
) -> tuple[
    tuple[tuple[float, float, float, float], float],
    ...,
]:
    """Roll a tool pose so its local closing axis is horizontal.

    Local +X is the longitudinal finger/approach axis and local +Y is the
    symmetric closing direction.  Rolling about +X preserves the already
    selected approach direction while giving the wrist two equivalent
    horizontal-closing configurations separated by pi radians.
    """

    normalized = normalize_quaternion(quaternion)
    local_y_world = _quaternion_rotate_vector(
        normalized,
        (0.0, 1.0, 0.0),
    )
    local_z_world = _quaternion_rotate_vector(
        normalized,
        (0.0, 0.0, 1.0),
    )
    roll = math.atan2(
        -local_y_world[2],
        local_z_world[2],
    )
    if math.hypot(local_y_world[2], local_z_world[2]) <= 1.0e-12:
        return ((normalized, 0.0),)
    alternate_roll = roll - math.copysign(math.pi, roll or 1.0)
    candidates = tuple(
        (
            quaternion_multiply(
                normalized,
                (
                    math.cos(0.5 * candidate_roll),
                    math.sin(0.5 * candidate_roll),
                    0.0,
                    0.0,
                ),
            ),
            candidate_roll,
        )
        for candidate_roll in (roll, alternate_roll)
    )
    return tuple(sorted(candidates, key=lambda candidate: abs(candidate[1])))


def piper_grasp_contact_height(doll_spec: DollSpec) -> float:
    """Choose a finger centre height that clears each doll's widest section."""

    nominal_height = min(
        PIPER_NOMINAL_GRASP_HEIGHT_M,
        0.35 * doll_spec.height,
    )
    diameter = 2.0 * doll_spec.footprint_radius
    if diameter < PIPER_LARGE_DOLL_DIAMETER_THRESHOLD_M:
        return nominal_height
    return min(
        PIPER_LARGE_DOLL_GRASP_HEIGHT_M,
        0.5 * doll_spec.height + PIPER_LARGE_DOLL_CENTER_ABOVE_TOP_M,
    )


def piper_grasp_search_parameters(doll_spec: DollSpec) -> tuple[float, int]:
    """Return the safe downward-axis threshold and IK search budget."""

    diameter = 2.0 * doll_spec.footprint_radius
    if diameter < PIPER_LARGE_DOLL_DIAMETER_THRESHOLD_M:
        return (
            PIPER_GRASP_MIN_DOWNWARD_AXIS_COMPONENT,
            PIPER_GRASP_SEED_CANDIDATES,
        )
    return (
        PIPER_LARGE_DOLL_MIN_DOWNWARD_AXIS_COMPONENT,
        PIPER_LARGE_DOLL_GRASP_SEED_CANDIDATES,
    )


def piper_final_approach_clearances(doll_spec: DollSpec) -> tuple[float, ...]:
    """Stop a large doll's long fingertips before they enter its wide waist."""

    diameter = 2.0 * doll_spec.footprint_radius
    if diameter < PIPER_LARGE_DOLL_DIAMETER_THRESHOLD_M:
        return PIPER_FINAL_APPROACH_CLEARANCES_M
    return PIPER_LARGE_DOLL_FINAL_APPROACH_CLEARANCES_M


def piper_preplace_clearance(doll_spec: DollSpec) -> float:
    """Keep a top-grasped large doll within Piper's transport workspace."""

    diameter = 2.0 * doll_spec.footprint_radius
    if diameter < PIPER_LARGE_DOLL_DIAMETER_THRESHOLD_M:
        return PIPER_PREPLACE_CLEARANCE_M
    return PIPER_LARGE_DOLL_PREPLACE_CLEARANCE_M


def piper_post_axis_retreat_clearance(doll_spec: DollSpec) -> float:
    """Avoid an unreachable extra lift after a top-grasped large-doll retreat."""

    diameter = 2.0 * doll_spec.footprint_radius
    if diameter < PIPER_LARGE_DOLL_DIAMETER_THRESHOLD_M:
        return PIPER_RETREAT_CLEARANCE_M
    return PIPER_LARGE_DOLL_RETREAT_CLEARANCE_M


def piper_planned_place_center(
    target_center: Sequence[float],
) -> tuple[float, float, float]:
    """Return the collision-free centre used before physical support settling."""

    if len(target_center) != 3:
        raise ValueError("A placement centre must contain three coordinates")
    return (
        float(target_center[0]),
        float(target_center[1]),
        float(target_center[2]) + PIPER_PLANNED_PLACE_SUPPORT_CLEARANCE_M,
    )


CAMERA_STAND_ORIENTATION: Final = normalize_quaternion(
    CAMERA_STAND_ORIENTATION_RAW
)
OVERHEAD_CAMERA_USD_ORIENTATION: Final = normalize_quaternion(
    OVERHEAD_CAMERA_USD_ORIENTATION_RAW
)
PIPER_BASE_ORIENTATION: Final = normalize_quaternion(PIPER_BASE_ORIENTATION_RAW)
LEFT_PIPER: Final = RobotSpec(
    name="left",
    prim_path=LEFT_PIPER_PRIM_PATH,
    base_pose=PoseSpec((-0.3, -0.45, TABLE_TOP_Z), PIPER_BASE_ORIENTATION),
)
RIGHT_PIPER: Final = RobotSpec(
    name="right",
    prim_path=RIGHT_PIPER_PRIM_PATH,
    base_pose=PoseSpec((0.3, -0.45, TABLE_TOP_Z), PIPER_BASE_ORIENTATION),
)
ROBOT_SPECS: Final = (LEFT_PIPER, RIGHT_PIPER)

CAMERA_SPECS: Final = (
    CameraSpec(
        LEFT_WRIST_CAMERA_NAME,
        CAMERA_RESOLUTION,
        CAMERA_FREQUENCY_HZ,
        CAMERA_FOCAL_LENGTH_MM,
        CAMERA_HORIZONTAL_APERTURE_MM,
        WRIST_CAMERA_CLIPPING_RANGE,
    ),
    CameraSpec(
        RIGHT_WRIST_CAMERA_NAME,
        CAMERA_RESOLUTION,
        CAMERA_FREQUENCY_HZ,
        CAMERA_FOCAL_LENGTH_MM,
        CAMERA_HORIZONTAL_APERTURE_MM,
        WRIST_CAMERA_CLIPPING_RANGE,
    ),
    CameraSpec(
        OVERHEAD_CAMERA_NAME,
        CAMERA_RESOLUTION,
        CAMERA_FREQUENCY_HZ,
        CAMERA_FOCAL_LENGTH_MM,
        CAMERA_HORIZONTAL_APERTURE_MM,
        OVERHEAD_CAMERA_CLIPPING_RANGE,
    ),
)
CAMERA_PRIM_PATHS: Final = {
    LEFT_WRIST_CAMERA_NAME: LEFT_WRIST_CAMERA_PRIM_PATH,
    RIGHT_WRIST_CAMERA_NAME: RIGHT_WRIST_CAMERA_PRIM_PATH,
    OVERHEAD_CAMERA_NAME: OVERHEAD_CAMERA_PRIM_PATH,
}


def _package_version(*distribution_names: str) -> str:
    for distribution_name in distribution_names:
        try:
            return importlib.metadata.version(distribution_name)
        except importlib.metadata.PackageNotFoundError:
            continue
    return "unknown"


def runtime_versions() -> dict[str, str]:
    """Return version strings without importing Isaac Sim or cuRobo."""

    return {
        "python": platform.python_version(),
        "isaac_sim": _package_version("isaacsim"),
        "isaac_lab": _package_version("isaaclab"),
        "curobo": _package_version("nvidia-curobo", "curobo"),
        "torch": _package_version("torch"),
        "warp": _package_version("warp-lang"),
        "h5py": _package_version("h5py"),
    }


def pose_matrix(pose: PoseSpec) -> list[list[float]]:
    """Return a camera/object-to-world homogeneous matrix for a wxyz pose."""

    w, x, y, z = normalize_quaternion(pose.quaternion)
    rotation = (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )
    return [
        [*rotation[0], pose.position[0]],
        [*rotation[1], pose.position[1]],
        [*rotation[2], pose.position[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def world_to_local_matrix(pose: PoseSpec) -> list[list[float]]:
    """Return the inverse homogeneous transform for ``pose``."""

    matrix = pose_matrix(pose)
    rotation_transpose = [
        [matrix[column][row] for column in range(3)]
        for row in range(3)
    ]
    translation = [
        -sum(
            rotation_transpose[row][column] * pose.position[column]
            for column in range(3)
        )
        for row in range(3)
    ]
    return [
        [*rotation_transpose[0], translation[0]],
        [*rotation_transpose[1], translation[1]],
        [*rotation_transpose[2], translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def build_episode_metadata(
    *,
    episode_id: str,
    seed: int,
    planner_seed: int,
    sampled_layout: Sequence[DollPlacement],
    initial_doll_report: dict[str, Any] | None = None,
    camera_report: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build the complete static, calibration, and task metadata document."""

    specs_by_id = {spec.asset_id: spec for spec in get_doll_specs()}
    targets = {
        placement.asset_id: placement
        for placement in compute_doll_target_layout()
    }
    sampled_by_id = {
        placement.asset_id: placement for placement in sampled_layout
    }
    assignments = assign_dolls_to_robots(
        {
            asset_id: sampled_by_id[asset_id].pose
            for asset_id in EPISODE_OBJECT_IDS
        }
    )
    return {
        "episode": {
            "id": episode_id,
            "created_at_utc": datetime.datetime.now(
                datetime.timezone.utc
            ).isoformat(),
            "schema_version": EPISODE_SCHEMA_VERSION,
        },
        "versions": runtime_versions(),
        "seeds": {
            "episode": int(seed),
            "numpy": int(seed),
            "torch": int(seed),
            "simulation": int(seed),
            "curobo_motion_planner": int(planner_seed),
        },
        "assets": {
            "repository_root": str(REPOSITORY_ROOT.resolve()),
            "piper_usd": str(PIPER_USD.resolve()),
            "piper_urdf": str(PIPER_URDF.resolve()),
            "room_usd": str(ROOM_USD.resolve()),
            "camera_stand_usd": str(CAMERA_STAND_USD.resolve()),
            "hdr_texture": str(HDR_TEXTURE.resolve()),
            "table_mdl": str(TABLE_MDL.resolve()),
            "ground_mdl": str(GROUND_MDL.resolve()),
            "matryoshka_usdz": {
                asset_id: str(specs_by_id[asset_id].asset_path.resolve())
                for asset_id in EPISODE_OBJECT_IDS
            },
        },
        "coordinates": {
            "world_units": "m",
            "world_up_axis": WORLD_UP_AXIS,
            "quaternion_order": QUATERNION_ORDER,
            "pose_layout": "[x,y,z,qw,qx,qy,qz]",
            "velocity_units": {
                "linear": "m/s",
                "angular": "rad/s",
            },
        },
        "timing": {
            "physics_frequency_hz": PHYSICS_FREQUENCY_HZ,
            "control_frequency_hz": CONTROL_FREQUENCY_HZ,
            "render_frequency_hz": RENDER_FREQUENCY_HZ,
            "camera_frequency_hz": CAMERA_FREQUENCY_HZ,
            "physics_dt_s": PHYSICS_DT,
            "control_dt_s": CONTROL_DT,
            "render_dt_s": RENDER_DT,
            "physics_steps_per_control": (
                PHYSICS_FREQUENCY_HZ // CONTROL_FREQUENCY_HZ
            ),
        },
        "scene": {
            "ground": {
                "prim_path": GROUND_PRIM_PATH,
                "position_m": list(GROUND_POSITION),
                "quaternion_wxyz": list(GROUND_ORIENTATION),
                "size_m": list(GROUND_SIZE),
                "material_path": str(GROUND_MDL.resolve()),
                "material_entry": GROUND_MATERIAL_ENTRY,
            },
            "table": {
                "prim_path": TABLE_PRIM_PATH,
                "position_m": list(TABLE_POSITION),
                "quaternion_wxyz": list(TABLE_ORIENTATION),
                "size_m": list(TABLE_SIZE),
                "top_z_m": TABLE_TOP_Z,
                "material_path": str(TABLE_MDL.resolve()),
                "material_entry": TABLE_MATERIAL_ENTRY,
            },
            "camera_stand": {
                "prim_path": CAMERA_STAND_PRIM_PATH,
                "position_m": list(CAMERA_STAND_POSITION),
                "quaternion_wxyz": list(CAMERA_STAND_ORIENTATION),
                "scale": [1.0, 1.0, 1.0],
                "static_geometry": True,
            },
            "room": {
                "prim_path": ROOM_PRIM_PATH,
                "asset_path": str(ROOM_USD.resolve()),
                "residual_light_disabled": ROOM_RESIDUAL_LIGHT_REL_PATH,
            },
            "dome_light": {
                "prim_path": HDR_DOME_PRIM_PATH,
                "texture_path": str(HDR_TEXTURE.resolve()),
                "intensity": HDR_DOME_INTENSITY,
                "rotation_degrees": HDR_DOME_ROTATION_DEGREES,
                "visible_in_primary_rays": HDR_DOME_VISIBLE_IN_PRIMARY_RAYS,
            },
        },
        "robots": [
            {
                "name": spec.name,
                "prim_path": spec.prim_path,
                "base_pose": asdict(spec.base_pose),
                "home_arm_joint_position_rad": list(
                    PIPER_HOME_JOINT_POSITION
                ),
                "open_gripper_position_m": PIPER_OPEN_GRIPPER_POSITION,
                "simulation_dof_order": list(PIPER_DOF_NAMES),
                "arm_action_joint_order": list(PIPER_ARM_JOINT_NAMES),
                "gripper_action_joint": "gripper_joint",
                "mimic_joint": "joint8",
                "tool_frame": PIPER_TOOL_LINK,
            }
            for spec in ROBOT_SPECS
        ],
        "objects": [
            {
                "asset_id": asset_id,
                "uuid": specs_by_id[asset_id].uuid,
                "height_m": specs_by_id[asset_id].height,
                "size_m": list(specs_by_id[asset_id].size),
                "footprint_radius_m": specs_by_id[
                    asset_id
                ].footprint_radius,
                "mass_kg": specs_by_id[asset_id].mass,
                "friction": specs_by_id[asset_id].friction,
                "sampled_pose": asdict(sampled_by_id[asset_id].pose),
                "sampled_yaw_rad": sampled_by_id[asset_id].yaw_rad,
                "target_pose": asdict(targets[asset_id].pose),
            }
            for asset_id in EPISODE_OBJECT_IDS
        ],
        "initial_doll_validation": (
            {} if initial_doll_report is None else initial_doll_report
        ),
        "cameras": [
            {
                "name": spec.name,
                "prim_path": CAMERA_PRIM_PATHS[spec.name],
                "resolution_wh": list(spec.resolution),
                "frequency_hz": spec.frequency_hz,
                "rgb_dtype": CAMERA_RGB_DTYPE,
                "rgb_channel_order": CAMERA_RGB_CHANNEL_ORDER,
                "depth_dtype": CAMERA_DEPTH_DTYPE,
                "depth_definition": spec.depth_definition,
                "depth_unit": CAMERA_DEPTH_UNIT,
                "invalid_depth": CAMERA_INVALID_DEPTH,
                "focal_length_mm": spec.focal_length_mm,
                "horizontal_aperture_mm": spec.horizontal_aperture_mm,
                "clipping_range_m": list(spec.clipping_range),
                "intrinsics": camera_intrinsics(spec),
                "pose_axes": "USD camera frame",
                "configured_pose": (
                    {
                        "position_m": list(OVERHEAD_CAMERA_POSITION),
                        "quaternion_wxyz": list(
                            OVERHEAD_CAMERA_USD_ORIENTATION
                        ),
                    }
                    if spec.name == OVERHEAD_CAMERA_NAME
                    else {
                        "parent_prim_path": str(
                            Path(CAMERA_PRIM_PATHS[spec.name]).parent
                        ),
                        "local_position_m": [0.0, 0.0, 0.0],
                        "local_quaternion_wxyz": list(
                            WRIST_CAMERA_LOCAL_USD_ORIENTATION
                        ),
                    }
                ),
                "initial_validation": (
                    {}
                    if camera_report is None
                    else camera_report.get(spec.name, {})
                ),
            }
            for spec in CAMERA_SPECS
        ],
        "task": {
            "sort_order_small_to_large": list(MATRYOSHKA_SORT_ORDER),
            "target_axis": "world_y",
            "target_line_x_m": 0.0,
            "target_line_center_y_m": TABLE_POSITION[1],
            "target_gap_m": MATRYOSHKA_TARGET_GAP,
            "position_tolerance_m": MATRYOSHKA_POSITION_TOLERANCE,
            "upright_tolerance_degrees": (
                MATRYOSHKA_UPRIGHT_TOLERANCE_DEGREES
            ),
            "robot_home_tolerance_rad": PIPER_HOME_TOLERANCE_RAD,
            "assignments": assignments,
        },
        "action_semantics": {
            "joint_action": (
                "position targets actually sent to the Isaac articulation "
                "controller at 30 Hz"
            ),
            "joint_action_order": list(PIPER_COMMAND_JOINT_NAMES),
            "grasp_event_codes": {
                str(code): name for code, name in GRASP_EVENT_NAMES.items()
            },
            "fixed_joint_relative_pose": (
                "body0/link6-to-object transform recorded with attach event"
            ),
            "replay_policy": (
                "apply recorded joint/gripper targets and recorded fixed-joint "
                "events only; never invoke cuRobo"
            ),
        },
    }


def _decode_hdf5_text(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return str(value)


class Hdf5EpisodeWriter:
    """Append synchronized control frames to one bounded-memory HDF5 file."""

    def __init__(
        self,
        path: Path,
        *,
        metadata: dict[str, Any],
        initial_state: dict[str, dict[str, Any]],
    ) -> None:
        import h5py
        import numpy as np

        self.path = path.resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = h5py.File(self.path, "w", libver="latest")
        self._datasets: dict[str, Any] = {}
        self.frame_count = 0
        self._capacity = 0
        self._closed = False

        root = self._file
        root.attrs["schema_version"] = EPISODE_SCHEMA_VERSION
        root.attrs["episode_id"] = metadata["episode"]["id"]
        root.attrs["created_at_utc"] = metadata["episode"][
            "created_at_utc"
        ]
        root.attrs["accepted"] = False
        root.attrs["expert_success"] = False
        root.attrs["replay_success"] = False
        root.attrs["failure_reason"] = ""
        root.attrs["frame_count"] = 0
        root.attrs["writer_state"] = "recording"

        text_dtype = h5py.string_dtype(encoding="utf-8")
        metadata_group = root.require_group("metadata")
        metadata_group.create_dataset(
            "json",
            data=json.dumps(metadata, sort_keys=True),
            dtype=text_dtype,
        )
        lookups = root.require_group("lookups")
        lookups.create_dataset(
            "robot_names",
            data=np.asarray(EPISODE_ROBOT_NAMES, dtype="S8"),
        )
        lookups.create_dataset(
            "object_ids",
            data=np.asarray(EPISODE_OBJECT_IDS, dtype="S5"),
        )
        lookups.create_dataset(
            "camera_names",
            data=np.asarray(EPISODE_CAMERA_NAMES, dtype="S32"),
        )
        lookups.create_dataset(
            "simulation_dof_names",
            data=np.asarray(PIPER_DOF_NAMES, dtype="S32"),
        )
        lookups.create_dataset(
            "command_joint_names",
            data=np.asarray(PIPER_COMMAND_JOINT_NAMES, dtype="S32"),
        )

        initial = root.require_group("initial")
        initial_robots = initial.require_group("robots")
        initial_objects = initial.require_group("objects")
        targets = root.require_group("targets")
        initial_robots.create_dataset(
            "joint_position",
            data=np.asarray(
                initial_state["robots"]["joint_position"],
                dtype=np.float32,
            ),
        )
        initial_robots.create_dataset(
            "joint_velocity",
            data=np.asarray(
                initial_state["robots"]["joint_velocity"],
                dtype=np.float32,
            ),
        )
        initial_robots.create_dataset(
            "joint_action",
            data=np.asarray(
                initial_state["robots"]["joint_action"],
                dtype=np.float32,
            ),
        )
        initial_objects.create_dataset(
            "pose",
            data=np.asarray(initial_state["objects"]["pose"], dtype=np.float32),
        )
        initial_objects.create_dataset(
            "linear_velocity",
            data=np.asarray(
                initial_state["objects"]["linear_velocity"],
                dtype=np.float32,
            ),
        )
        initial_objects.create_dataset(
            "angular_velocity",
            data=np.asarray(
                initial_state["objects"]["angular_velocity"],
                dtype=np.float32,
            ),
        )
        targets.create_dataset(
            "object_pose",
            data=np.asarray(
                initial_state["targets"]["object_pose"],
                dtype=np.float32,
            ),
        )
        expected_initial_shapes = {
            "initial/robots/joint_position": (
                len(EPISODE_ROBOT_NAMES),
                len(PIPER_DOF_NAMES),
            ),
            "initial/robots/joint_velocity": (
                len(EPISODE_ROBOT_NAMES),
                len(PIPER_DOF_NAMES),
            ),
            "initial/robots/joint_action": (
                len(EPISODE_ROBOT_NAMES),
                len(PIPER_COMMAND_JOINT_NAMES),
            ),
            "initial/objects/pose": (len(EPISODE_OBJECT_IDS), 7),
            "initial/objects/linear_velocity": (
                len(EPISODE_OBJECT_IDS),
                3,
            ),
            "initial/objects/angular_velocity": (
                len(EPISODE_OBJECT_IDS),
                3,
            ),
            "targets/object_pose": (len(EPISODE_OBJECT_IDS), 7),
        }
        for dataset_path, expected_shape in expected_initial_shapes.items():
            actual_shape = tuple(root[dataset_path].shape)
            if actual_shape != expected_shape:
                root.close()
                self._closed = True
                raise ValueError(
                    f"{dataset_path}: expected {expected_shape}, "
                    f"found {actual_shape}"
                )

        results = root.require_group("results")
        results.create_dataset("expert_summary_json", data="{}", dtype=text_dtype)
        results.create_dataset("replay_summary_json", data="{}", dtype=text_dtype)

        self._create_frame_dataset("frames/frame_index", (), np.int64)
        self._create_frame_dataset("frames/simulation_time_s", (), np.float64)
        self._create_frame_dataset("frames/world_time_s", (), np.float64)
        self._create_frame_dataset(
            "frames/robots/joint_position",
            (len(EPISODE_ROBOT_NAMES), len(PIPER_DOF_NAMES)),
            np.float32,
        )
        self._create_frame_dataset(
            "frames/robots/joint_velocity",
            (len(EPISODE_ROBOT_NAMES), len(PIPER_DOF_NAMES)),
            np.float32,
        )
        self._create_frame_dataset(
            "frames/robots/joint_action",
            (len(EPISODE_ROBOT_NAMES), len(PIPER_COMMAND_JOINT_NAMES)),
            np.float32,
        )
        self._create_frame_dataset(
            "frames/robots/arm_joint_action",
            (len(EPISODE_ROBOT_NAMES), len(PIPER_ARM_JOINT_NAMES)),
            np.float32,
        )
        self._create_frame_dataset(
            "frames/robots/gripper_position",
            (len(EPISODE_ROBOT_NAMES), len(PIPER_GRIPPER_JOINT_NAMES)),
            np.float32,
        )
        self._create_frame_dataset(
            "frames/robots/gripper_action",
            (len(EPISODE_ROBOT_NAMES),),
            np.float32,
        )
        self._create_frame_dataset(
            "frames/robots/end_effector_world_pose",
            (len(EPISODE_ROBOT_NAMES), 7),
            np.float32,
        )
        self._create_frame_dataset(
            "frames/objects/world_pose",
            (len(EPISODE_OBJECT_IDS), 7),
            np.float32,
        )
        self._create_frame_dataset(
            "frames/objects/linear_velocity",
            (len(EPISODE_OBJECT_IDS), 3),
            np.float32,
        )
        self._create_frame_dataset(
            "frames/objects/angular_velocity",
            (len(EPISODE_OBJECT_IDS), 3),
            np.float32,
        )
        self._create_frame_dataset("frames/task/phase", (), "S48")
        self._create_frame_dataset("frames/task/operator", (), "S8")
        self._create_frame_dataset("frames/task/object_id", (), "S5")
        self._create_frame_dataset(
            "frames/control/grasp_event_code",
            (len(EPISODE_ROBOT_NAMES),),
            np.int8,
        )
        self._create_frame_dataset(
            "frames/control/grasp_event_object_index",
            (len(EPISODE_ROBOT_NAMES),),
            np.int8,
        )
        self._create_frame_dataset(
            "frames/control/grasp_event_relative_pose",
            (len(EPISODE_ROBOT_NAMES), 7),
            np.float32,
        )
        width, height = CAMERA_RESOLUTION
        for camera_name in EPISODE_CAMERA_NAMES:
            base = f"frames/cameras/{camera_name}"
            self._create_frame_dataset(
                f"{base}/rgb",
                (height, width, 3),
                np.uint8,
                image=True,
            )
            self._create_frame_dataset(
                f"{base}/depth",
                (height, width),
                np.float32,
                image=True,
            )
            self._create_frame_dataset(
                f"{base}/rendering_time_s",
                (),
                np.float64,
            )
            self._create_frame_dataset(
                f"{base}/world_pose",
                (7,),
                np.float32,
            )
            self._create_frame_dataset(
                f"{base}/world_to_camera",
                (4, 4),
                np.float32,
            )
        root.flush()

    def _create_frame_dataset(
        self,
        path: str,
        tail_shape: tuple[int, ...],
        dtype: Any,
        *,
        image: bool = False,
    ) -> None:
        group_path, dataset_name = path.rsplit("/", 1)
        group = self._file.require_group(group_path)
        chunk_frames = 1 if image else EPISODE_DATASET_ALLOCATION_BLOCK
        chunks = (chunk_frames, *tail_shape)
        self._datasets[path] = group.create_dataset(
            dataset_name,
            shape=(0, *tail_shape),
            maxshape=(None, *tail_shape),
            chunks=chunks,
            dtype=dtype,
            compression=EPISODE_HDF5_COMPRESSION,
            compression_opts=EPISODE_HDF5_COMPRESSION_LEVEL,
            shuffle=True,
        )

    def _ensure_capacity(self) -> None:
        if self.frame_count < self._capacity:
            return
        self._capacity += EPISODE_DATASET_ALLOCATION_BLOCK
        for dataset in self._datasets.values():
            dataset.resize((self._capacity, *dataset.shape[1:]))

    def append_frame(self, frame: dict[str, Any]) -> None:
        """Append one already synchronized state/action/RGB-D sample."""

        import numpy as np

        if self._closed:
            raise RuntimeError("Cannot append to a closed episode")
        self._ensure_capacity()
        index = self.frame_count
        robots = frame["robots"]
        objects = frame["objects"]
        task = frame["task"]
        control = frame["control"]
        values: dict[str, Any] = {
            "frames/frame_index": index,
            "frames/simulation_time_s": float(frame["simulation_time_s"]),
            "frames/world_time_s": float(frame["world_time_s"]),
            "frames/robots/joint_position": robots["joint_position"],
            "frames/robots/joint_velocity": robots["joint_velocity"],
            "frames/robots/joint_action": robots["joint_action"],
            "frames/robots/arm_joint_action": np.asarray(
                robots["joint_action"]
            )[:, : len(PIPER_ARM_JOINT_NAMES)],
            "frames/robots/gripper_position": np.asarray(
                robots["joint_position"]
            )[:, 6:8],
            "frames/robots/gripper_action": np.asarray(
                robots["joint_action"]
            )[:, 6],
            "frames/robots/end_effector_world_pose": robots[
                "end_effector_world_pose"
            ],
            "frames/objects/world_pose": objects["world_pose"],
            "frames/objects/linear_velocity": objects["linear_velocity"],
            "frames/objects/angular_velocity": objects["angular_velocity"],
            "frames/task/phase": str(task["phase"]).encode("utf-8"),
            "frames/task/operator": str(task["operator"]).encode("utf-8"),
            "frames/task/object_id": str(task["object_id"]).encode("utf-8"),
            "frames/control/grasp_event_code": control[
                "grasp_event_code"
            ],
            "frames/control/grasp_event_object_index": control[
                "grasp_event_object_index"
            ],
            "frames/control/grasp_event_relative_pose": control[
                "grasp_event_relative_pose"
            ],
        }
        for camera_name in EPISODE_CAMERA_NAMES:
            camera = frame["cameras"][camera_name]
            rgb = np.asarray(camera["rgb"])
            depth = np.asarray(camera["depth"])
            expected_rgb_shape = (
                CAMERA_RESOLUTION[1],
                CAMERA_RESOLUTION[0],
                3,
            )
            expected_depth_shape = (
                CAMERA_RESOLUTION[1],
                CAMERA_RESOLUTION[0],
            )
            if rgb.shape != expected_rgb_shape or rgb.dtype != np.uint8:
                raise ValueError(
                    f"{camera_name}: invalid frame RGB {rgb.shape}/{rgb.dtype}"
                )
            if depth.shape != expected_depth_shape or depth.dtype != np.float32:
                raise ValueError(
                    f"{camera_name}: invalid frame depth "
                    f"{depth.shape}/{depth.dtype}"
                )
            base = f"frames/cameras/{camera_name}"
            values[f"{base}/rgb"] = rgb
            values[f"{base}/depth"] = depth
            values[f"{base}/rendering_time_s"] = float(
                camera["rendering_time_s"]
            )
            values[f"{base}/world_pose"] = camera["world_pose"]
            values[f"{base}/world_to_camera"] = camera["world_to_camera"]
        if set(values) != set(self._datasets):
            missing = sorted(set(self._datasets) - set(values))
            extra = sorted(set(values) - set(self._datasets))
            raise ValueError(
                f"Episode frame fields changed: missing={missing}, extra={extra}"
            )
        for path, value in values.items():
            self._datasets[path][index] = value
        self.frame_count += 1
        if self.frame_count % EPISODE_DATASET_ALLOCATION_BLOCK == 0:
            self._file.attrs["frame_count"] = self.frame_count
            self._file.flush()

    def finish_expert(
        self,
        *,
        success: bool,
        summary: dict[str, Any] | None = None,
        failure_reason: str = "",
    ) -> None:
        if self._closed:
            raise RuntimeError("Episode writer is already closed")
        if success and failure_reason:
            raise ValueError("A successful expert episode cannot have a failure")
        self._file.attrs["expert_success"] = bool(success)
        self._file.attrs["replay_success"] = False
        self._file.attrs["accepted"] = False
        self._file.attrs["failure_reason"] = failure_reason
        self._file.attrs["writer_state"] = (
            "expert_complete" if success else "expert_failed"
        )
        self._file["results/expert_summary_json"][()] = json.dumps(
            {} if summary is None else summary,
            sort_keys=True,
        )

    def close(self) -> None:
        if self._closed:
            return
        for dataset in self._datasets.values():
            dataset.resize((self.frame_count, *dataset.shape[1:]))
        self._file.attrs["frame_count"] = self.frame_count
        self._file.flush()
        self._file.close()
        self._closed = True


def validate_episode_hdf5(
    path: Path,
    *,
    require_accepted: bool = False,
) -> dict[str, Any]:
    """Validate schema, dtypes, shapes, synchronization, and acceptance flags."""

    import h5py
    import numpy as np

    required_metadata_keys = {
        "episode",
        "versions",
        "seeds",
        "assets",
        "coordinates",
        "timing",
        "scene",
        "robots",
        "objects",
        "cameras",
        "task",
        "action_semantics",
    }
    required_datasets = {
        "metadata/json",
        "lookups/robot_names",
        "lookups/object_ids",
        "lookups/camera_names",
        "initial/robots/joint_position",
        "initial/robots/joint_velocity",
        "initial/robots/joint_action",
        "initial/objects/pose",
        "initial/objects/linear_velocity",
        "initial/objects/angular_velocity",
        "targets/object_pose",
        "frames/frame_index",
        "frames/simulation_time_s",
        "frames/world_time_s",
        "frames/robots/joint_position",
        "frames/robots/joint_velocity",
        "frames/robots/joint_action",
        "frames/robots/arm_joint_action",
        "frames/robots/gripper_position",
        "frames/robots/gripper_action",
        "frames/robots/end_effector_world_pose",
        "frames/objects/world_pose",
        "frames/objects/linear_velocity",
        "frames/objects/angular_velocity",
        "frames/task/phase",
        "frames/task/operator",
        "frames/task/object_id",
        "frames/control/grasp_event_code",
        "frames/control/grasp_event_object_index",
        "frames/control/grasp_event_relative_pose",
        "results/expert_summary_json",
        "results/replay_summary_json",
    }
    for camera_name in EPISODE_CAMERA_NAMES:
        for field in (
            "rgb",
            "depth",
            "rendering_time_s",
            "world_pose",
            "world_to_camera",
        ):
            required_datasets.add(
                f"frames/cameras/{camera_name}/{field}"
            )

    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"Episode does not exist: {resolved}")
    with h5py.File(resolved, "r") as episode:
        if _decode_hdf5_text(
            episode.attrs.get("schema_version", "")
        ) != EPISODE_SCHEMA_VERSION:
            raise ValueError("Episode schema_version is missing or unsupported")
        missing = sorted(
            dataset_path
            for dataset_path in required_datasets
            if dataset_path not in episode
        )
        if missing:
            raise ValueError(f"Episode is missing datasets: {missing}")
        metadata = json.loads(
            _decode_hdf5_text(episode["metadata/json"][()])
        )
        if not required_metadata_keys.issubset(metadata):
            raise ValueError(
                "Episode metadata is incomplete: "
                f"{sorted(required_metadata_keys - set(metadata))}"
            )
        frame_count = int(episode.attrs.get("frame_count", -1))
        if frame_count <= 0:
            raise ValueError(f"Episode has invalid frame_count={frame_count}")
        frame_paths = [
            dataset_path
            for dataset_path in required_datasets
            if dataset_path.startswith("frames/")
        ]
        unsynchronized = {
            dataset_path: int(episode[dataset_path].shape[0])
            for dataset_path in frame_paths
            if episode[dataset_path].shape[0] != frame_count
        }
        if unsynchronized:
            raise ValueError(
                f"Episode frame dimensions are not synchronized: {unsynchronized}"
            )
        expected_indices = np.arange(frame_count, dtype=np.int64)
        if not np.array_equal(
            episode["frames/frame_index"][:], expected_indices
        ):
            raise ValueError("Episode frame_index is not contiguous from zero")
        timestamps = np.asarray(
            episode["frames/simulation_time_s"][:],
            dtype=np.float64,
        )
        if frame_count > 1 and not np.allclose(
            np.diff(timestamps),
            CONTROL_DT,
            atol=1.0e-9,
            rtol=0.0,
        ):
            raise ValueError("Episode simulation timestamps are not 30 Hz")
        expected_shapes = {
            "initial/robots/joint_position": (
                len(EPISODE_ROBOT_NAMES),
                len(PIPER_DOF_NAMES),
            ),
            "initial/robots/joint_velocity": (
                len(EPISODE_ROBOT_NAMES),
                len(PIPER_DOF_NAMES),
            ),
            "initial/robots/joint_action": (
                len(EPISODE_ROBOT_NAMES),
                len(PIPER_COMMAND_JOINT_NAMES),
            ),
            "initial/objects/pose": (len(EPISODE_OBJECT_IDS), 7),
            "initial/objects/linear_velocity": (
                len(EPISODE_OBJECT_IDS),
                3,
            ),
            "initial/objects/angular_velocity": (
                len(EPISODE_OBJECT_IDS),
                3,
            ),
            "targets/object_pose": (len(EPISODE_OBJECT_IDS), 7),
            "frames/robots/joint_position": (
                frame_count,
                len(EPISODE_ROBOT_NAMES),
                len(PIPER_DOF_NAMES),
            ),
            "frames/robots/joint_action": (
                frame_count,
                len(EPISODE_ROBOT_NAMES),
                len(PIPER_COMMAND_JOINT_NAMES),
            ),
            "frames/objects/world_pose": (
                frame_count,
                len(EPISODE_OBJECT_IDS),
                7,
            ),
        }
        for dataset_path, expected_shape in expected_shapes.items():
            if tuple(episode[dataset_path].shape) != expected_shape:
                raise ValueError(
                    f"{dataset_path}: expected {expected_shape}, found "
                    f"{episode[dataset_path].shape}"
                )
        camera_report: dict[str, Any] = {}
        for camera_name in EPISODE_CAMERA_NAMES:
            base = f"frames/cameras/{camera_name}"
            rgb = episode[f"{base}/rgb"]
            depth = episode[f"{base}/depth"]
            expected_rgb = (
                frame_count,
                CAMERA_RESOLUTION[1],
                CAMERA_RESOLUTION[0],
                3,
            )
            expected_depth = (
                frame_count,
                CAMERA_RESOLUTION[1],
                CAMERA_RESOLUTION[0],
            )
            if tuple(rgb.shape) != expected_rgb or rgb.dtype != np.uint8:
                raise ValueError(
                    f"{camera_name}: invalid RGB schema "
                    f"{rgb.shape}/{rgb.dtype}"
                )
            if tuple(depth.shape) != expected_depth or depth.dtype != np.float32:
                raise ValueError(
                    f"{camera_name}: invalid depth schema "
                    f"{depth.shape}/{depth.dtype}"
                )
            camera_report[camera_name] = {
                "rgb_shape": list(rgb.shape),
                "rgb_dtype": str(rgb.dtype),
                "depth_shape": list(depth.shape),
                "depth_dtype": str(depth.dtype),
                "frequency_hz": CAMERA_FREQUENCY_HZ,
                "depth_unit": CAMERA_DEPTH_UNIT,
            }
        event_codes = np.asarray(
            episode["frames/control/grasp_event_code"][:],
            dtype=np.int8,
        )
        if not set(np.unique(event_codes).tolist()).issubset(
            set(GRASP_EVENT_NAMES)
        ):
            raise ValueError("Episode contains an unknown grasp event code")
        expert_success = bool(episode.attrs.get("expert_success", False))
        replay_success = bool(episode.attrs.get("replay_success", False))
        accepted = bool(episode.attrs.get("accepted", False))
        failure_reason = _decode_hdf5_text(
            episode.attrs.get("failure_reason", "")
        )
        if accepted and not (expert_success and replay_success):
            raise ValueError(
                "accepted=true requires expert_success and replay_success"
            )
        if accepted and failure_reason:
            raise ValueError("An accepted episode cannot have a failure reason")
        if require_accepted and not accepted:
            raise ValueError("Episode has not passed replay acceptance")
        return {
            "path": str(resolved),
            "schema_version": EPISODE_SCHEMA_VERSION,
            "episode_id": _decode_hdf5_text(
                episode.attrs["episode_id"]
            ),
            "frame_count": frame_count,
            "duration_s": float(timestamps[-1]),
            "expert_success": expert_success,
            "replay_success": replay_success,
            "accepted": accepted,
            "failure_reason": failure_reason,
            "writer_state": _decode_hdf5_text(
                episode.attrs.get("writer_state", "")
            ),
            "camera_streams": camera_report,
            "event_counts": {
                name: int(np.count_nonzero(event_codes == code))
                for code, name in GRASP_EVENT_NAMES.items()
            },
        }

_EXPECTED_DOLL_HEIGHTS: Final = (0.13, 0.11, 0.09, 0.07, 0.05)
_EXPECTED_USD_DEFAULT_PRIMS: Final = {
    str(PIPER_USD): "/Piper",
    str(ROOM_USD): "/World",
    str(CAMERA_STAND_USD): "/root",
    **{
        str(MATRYOSHKA_ROOT / f"{index:05d}/object.usdz"): "/root"
        for index in range(5)
    },
}
_EXPECTED_PIPER_USD_JOINTS: Final = {
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper_joint",
    "joint8",
    "root_joint",
}


def _read_doll_specs() -> tuple[DollSpec, ...]:
    specs: list[DollSpec] = []
    for index, expected_height in enumerate(_EXPECTED_DOLL_HEIGHTS):
        asset_id = f"{index:05d}"
        directory = MATRYOSHKA_ROOT / asset_id
        metadata_path = directory / "metadata.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        size = tuple(float(value) for value in metadata["physics"]["size"])
        if len(size) != 3:
            raise ValueError(f"{metadata_path}: physics.size must contain three values")
        height = size[2]
        if not math.isclose(height, expected_height, abs_tol=1.0e-9):
            raise ValueError(
                f"{metadata_path}: expected height {expected_height}, found {height}"
            )
        specs.append(
            DollSpec(
                asset_id=asset_id,
                asset_path=directory / "object.usdz",
                metadata_path=metadata_path,
                height=height,
                size=size,  # type: ignore[arg-type]
                footprint_radius=max(size[0], size[1]) / 2.0,
                mass=float(metadata["physics"]["mass"]),
                friction=float(metadata["physics"]["friction"]),
                uuid=str(metadata["uuid"]),
            )
        )
    return tuple(specs)


def get_doll_specs() -> tuple[DollSpec, ...]:
    """Load the five allowed doll specifications from local metadata."""

    return _read_doll_specs()


def compute_doll_target_layout() -> tuple[DollPlacement, ...]:
    """Compute the size-aware, Y-axis target line centred on the table."""

    specs_by_id = {spec.asset_id: spec for spec in get_doll_specs()}
    ordered_specs = [specs_by_id[asset_id] for asset_id in MATRYOSHKA_SORT_ORDER]
    center_distances = [
        first.footprint_radius
        + second.footprint_radius
        + MATRYOSHKA_TARGET_GAP
        for first, second in zip(ordered_specs, ordered_specs[1:])
    ]
    occupied_length = (
        ordered_specs[0].footprint_radius
        + sum(center_distances)
        + ordered_specs[-1].footprint_radius
    )
    table_center_y = TABLE_POSITION[1]
    current_y = (
        table_center_y
        - occupied_length / 2.0
        + ordered_specs[0].footprint_radius
    )
    placements: list[DollPlacement] = []
    for index, spec in enumerate(ordered_specs):
        placements.append(
            DollPlacement(
                asset_id=spec.asset_id,
                pose=PoseSpec(
                    (
                        0.0,
                        current_y,
                        TABLE_TOP_Z + spec.height / 2.0,
                    ),
                    IDENTITY_QUATERNION,
                ),
                yaw_rad=0.0,
            )
        )
        if index < len(center_distances):
            current_y += center_distances[index]
    return tuple(placements)


def build_curobo_robot_config() -> dict[str, Any]:
    """Build the in-memory six-DOF Piper model used by cuRobo."""

    collision_links = (
        "link1",
        "link2",
        "link3",
        "link4",
        "link5",
        "link6",
        "link7",
        "link8",
        CUROBO_ATTACHED_OBJECT_LINK,
    )
    collision_spheres = {
        "link1": [{"center": [0.0, 0.0, 0.0], "radius": 0.045}],
        "link2": [
            {"center": [x, 0.0, 0.0], "radius": 0.052}
            for x in (0.020, 0.085, 0.150, 0.215, 0.280)
        ],
        "link3": [
            {"center": [0.0, y, 0.0], "radius": 0.046}
            for y in (0.0, -0.055, -0.110, -0.165, -0.210)
        ],
        "link4": [{"center": [0.0, 0.0, 0.0], "radius": 0.043}],
        "link5": [
            {"center": [0.0, y, 0.0], "radius": 0.040}
            for y in (0.0, -0.050, -0.090)
        ],
        "link6": [
            {"center": [-0.060, 0.0, 0.027], "radius": 0.045},
            {"center": [-0.005, 0.0, 0.035], "radius": 0.045},
            {"center": [-0.020, -0.060, 0.058], "radius": 0.030},
            {"center": [-0.020, 0.060, 0.058], "radius": 0.030},
        ],
        "link7": [
            {"center": [0.0, -0.013, -0.020], "radius": 0.028},
            {"center": [0.0, -0.013, -0.058], "radius": 0.028},
        ],
        "link8": [
            {"center": [0.0, 0.013, -0.020], "radius": 0.028},
            {"center": [0.0, 0.013, -0.058], "radius": 0.028},
        ],
    }
    self_collision_ignore = {
        # The retracted zero pose folds link5 alongside link1/link2.  The
        # conservative spheres overlap there even though the authored meshes
        # retain clearance, so those measured home-neighbour pairs are ignored.
        "link1": ["link2", "link3", "link4", "link5"],
        "link2": ["link3", "link4", "link5"],
        "link3": ["link4", "link5"],
        "link4": ["link5", "link6"],
        "link5": ["link6", "link7", "link8"],
        "link6": ["link7", "link8", CUROBO_ATTACHED_OBJECT_LINK],
        "link7": ["link8", CUROBO_ATTACHED_OBJECT_LINK],
        "link8": [CUROBO_ATTACHED_OBJECT_LINK],
    }
    return {
        "robot_cfg": {
            "kinematics": {
                "format_version": 2.0,
                "urdf_path": str(PIPER_URDF),
                "asset_root_path": str(PIPER_URDF.parents[2]),
                "base_link": PIPER_BASE_LINK,
                "tool_frames": [PIPER_TOOL_LINK],
                "collision_link_names": list(collision_links),
                "collision_sphere_buffer": 0.0,
                "collision_spheres": collision_spheres,
                "self_collision_buffer": {
                    link_name: 0.0 for link_name in collision_links
                },
                "self_collision_ignore": self_collision_ignore,
                "grasp_contact_link_names": [
                    "link6",
                    "link7",
                    "link8",
                    CUROBO_ATTACHED_OBJECT_LINK,
                ],
                "lock_joints": {
                    "gripper_joint": PIPER_OPEN_GRIPPER_POSITION,
                },
                "extra_collision_spheres": {
                    CUROBO_ATTACHED_OBJECT_LINK: CUROBO_ATTACHED_OBJECT_SPHERES,
                },
                "extra_links": {
                    PIPER_TOOL_LINK: {
                        "link_name": PIPER_TOOL_LINK,
                        "parent_link_name": PIPER_URDF_TOOL_LINK,
                        "joint_name": "finger_center_fixed",
                        "joint_type": "FIXED",
                        "fixed_transform": [
                            *PIPER_FINGER_CENTER_OFFSET_IN_TOOL_M,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                    },
                    CUROBO_ATTACHED_OBJECT_LINK: {
                        "link_name": CUROBO_ATTACHED_OBJECT_LINK,
                        "parent_link_name": PIPER_TOOL_LINK,
                        "joint_name": "attached_object_fixed",
                        "joint_type": "FIXED",
                        "fixed_transform": [
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                        ],
                    }
                },
                "cspace": {
                    "joint_names": list(PIPER_ARM_JOINT_NAMES),
                    "default_joint_position": list(PIPER_HOME_JOINT_POSITION),
                    "cspace_distance_weight": [1.0, 1.0, 1.0, 0.8, 0.8, 0.6],
                    "null_space_weight": [1.0, 1.0, 1.0, 0.5, 0.5, 0.4],
                    "max_acceleration": [4.0, 4.0, 4.0, 6.0, 6.0, 6.0],
                    "max_jerk": [80.0] * 6,
                    "velocity_scale": [0.35] * 6,
                    "acceleration_scale": [0.40] * 6,
                },
            }
        }
    }


def build_curobo_planner(seed: int) -> Any:
    """Create an actual GPU cuRobo planner from the in-memory Piper config."""

    import torch
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    from curobo.types import DeviceCfg

    planner_config = MotionPlannerCfg.create(
        robot=build_curobo_robot_config(),
        collision_cache=dict(CUROBO_COLLISION_CACHE),
        self_collision_check=True,
        device_cfg=DeviceCfg(device=CUROBO_DEVICE, dtype=torch.float32),
        num_ik_seeds=CUROBO_NUM_IK_SEEDS,
        num_trajopt_seeds=CUROBO_NUM_TRAJOPT_SEEDS,
        position_tolerance=CUROBO_POSITION_TOLERANCE_M,
        orientation_tolerance=CUROBO_ORIENTATION_TOLERANCE_RAD,
        use_cuda_graph=False,
        random_seed=seed,
        optimizer_collision_activation_distance=(
            CUROBO_COLLISION_ACTIVATION_DISTANCE_M
        ),
        interpolation_dt=CONTROL_DT,
        interpolation_buffer_size=CUROBO_MAX_TRAJECTORY_STEPS,
    )
    planner = MotionPlanner(planner_config)
    if planner.joint_names != list(PIPER_ARM_JOINT_NAMES):
        planner.destroy()
        raise ValueError(
            f"cuRobo joint order {planner.joint_names} does not match "
            f"{PIPER_ARM_JOINT_NAMES}"
        )
    if planner.tool_frames != [PIPER_TOOL_LINK]:
        planner.destroy()
        raise ValueError(
            f"cuRobo tool frames {planner.tool_frames} do not contain "
            f"{PIPER_TOOL_LINK}"
        )
    return planner


def _robot_spec_by_name(name: str) -> RobotSpec:
    """Resolve a serialized robot name without accepting arbitrary poses."""

    for spec in ROBOT_SPECS:
        if spec.name == name:
            return spec
    raise ValueError(f"Unknown Piper name {name!r}")


def _pose_spec_from_mapping(value: dict[str, Any]) -> PoseSpec:
    """Decode one JSON pose while retaining the program's pose convention."""

    position = value["position"]
    quaternion = value["quaternion"]
    if len(position) != 3 or len(quaternion) != 4:
        raise ValueError(f"Invalid serialized pose: {value!r}")
    return PoseSpec(
        tuple(float(component) for component in position),  # type: ignore[arg-type]
        normalize_quaternion(quaternion),
    )


def _curobo_report_to_json(report: dict[str, Any]) -> dict[str, Any]:
    """Convert the one ndarray in a planner report to plain JSON values."""

    converted = dict(report)
    converted["joint_position"] = report["joint_position"].tolist()
    return converted


def attach_curobo_doll(
    planner: Any,
    robot: RobotSpec,
    current_joint_position: Sequence[float],
    asset_id: str,
    doll_world_pose: PoseSpec,
) -> dict[str, Any]:
    """Attach a conservative doll sphere model to cuRobo's tool link."""

    import torch
    from curobo.types import JointState, Pose

    specs_by_id = {spec.asset_id: spec for spec in get_doll_specs()}
    spec = specs_by_id[asset_id]
    sphere_count = min(
        CUROBO_ATTACHED_OBJECT_SPHERES,
        max(1, math.ceil(spec.height / (2.0 * spec.footprint_radius))),
    )
    if sphere_count == 1:
        sphere_z = [0.0]
    else:
        half_span = max(0.0, spec.height / 2.0 - spec.footprint_radius)
        sphere_z = [
            -half_span + 2.0 * half_span * index / (sphere_count - 1)
            for index in range(sphere_count)
        ]
    collision_radius = max(
        0.005,
        spec.footprint_radius - CUROBO_ATTACHED_OBJECT_INSET_M,
    )
    sphere_tensor = torch.as_tensor(
        [
            [0.0, 0.0, center_z, collision_radius]
            for center_z in sphere_z
        ],
        device=CUROBO_DEVICE,
        dtype=torch.float32,
    )
    current_state = JointState.from_position(
        torch.as_tensor(
            [[float(value) for value in current_joint_position]],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
        joint_names=list(PIPER_ARM_JOINT_NAMES),
    )
    base_object_pose = world_pose_to_robot_base(robot, doll_world_pose)
    world_objects_pose_offset = Pose(
        position=torch.as_tensor(
            [base_object_pose.position],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
        quaternion=torch.as_tensor(
            [base_object_pose.quaternion],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
    )
    planner.attachment_manager.update(
        sphere_tensor,
        current_state,
        link_name=CUROBO_ATTACHED_OBJECT_LINK,
        world_objects_pose_offset=world_objects_pose_offset,
    )
    return {
        "asset_id": asset_id,
        "sphere_count": sphere_count,
        "physical_footprint_radius_m": spec.footprint_radius,
        "collision_sphere_radius_m": collision_radius,
        "collision_inset_m": CUROBO_ATTACHED_OBJECT_INSET_M,
        "sphere_centers_object_m": [
            [0.0, 0.0, center_z] for center_z in sphere_z
        ],
        "world_pose": asdict(doll_world_pose),
        "base_pose": asdict(base_object_pose),
    }


def run_curobo_planner_worker(seed: int) -> int:
    """Serve cuRobo planning requests in a Warp-isolated child process.

    Isaac Sim 5.1 loads its bundled Warp 1.8 module into the simulation
    process.  The installed cuRobo build uses Warp 1.15 APIs, so importing
    both packages in one interpreter is unsafe.  This mode starts before any
    Isaac import and owns one persistent GPU planner.
    """

    protocol_stdout = sys.stdout

    def respond(payload: dict[str, Any]) -> None:
        print(
            CUROBO_WORKER_RESPONSE_PREFIX
            + json.dumps(payload, separators=(",", ":"), sort_keys=True),
            file=protocol_stdout,
            flush=True,
        )

    planner: Any | None = None
    try:
        planner = build_curobo_planner(seed)
        respond({"ok": True, "command": "ready", "seed": seed})
        for raw_line in sys.stdin:
            try:
                request = json.loads(raw_line)
                command = request["command"]
                if command == "shutdown":
                    respond({"ok": True, "command": command})
                    return 0
                active_robot = _robot_spec_by_name(request["active_robot"])
                if command == "attach":
                    report = attach_curobo_doll(
                        planner,
                        active_robot,
                        request["current_joint_position"],
                        request["asset_id"],
                        _pose_spec_from_mapping(request["doll_world_pose"]),
                    )
                    respond(
                        {"ok": True, "command": command, "report": report}
                    )
                    continue
                if command == "detach":
                    planner.attachment_manager.detach(
                        link_name=CUROBO_ATTACHED_OBJECT_LINK
                    )
                    respond({"ok": True, "command": command})
                    continue
                other_robot = _robot_spec_by_name(request["other_robot"])
                if active_robot.name == other_robot.name:
                    raise ValueError("Active and other Piper must differ")
                doll_poses = {
                    asset_id: _pose_spec_from_mapping(pose)
                    for asset_id, pose in request.get("doll_poses", {}).items()
                }
                scene = build_curobo_scene(
                    planner,
                    active_robot,
                    other_robot,
                    request["other_joint_position"],
                    doll_poses,
                    request.get("excluded_doll_ids", ()),
                )
                planner.update_world(scene)
                if command == "plan_position":
                    report = plan_curobo_to_position(
                        planner,
                        active_robot,
                        request["current_joint_position"],
                        request["world_goal_position"],
                        prefer_tool_x_down=bool(
                            request.get("prefer_tool_x_down", True)
                        ),
                        preferred_world_orientation=request.get(
                            "preferred_world_orientation"
                        ),
                        tool_to_attached_object_orientation=request.get(
                            "tool_to_attached_object_orientation"
                        ),
                        max_attached_object_tilt_degrees=request.get(
                            "max_attached_object_tilt_degrees"
                        ),
                    )
                elif command == "plan_pose":
                    report = plan_curobo_pose(
                        planner,
                        active_robot,
                        request["current_joint_position"],
                        _pose_spec_from_mapping(request["world_goal"]),
                    )
                elif command == "check_pose":
                    report = check_curobo_pose_ik(
                        planner,
                        active_robot,
                        request["current_joint_position"],
                        _pose_spec_from_mapping(request["world_goal"]),
                    )
                elif command == "plan_joint":
                    report = plan_curobo_joint_goal(
                        planner,
                        active_robot,
                        request["current_joint_position"],
                        request["goal_joint_position"],
                    )
                else:
                    raise ValueError(f"Unknown cuRobo worker command {command!r}")
                respond(
                    {
                        "ok": True,
                        "command": command,
                        "report": _curobo_report_to_json(report),
                    }
                )
            except Exception as error:
                respond(
                    {
                        "ok": False,
                        "error_type": type(error).__name__,
                        "error": str(error),
                    }
                )
    except Exception as error:
        respond(
            {
                "ok": False,
                "command": "ready",
                "error_type": type(error).__name__,
                "error": str(error),
            }
        )
        return 1
    finally:
        if planner is not None:
            planner.destroy()
    return 0


class CuroboPlannerWorker:
    """Small synchronous client for the isolated persistent planner."""

    def __init__(self, seed: int) -> None:
        self.seed = int(seed)
        self._process = subprocess.Popen(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                "--mode",
                "planner-worker",
                "--seed",
                str(self.seed),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self._diagnostic_lines: list[str] = []
        ready = self._read_response()
        if not ready.get("ok") or ready.get("command") != "ready":
            self.close()
            raise RuntimeError(f"cuRobo worker startup failed: {ready}")

    @property
    def diagnostic_lines(self) -> tuple[str, ...]:
        return tuple(self._diagnostic_lines)

    def _read_response(self) -> dict[str, Any]:
        stdout = self._process.stdout
        if stdout is None:
            raise RuntimeError("cuRobo worker stdout is unavailable")
        selector = selectors.DefaultSelector()
        selector.register(stdout, selectors.EVENT_READ)
        try:
            while True:
                if not selector.select(CUROBO_WORKER_TIMEOUT_S):
                    raise TimeoutError(
                        "cuRobo worker exceeded "
                        f"{CUROBO_WORKER_TIMEOUT_S:.1f} s"
                    )
                line = stdout.readline()
                if not line:
                    return_code = self._process.poll()
                    raise RuntimeError(
                        "cuRobo worker exited before responding "
                        f"(return code {return_code}); tail="
                        f"{self._diagnostic_lines[-20:]}"
                    )
                stripped = line.rstrip()
                if stripped.startswith(CUROBO_WORKER_RESPONSE_PREFIX):
                    return json.loads(
                        stripped[len(CUROBO_WORKER_RESPONSE_PREFIX) :]
                    )
                self._diagnostic_lines.append(stripped)
                del self._diagnostic_lines[:-200]
        finally:
            selector.close()

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        stdin = self._process.stdin
        if stdin is None or self._process.poll() is not None:
            raise RuntimeError("cuRobo worker is not running")
        stdin.write(json.dumps(payload, separators=(",", ":")) + "\n")
        stdin.flush()
        response = self._read_response()
        if not response.get("ok"):
            raise RuntimeError(
                "cuRobo worker planning failed: "
                f"{response.get('error_type')}: {response.get('error')}"
            )
        return response

    def plan_position(
        self,
        *,
        active_robot: RobotSpec,
        other_robot: RobotSpec,
        current_joint_position: Sequence[float],
        other_joint_position: Sequence[float],
        world_goal_position: Sequence[float],
        doll_poses: dict[str, PoseSpec] | None = None,
        excluded_doll_ids: Sequence[str] = (),
        prefer_tool_x_down: bool = True,
        preferred_world_orientation: Sequence[float] | None = None,
        tool_to_attached_object_orientation: (
            Sequence[float] | None
        ) = None,
        max_attached_object_tilt_degrees: float | None = None,
    ) -> dict[str, Any]:
        response = self.request(
            {
                "command": "plan_position",
                "active_robot": active_robot.name,
                "other_robot": other_robot.name,
                "current_joint_position": [
                    float(value) for value in current_joint_position
                ],
                "other_joint_position": [
                    float(value) for value in other_joint_position
                ],
                "world_goal_position": [
                    float(value) for value in world_goal_position
                ],
                "doll_poses": {
                    asset_id: asdict(pose)
                    for asset_id, pose in (doll_poses or {}).items()
                },
                "excluded_doll_ids": list(excluded_doll_ids),
                "prefer_tool_x_down": prefer_tool_x_down,
                "preferred_world_orientation": (
                    None
                    if preferred_world_orientation is None
                    else [
                        float(value)
                        for value in preferred_world_orientation
                    ]
                ),
                "tool_to_attached_object_orientation": (
                    None
                    if tool_to_attached_object_orientation is None
                    else [
                        float(value)
                        for value in tool_to_attached_object_orientation
                    ]
                ),
                "max_attached_object_tilt_degrees": (
                    None
                    if max_attached_object_tilt_degrees is None
                    else float(max_attached_object_tilt_degrees)
                ),
            }
        )
        return response["report"]

    def plan_joint(
        self,
        *,
        active_robot: RobotSpec,
        other_robot: RobotSpec,
        current_joint_position: Sequence[float],
        other_joint_position: Sequence[float],
        goal_joint_position: Sequence[float],
        doll_poses: dict[str, PoseSpec] | None = None,
        excluded_doll_ids: Sequence[str] = (),
    ) -> dict[str, Any]:
        response = self.request(
            {
                "command": "plan_joint",
                "active_robot": active_robot.name,
                "other_robot": other_robot.name,
                "current_joint_position": [
                    float(value) for value in current_joint_position
                ],
                "other_joint_position": [
                    float(value) for value in other_joint_position
                ],
                "goal_joint_position": [
                    float(value) for value in goal_joint_position
                ],
                "doll_poses": {
                    asset_id: asdict(pose)
                    for asset_id, pose in (doll_poses or {}).items()
                },
                "excluded_doll_ids": list(excluded_doll_ids),
            }
        )
        return response["report"]

    def plan_pose(
        self,
        *,
        active_robot: RobotSpec,
        other_robot: RobotSpec,
        current_joint_position: Sequence[float],
        other_joint_position: Sequence[float],
        world_goal: PoseSpec,
        doll_poses: dict[str, PoseSpec] | None = None,
        excluded_doll_ids: Sequence[str] = (),
    ) -> dict[str, Any]:
        response = self.request(
            {
                "command": "plan_pose",
                "active_robot": active_robot.name,
                "other_robot": other_robot.name,
                "current_joint_position": [
                    float(value) for value in current_joint_position
                ],
                "other_joint_position": [
                    float(value) for value in other_joint_position
                ],
                "world_goal": asdict(world_goal),
                "doll_poses": {
                    asset_id: asdict(pose)
                    for asset_id, pose in (doll_poses or {}).items()
                },
                "excluded_doll_ids": list(excluded_doll_ids),
            }
        )
        return response["report"]

    def check_pose(
        self,
        *,
        active_robot: RobotSpec,
        other_robot: RobotSpec,
        current_joint_position: Sequence[float],
        other_joint_position: Sequence[float],
        world_goal: PoseSpec,
        doll_poses: dict[str, PoseSpec] | None = None,
        excluded_doll_ids: Sequence[str] = (),
    ) -> dict[str, Any]:
        response = self.request(
            {
                "command": "check_pose",
                "active_robot": active_robot.name,
                "other_robot": other_robot.name,
                "current_joint_position": [
                    float(value) for value in current_joint_position
                ],
                "other_joint_position": [
                    float(value) for value in other_joint_position
                ],
                "world_goal": asdict(world_goal),
                "doll_poses": {
                    asset_id: asdict(pose)
                    for asset_id, pose in (doll_poses or {}).items()
                },
                "excluded_doll_ids": list(excluded_doll_ids),
            }
        )
        return response["report"]

    def attach(
        self,
        *,
        active_robot: RobotSpec,
        current_joint_position: Sequence[float],
        asset_id: str,
        doll_world_pose: PoseSpec,
    ) -> dict[str, Any]:
        response = self.request(
            {
                "command": "attach",
                "active_robot": active_robot.name,
                "current_joint_position": [
                    float(value) for value in current_joint_position
                ],
                "asset_id": asset_id,
                "doll_world_pose": asdict(doll_world_pose),
            }
        )
        return response["report"]

    def detach(self, *, active_robot: RobotSpec) -> None:
        self.request(
            {
                "command": "detach",
                "active_robot": active_robot.name,
            }
        )

    def close(self) -> None:
        process = self._process
        if process.poll() is not None:
            return
        try:
            self.request({"command": "shutdown"})
            process.wait(timeout=10.0)
        except Exception:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)

    def __enter__(self) -> CuroboPlannerWorker:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()


def build_curobo_scene(
    planner: Any,
    active_robot: RobotSpec,
    other_robot: RobotSpec,
    other_joint_position: Sequence[float],
    doll_poses: dict[str, PoseSpec] | None = None,
    excluded_doll_ids: Sequence[str] = (),
) -> Any:
    """Build the active arm's collision world in its own base frame."""

    import torch
    from curobo.scene import Cuboid, Scene, Sphere
    from curobo.types import JointState

    cuboids: list[Any] = []
    spheres: list[Any] = []

    def cuboid_from_world(
        name: str,
        pose: PoseSpec,
        dims: Sequence[float],
    ) -> None:
        base_pose = world_pose_to_robot_base(active_robot, pose)
        cuboids.append(
            Cuboid(
                name=name,
                pose=[*base_pose.position, *base_pose.quaternion],
                dims=[float(value) for value in dims],
            )
        )

    cuboid_from_world(
        "table",
        PoseSpec(TABLE_POSITION, TABLE_ORIENTATION),
        TABLE_SIZE,
    )
    cuboid_from_world(
        "ground",
        PoseSpec(GROUND_POSITION, GROUND_ORIENTATION),
        GROUND_SIZE,
    )

    stand_source_corners = [
        (x, y, z)
        for x in (
            CAMERA_STAND_SOURCE_BBOX_MIN[0],
            CAMERA_STAND_SOURCE_BBOX_MAX[0],
        )
        for y in (
            CAMERA_STAND_SOURCE_BBOX_MIN[1],
            CAMERA_STAND_SOURCE_BBOX_MAX[1],
        )
        for z in (
            CAMERA_STAND_SOURCE_BBOX_MIN[2],
            CAMERA_STAND_SOURCE_BBOX_MAX[2],
        )
    ]
    stand_world_corners = [
        tuple(
            CAMERA_STAND_POSITION[index] + rotated[index]
            for index in range(3)
        )
        for rotated in (
            _quaternion_rotate_vector(CAMERA_STAND_ORIENTATION, corner)
            for corner in stand_source_corners
        )
    ]
    stand_min = tuple(
        min(corner[index] for corner in stand_world_corners)
        for index in range(3)
    )
    stand_max = tuple(
        max(corner[index] for corner in stand_world_corners)
        for index in range(3)
    )
    cuboid_from_world(
        "camera_stand_conservative_bbox",
        PoseSpec(
            tuple((stand_min[index] + stand_max[index]) / 2.0 for index in range(3)),
            IDENTITY_QUATERNION,
        ),
        tuple(stand_max[index] - stand_min[index] for index in range(3)),
    )

    # The room is far outside the tabletop workspace.  Four conservative wall
    # slabs retain it in the collision world without duplicating its floor.
    room_x_min, room_x_max = -4.6153, 4.6153
    room_y_min, room_y_max = -3.4970, 4.9532
    room_z_center, room_height = 2.12, 4.38
    wall_thickness = 0.05
    cuboid_from_world(
        "room_wall_x_min",
        PoseSpec((room_x_min, 0.7281, room_z_center), IDENTITY_QUATERNION),
        (wall_thickness, room_y_max - room_y_min, room_height),
    )
    cuboid_from_world(
        "room_wall_x_max",
        PoseSpec((room_x_max, 0.7281, room_z_center), IDENTITY_QUATERNION),
        (wall_thickness, room_y_max - room_y_min, room_height),
    )
    cuboid_from_world(
        "room_wall_y_min",
        PoseSpec((0.0, room_y_min, room_z_center), IDENTITY_QUATERNION),
        (room_x_max - room_x_min, wall_thickness, room_height),
    )
    cuboid_from_world(
        "room_wall_y_max",
        PoseSpec((0.0, room_y_max, room_z_center), IDENTITY_QUATERNION),
        (room_x_max - room_x_min, wall_thickness, room_height),
    )

    cuboid_from_world(
        f"{other_robot.name}_base",
        PoseSpec(
            (
                other_robot.base_pose.position[0],
                other_robot.base_pose.position[1],
                other_robot.base_pose.position[2] + 0.060,
            ),
            other_robot.base_pose.quaternion,
        ),
        (0.130, 0.130, 0.120),
    )
    other_state = JointState.from_position(
        torch.as_tensor(
            [list(float(value) for value in other_joint_position)],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
        joint_names=list(PIPER_ARM_JOINT_NAMES),
    )
    other_spheres = (
        planner.compute_kinematics(other_state)
        .robot_spheres.detach()
        .cpu()
        .reshape(-1, 4)
        .tolist()
    )
    for index, (x, y, z, radius) in enumerate(other_spheres):
        if radius <= 0.0:
            continue
        world_offset = _quaternion_rotate_vector(
            other_robot.base_pose.quaternion,
            (x, y, z),
        )
        world_center = tuple(
            other_robot.base_pose.position[axis] + world_offset[axis]
            for axis in range(3)
        )
        base_center = world_pose_to_robot_base(
            active_robot,
            PoseSpec(world_center, IDENTITY_QUATERNION),
        ).position
        spheres.append(
            Sphere(
                name=f"{other_robot.name}_sphere_{index:02d}",
                pose=[*base_center, 1.0, 0.0, 0.0, 0.0],
                radius=float(radius),
            )
        )

    if doll_poses:
        specs_by_id = {spec.asset_id: spec for spec in get_doll_specs()}
        excluded = set(excluded_doll_ids)
        for asset_id, world_pose in doll_poses.items():
            if asset_id in excluded:
                continue
            spec = specs_by_id[asset_id]
            radius = spec.footprint_radius
            count = max(1, math.ceil(spec.height / (2.0 * radius)))
            bottom_z = world_pose.position[2] - spec.height / 2.0
            if count == 1:
                z_values = [bottom_z + spec.height / 2.0]
            else:
                z_values = [
                    bottom_z
                    + radius
                    + index
                    * (spec.height - 2.0 * radius)
                    / (count - 1)
                    for index in range(count)
                ]
            for index, center_z in enumerate(z_values):
                base_center = world_pose_to_robot_base(
                    active_robot,
                    PoseSpec(
                        (
                            world_pose.position[0],
                            world_pose.position[1],
                            center_z,
                        ),
                        IDENTITY_QUATERNION,
                    ),
                ).position
                spheres.append(
                    Sphere(
                        name=f"doll_{asset_id}_{index}",
                        pose=[*base_center, 1.0, 0.0, 0.0, 0.0],
                        radius=radius,
                    )
                )
    if len(cuboids) > CUROBO_COLLISION_CACHE["cuboid"]:
        raise ValueError("cuRobo cuboid cache is too small for the planning world")
    if len(spheres) > CUROBO_COLLISION_CACHE["sphere"]:
        raise ValueError("cuRobo sphere cache is too small for the planning world")
    return Scene(cuboid=cuboids, sphere=spheres)


def plan_curobo_pose(
    planner: Any,
    robot: RobotSpec,
    current_joint_position: Sequence[float],
    world_goal: PoseSpec,
) -> dict[str, Any]:
    """Plan one collision-checked six-joint trajectory with actual cuRobo."""

    import torch
    from curobo.types import GoalToolPose, JointState, Pose

    base_goal = world_pose_to_robot_base(robot, world_goal)
    current_state, current_state_projection = (
        _curobo_planning_current_state(
            planner,
            current_joint_position,
        )
    )
    goal_pose = Pose(
        position=torch.as_tensor(
            [base_goal.position],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
        quaternion=torch.as_tensor(
            [base_goal.quaternion],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
    )
    goal = GoalToolPose.from_poses(
        {PIPER_TOOL_LINK: goal_pose},
        ordered_tool_frames=[PIPER_TOOL_LINK],
    )
    result = planner.plan_pose(
        goal,
        current_state,
        max_attempts=CUROBO_MAX_PLAN_ATTEMPTS,
        enable_graph_attempt=1,
    )
    if result is None or not bool(result.success.any().item()):
        raise RuntimeError(
            f"cuRobo failed to plan {robot.name} to world goal "
            f"{world_goal.position}"
        )
    trajectory = result.get_interpolated_plan()
    positions = _extract_curobo_arm_positions(trajectory)
    if positions.shape[0] > CUROBO_MAX_TRAJECTORY_STEPS:
        raise RuntimeError(
            f"cuRobo trajectory has {positions.shape[0]} steps, over "
            f"{CUROBO_MAX_TRAJECTORY_STEPS}"
        )
    return {
        "joint_position": positions.numpy(),
        "joint_names": list(PIPER_ARM_JOINT_NAMES),
        "planning_time_s": float(result.total_time),
        "solve_time_s": float(result.solve_time),
        "world_goal": asdict(world_goal),
        "base_goal": asdict(base_goal),
        "current_state_projection_max_rad": current_state_projection,
        "success": True,
    }


def check_curobo_pose_ik(
    planner: Any,
    robot: RobotSpec,
    current_joint_position: Sequence[float],
    world_goal: PoseSpec,
) -> dict[str, Any]:
    """Check exact pose IK/collision feasibility without planning a path."""

    import torch
    from curobo.types import GoalToolPose, JointState, Pose

    base_goal = world_pose_to_robot_base(robot, world_goal)
    current_state, current_state_projection = (
        _curobo_planning_current_state(
            planner,
            current_joint_position,
        )
    )
    goal_pose = Pose(
        position=torch.as_tensor(
            [base_goal.position],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
        quaternion=torch.as_tensor(
            [base_goal.quaternion],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
    )
    goal = GoalToolPose.from_poses(
        {PIPER_TOOL_LINK: goal_pose},
        ordered_tool_frames=[PIPER_TOOL_LINK],
    )
    result = planner.ik_solver.solve_pose(
        goal,
        return_seeds=CUROBO_NUM_IK_SEEDS,
        current_state=current_state,
    )
    success_count = int(result.success.sum().item())
    if success_count:
        joint_position = (
            result.solution[result.success][0]
            .detach()
            .cpu()
            .reshape(1, -1)
            .numpy()
        )
    else:
        joint_position = (
            torch.empty(
                (0, len(PIPER_ARM_JOINT_NAMES)),
                dtype=torch.float32,
            )
            .numpy()
        )
    return {
        "joint_position": joint_position,
        "joint_names": list(PIPER_ARM_JOINT_NAMES),
        "world_goal": asdict(world_goal),
        "base_goal": asdict(base_goal),
        "ik_success_count": success_count,
        "current_state_projection_max_rad": current_state_projection,
        "success": success_count > 0,
    }


def _extract_curobo_arm_positions(trajectory: Any) -> Any:
    """Extract the six planned arm joints from cuRobo's dense state."""

    trajectory_width = trajectory.position.shape[-1]
    trajectory_positions = trajectory.position.detach().cpu().reshape(
        -1,
        trajectory_width,
    )
    arm_indices = [
        trajectory.joint_names.index(joint_name)
        for joint_name in PIPER_ARM_JOINT_NAMES
    ]
    return trajectory_positions[:, arm_indices]


def _curobo_planning_current_state(
    planner: Any,
    current_joint_position: Sequence[float],
) -> tuple[Any, float]:
    """Project float-noisy simulator feedback just inside cuRobo joint limits."""

    import torch
    from curobo.types import JointState

    raw_position = torch.as_tensor(
        [list(float(value) for value in current_joint_position)],
        device=CUROBO_DEVICE,
        dtype=torch.float32,
    )
    rollout = planner.trajopt_solver.core.auxiliary_rollout
    lower = rollout.action_bound_lows.reshape(1, -1)
    upper = rollout.action_bound_highs.reshape(1, -1)
    projected_position = torch.maximum(
        raw_position,
        lower + CUROBO_CURRENT_STATE_LIMIT_MARGIN_RAD,
    )
    projected_position = torch.minimum(
        projected_position,
        upper - CUROBO_CURRENT_STATE_LIMIT_MARGIN_RAD,
    )
    maximum_projection = float(
        torch.max(torch.abs(projected_position - raw_position)).item()
    )
    if maximum_projection > CUROBO_MAX_CURRENT_STATE_PROJECTION_RAD:
        raise RuntimeError(
            "Simulator joint feedback lies materially outside cuRobo limits: "
            f"projection={maximum_projection:.9f} rad"
        )
    return (
        JointState.from_position(
            projected_position,
            joint_names=list(PIPER_ARM_JOINT_NAMES),
        ),
        maximum_projection,
    )


def _curobo_cspace_failure_diagnostics(
    planner: Any,
    current_position: Any,
    goal_position: Any,
) -> dict[str, Any]:
    """Explain whether a failed c-space request starts or ends in collision."""

    import torch

    if planner.graph_planner is None:
        return {"graph_planner_available": False}
    fractions = torch.linspace(
        0.0,
        1.0,
        21,
        device=current_position.device,
        dtype=current_position.dtype,
    )
    samples = (
        current_position
        + fractions[:, None] * (goal_position - current_position)
    )
    feasible = (
        planner.graph_planner.check_samples_feasibility(samples)
        .detach()
        .cpu()
        .reshape(-1)
    )
    rollout_metrics = (
        planner.graph_planner.feasibility_rollout.compute_metrics_from_action(
            samples.unsqueeze(1)
        )
    )
    constraints = rollout_metrics.costs_and_constraints.constraints
    constraint_maxima: dict[str, list[float]] = {}
    for name, values in zip(constraints.names, constraints.values):
        per_sample = (
            values.detach().cpu().reshape(samples.shape[0], -1).amax(dim=-1)
        )
        constraint_maxima[name] = [
            float(per_sample[0].item()),
            float(per_sample[-1].item()),
            float(per_sample.max().item()),
        ]
    invalid_indices = [
        index for index, value in enumerate(feasible.tolist()) if not value
    ]
    return {
        "graph_planner_available": True,
        "current_state_feasible": bool(feasible[0].item()),
        "goal_state_feasible": bool(feasible[-1].item()),
        "linear_feasible_samples": int(feasible.sum().item()),
        "linear_sample_count": int(feasible.numel()),
        "first_infeasible_fraction": (
            None
            if not invalid_indices
            else float(fractions[invalid_indices[0]].item())
        ),
        "maximum_absolute_joint_delta_rad": float(
            torch.max(torch.abs(goal_position - current_position)).item()
        ),
        "constraint_maxima_current_goal_path": constraint_maxima,
    }


def plan_curobo_to_position(
    planner: Any,
    robot: RobotSpec,
    current_joint_position: Sequence[float],
    world_goal_position: Sequence[float],
    *,
    prefer_tool_x_down: bool = True,
    preferred_world_orientation: Sequence[float] | None = None,
    tool_to_attached_object_orientation: Sequence[float] | None = None,
    max_attached_object_tilt_degrees: float | None = None,
) -> dict[str, Any]:
    """Use cuRobo position IK plus collision-checked c-space planning.

    Piper's restricted wrist range makes a single exact vertical tool
    quaternion unnecessarily brittle.  cuRobo therefore generates a finite
    set of position-valid IK solutions; solutions whose tool +X points most
    downward are tried first, and cuRobo validates and plans every selected
    c-space trajectory.
    """

    import torch
    from curobo.types import GoalToolPose, JointState, Pose, ToolPoseCriteria

    joint_names = list(PIPER_ARM_JOINT_NAMES)
    current_state, current_state_projection = (
        _curobo_planning_current_state(
            planner,
            current_joint_position,
        )
    )
    base_goal = world_pose_to_robot_base(
        robot,
        PoseSpec(
            tuple(float(value) for value in world_goal_position),  # type: ignore[arg-type]
            IDENTITY_QUATERNION,
        ),
    )
    goal_pose = Pose(
        position=torch.as_tensor(
            [base_goal.position],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
        quaternion=torch.as_tensor(
            [IDENTITY_QUATERNION],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
    )
    goal = GoalToolPose.from_poses(
        {PIPER_TOOL_LINK: goal_pose},
        ordered_tool_frames=[PIPER_TOOL_LINK],
    )

    # Change only the IK rollout.  Mutating the trajectory optimizer's pose
    # criteria also affects c-space planning in this cuRobo version.
    planner.ik_solver.update_tool_pose_criteria(
        {PIPER_TOOL_LINK: ToolPoseCriteria.track_position()}
    )
    try:
        ik_result = planner.ik_solver.solve_pose(
            goal,
            return_seeds=CUROBO_NUM_IK_SEEDS,
            current_state=current_state,
        )
    finally:
        planner.ik_solver.update_tool_pose_criteria(
            {PIPER_TOOL_LINK: ToolPoseCriteria()}
        )
    if not bool(ik_result.success.any().item()):
        raise RuntimeError(
            f"cuRobo position IK failed for {robot.name} at "
            f"{tuple(world_goal_position)}"
        )

    solutions = ik_result.solution[ik_result.success]
    solution_states = JointState.from_position(
        solutions,
        joint_names=joint_names,
    )
    tool_poses = planner.compute_kinematics(solution_states).tool_poses
    tool_quaternions = tool_poses.quaternion.reshape(-1, 4)
    w, x, y, z = tool_quaternions.unbind(-1)
    tool_x_world_z = 2.0 * (x * z - w * y)
    joint_distance = torch.linalg.vector_norm(
        solutions - current_state.position,
        dim=-1,
    )
    if (
        max_attached_object_tilt_degrees is not None
        and tool_to_attached_object_orientation is None
    ):
        raise ValueError(
            "An attached-object tilt limit requires its tool-relative "
            "orientation"
        )
    attached_object_tilt = None
    if tool_to_attached_object_orientation is not None:
        attached_object_tilt = torch.as_tensor(
            [
                math.radians(
                    attached_object_upright_tilt_degrees(
                        quaternion_multiply(
                            robot.base_pose.quaternion,
                            candidate_base_orientation,
                        ),
                        tool_to_attached_object_orientation,
                    )
                )
                for candidate_base_orientation in (
                    tool_quaternions.detach().cpu().tolist()
                )
            ],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        )
    orientation_error = None
    if preferred_world_orientation is not None:
        preferred_base_orientation = world_pose_to_robot_base(
            robot,
            PoseSpec(
                tuple(float(value) for value in world_goal_position),  # type: ignore[arg-type]
                normalize_quaternion(preferred_world_orientation),
            ),
        ).quaternion
        preferred_quaternion = torch.as_tensor(
            preferred_base_orientation,
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        )
        orientation_dot = torch.abs(
            torch.sum(tool_quaternions * preferred_quaternion, dim=-1)
        ).clamp(max=1.0)
        orientation_error = 2.0 * torch.acos(orientation_dot)
        candidate_score = orientation_error + 0.02 * joint_distance
    elif prefer_tool_x_down:
        candidate_score = tool_x_world_z + 0.02 * joint_distance
    else:
        candidate_score = 0.02 * joint_distance

    if attached_object_tilt is not None:
        candidate_score = (
            attached_object_tilt
            + (
                0.01 * orientation_error
                if orientation_error is not None
                else 0.0
            )
            + 0.02 * joint_distance
        )

    failures: list[dict[str, Any]] = []
    planned_candidate_failures = 0
    for candidate_index in torch.argsort(candidate_score).tolist():
        candidate_attached_object_tilt_degrees = (
            None
            if attached_object_tilt is None
            else math.degrees(
                float(attached_object_tilt[candidate_index].item())
            )
        )
        if (
            max_attached_object_tilt_degrees is not None
            and candidate_attached_object_tilt_degrees is not None
            and candidate_attached_object_tilt_degrees
            > max_attached_object_tilt_degrees
        ):
            failures.append(
                {
                    "candidate_index": candidate_index,
                    "attached_object_tilt_degrees": (
                        candidate_attached_object_tilt_degrees
                    ),
                }
            )
            continue
        candidate = solutions[candidate_index : candidate_index + 1]
        candidate_state = JointState.from_position(
            candidate,
            joint_names=joint_names,
        )
        result = planner.plan_cspace(
            candidate_state,
            current_state,
            max_attempts=CUROBO_MAX_PLAN_ATTEMPTS,
            enable_graph_attempt=1,
        )
        if result is None or not bool(result.success.any().item()):
            failure = {
                "candidate_index": candidate_index,
                "tool_x_world_z": float(
                    tool_x_world_z[candidate_index].item()
                ),
            }
            if planned_candidate_failures == 0:
                failure["cspace_diagnostics"] = (
                    _curobo_cspace_failure_diagnostics(
                        planner,
                        current_state.position,
                        candidate,
                    )
                )
            failures.append(failure)
            planned_candidate_failures += 1
            continue
        trajectory = result.get_interpolated_plan()
        positions = _extract_curobo_arm_positions(trajectory)
        if positions.shape[0] > CUROBO_MAX_TRAJECTORY_STEPS:
            failures.append(
                {
                    "candidate_index": candidate_index,
                    "trajectory_steps": int(positions.shape[0]),
                }
            )
            continue
        selected_base_pose = PoseSpec(
            tuple(
                float(value)
                for value in tool_poses.position.reshape(-1, 3)[
                    candidate_index
                ]
                .detach()
                .cpu()
                .tolist()
            ),  # type: ignore[arg-type]
            tuple(
                float(value)
                for value in tool_quaternions[candidate_index]
                .detach()
                .cpu()
                .tolist()
            ),  # type: ignore[arg-type]
        )
        selected_world_pose = robot_base_pose_to_world(
            robot,
            selected_base_pose,
        )
        return {
            "joint_position": positions.numpy(),
            "joint_names": joint_names,
            "planning_time_s": float(result.total_time),
            "solve_time_s": float(result.solve_time),
            "ik_success_count": int(ik_result.success.sum().item()),
            "selected_candidate_index": int(candidate_index),
            "tool_x_world_z": float(
                tool_x_world_z[candidate_index].item()
            ),
            "preferred_orientation_error_rad": (
                None
                if orientation_error is None
                else float(orientation_error[candidate_index].item())
            ),
            "attached_object_tilt_degrees": (
                candidate_attached_object_tilt_degrees
            ),
            "world_goal_position": [
                float(value) for value in world_goal_position
            ],
            "selected_world_tool_pose": asdict(selected_world_pose),
            "failed_candidates": failures,
            "current_state_projection_max_rad": current_state_projection,
            "success": True,
        }
    raise RuntimeError(
        f"cuRobo found {solutions.shape[0]} IK solutions for {robot.name} at "
        f"{tuple(world_goal_position)}, but no collision-free trajectory; "
        f"failures={failures}"
    )


def plan_curobo_joint_goal(
    planner: Any,
    robot: RobotSpec,
    current_joint_position: Sequence[float],
    goal_joint_position: Sequence[float],
) -> dict[str, Any]:
    """Plan a collision-checked cuRobo trajectory to an explicit arm state."""

    import torch
    from curobo.types import JointState

    joint_names = list(PIPER_ARM_JOINT_NAMES)
    current_state, current_state_projection = (
        _curobo_planning_current_state(
            planner,
            current_joint_position,
        )
    )
    goal_state = JointState.from_position(
        torch.as_tensor(
            [list(float(value) for value in goal_joint_position)],
            device=CUROBO_DEVICE,
            dtype=torch.float32,
        ),
        joint_names=joint_names,
    )
    result = planner.plan_cspace(
        goal_state,
        current_state,
        max_attempts=CUROBO_MAX_PLAN_ATTEMPTS,
        enable_graph_attempt=1,
    )
    if result is None or not bool(result.success.any().item()):
        raise RuntimeError(
            f"cuRobo failed to plan {robot.name} from "
            f"{tuple(current_joint_position)} to {tuple(goal_joint_position)}"
        )
    positions = _extract_curobo_arm_positions(
        result.get_interpolated_plan()
    )
    if positions.shape[0] > CUROBO_MAX_TRAJECTORY_STEPS:
        raise RuntimeError(
            f"cuRobo trajectory has {positions.shape[0]} steps, over "
            f"{CUROBO_MAX_TRAJECTORY_STEPS}"
        )
    return {
        "joint_position": positions.numpy(),
        "joint_names": joint_names,
        "planning_time_s": float(result.total_time),
        "solve_time_s": float(result.solve_time),
        "goal_joint_position": [
            float(value) for value in goal_joint_position
        ],
        "current_state_projection_max_rad": current_state_projection,
        "success": True,
    }


def _planar_distance(first: PoseSpec, second: PoseSpec) -> float:
    return math.hypot(
        first.position[0] - second.position[0],
        first.position[1] - second.position[1],
    )


def validate_initial_doll_layout(
    placements: Sequence[DollPlacement],
) -> dict[str, Any]:
    """Validate IDs, table margins, base exclusion, gaps, and non-sorted state."""

    specs_by_id = {spec.asset_id: spec for spec in get_doll_specs()}
    placement_by_id = {placement.asset_id: placement for placement in placements}
    expected_ids = set(specs_by_id)
    if len(placements) != len(expected_ids) or set(placement_by_id) != expected_ids:
        raise ValueError(
            f"Layout must contain each selected doll exactly once: {sorted(expected_ids)}"
        )

    minimum_table_clearance = math.inf
    minimum_robot_base_clearance = math.inf
    for asset_id, placement in placement_by_id.items():
        spec = specs_by_id[asset_id]
        x, y, z = placement.pose.position
        if not (
            MATRYOSHKA_RANDOM_X_RANGE[0] <= x <= MATRYOSHKA_RANDOM_X_RANGE[1]
            and MATRYOSHKA_RANDOM_Y_RANGE[0] <= y <= MATRYOSHKA_RANDOM_Y_RANGE[1]
        ):
            raise ValueError(f"{asset_id}: sampled centre is outside the random area")
        expected_z = TABLE_TOP_Z + spec.height / 2.0 + MATRYOSHKA_SPAWN_CLEARANCE
        if not math.isclose(z, expected_z, abs_tol=1.0e-9):
            raise ValueError(f"{asset_id}: sampled Z is not the upright spawn height")
        table_clearance = min(
            x - spec.footprint_radius - TABLE_X_RANGE[0],
            TABLE_X_RANGE[1] - x - spec.footprint_radius,
            y - spec.footprint_radius - TABLE_Y_RANGE[0],
            TABLE_Y_RANGE[1] - y - spec.footprint_radius,
        )
        minimum_table_clearance = min(minimum_table_clearance, table_clearance)
        if table_clearance < MATRYOSHKA_TABLE_EDGE_CLEARANCE:
            raise ValueError(
                f"{asset_id}: table edge clearance {table_clearance:.4f} m"
            )
        for robot_spec in ROBOT_SPECS:
            distance = math.hypot(
                x - robot_spec.base_pose.position[0],
                y - robot_spec.base_pose.position[1],
            )
            clearance = (
                distance
                - spec.footprint_radius
                - MATRYOSHKA_ROBOT_BASE_EXCLUSION_RADIUS
            )
            minimum_robot_base_clearance = min(
                minimum_robot_base_clearance, clearance
            )
            if clearance < 0.0:
                raise ValueError(f"{asset_id}: intersects a robot base exclusion zone")

    minimum_pair_gap = math.inf
    placement_items = list(placement_by_id.items())
    for index, (first_id, first) in enumerate(placement_items):
        for second_id, second in placement_items[index + 1 :]:
            gap = (
                _planar_distance(first.pose, second.pose)
                - specs_by_id[first_id].footprint_radius
                - specs_by_id[second_id].footprint_radius
            )
            minimum_pair_gap = min(minimum_pair_gap, gap)
            if gap < MATRYOSHKA_INITIAL_GAP - 1.0e-9:
                raise ValueError(
                    f"{first_id}/{second_id}: pair gap {gap:.4f} m is too small"
                )

    targets = {
        placement.asset_id: placement for placement in compute_doll_target_layout()
    }
    target_errors = {
        asset_id: _planar_distance(placement.pose, targets[asset_id].pose)
        for asset_id, placement in placement_by_id.items()
    }
    already_sorted = all(
        error <= MATRYOSHKA_POSITION_TOLERANCE
        for error in target_errors.values()
    )
    if already_sorted:
        raise ValueError("Initial layout already satisfies the final sorted targets")

    return {
        "minimum_pair_surface_gap_m": minimum_pair_gap,
        "minimum_table_edge_clearance_m": minimum_table_clearance,
        "minimum_robot_base_exclusion_clearance_m": minimum_robot_base_clearance,
        "already_sorted": already_sorted,
        "target_xy_errors_m": target_errors,
    }


def sample_initial_doll_layout(
    seed: int,
    *,
    samples_per_object: int = MATRYOSHKA_LAYOUT_SAMPLES_PER_OBJECT,
) -> tuple[DollPlacement, ...]:
    """Use finite deterministic rejection sampling for five upright poses."""

    import numpy as np

    if seed < 0:
        raise ValueError("Episode seed must be non-negative")
    if samples_per_object <= 0:
        raise ValueError("samples_per_object must be positive")
    rng = np.random.default_rng(seed)
    specs = get_doll_specs()
    specs_by_id = {spec.asset_id: spec for spec in specs}
    # Largest-first placement keeps rejection rates low while the return order
    # remains the stable asset-ID order used everywhere else.
    sampling_order = sorted(specs, key=lambda spec: spec.footprint_radius, reverse=True)
    accepted: dict[str, DollPlacement] = {}
    for spec in sampling_order:
        for _ in range(samples_per_object):
            x = float(rng.uniform(*MATRYOSHKA_RANDOM_X_RANGE))
            y = float(rng.uniform(*MATRYOSHKA_RANDOM_Y_RANGE))
            yaw = float(rng.uniform(-math.pi, math.pi))
            candidate = DollPlacement(
                asset_id=spec.asset_id,
                pose=PoseSpec(
                    (
                        x,
                        y,
                        TABLE_TOP_Z
                        + spec.height / 2.0
                        + MATRYOSHKA_SPAWN_CLEARANCE,
                    ),
                    (
                        math.cos(yaw / 2.0),
                        0.0,
                        0.0,
                        math.sin(yaw / 2.0),
                    ),
                ),
                yaw_rad=yaw,
            )
            table_clearance = min(
                x - spec.footprint_radius - TABLE_X_RANGE[0],
                TABLE_X_RANGE[1] - x - spec.footprint_radius,
                y - spec.footprint_radius - TABLE_Y_RANGE[0],
                TABLE_Y_RANGE[1] - y - spec.footprint_radius,
            )
            if table_clearance < MATRYOSHKA_TABLE_EDGE_CLEARANCE:
                continue
            if any(
                math.hypot(
                    x - robot.base_pose.position[0],
                    y - robot.base_pose.position[1],
                )
                < MATRYOSHKA_ROBOT_BASE_EXCLUSION_RADIUS
                + spec.footprint_radius
                for robot in ROBOT_SPECS
            ):
                continue
            if any(
                _planar_distance(candidate.pose, previous.pose)
                < spec.footprint_radius
                + specs_by_id[previous.asset_id].footprint_radius
                + MATRYOSHKA_INITIAL_GAP
                for previous in accepted.values()
            ):
                continue
            accepted[spec.asset_id] = candidate
            break
        else:
            raise RuntimeError(
                f"Failed to sample {spec.asset_id} after "
                f"{samples_per_object} finite attempts for seed {seed}"
            )

    layout = tuple(accepted[spec.asset_id] for spec in specs)
    validate_initial_doll_layout(layout)
    return layout


def set_episode_random_seeds(seed: int) -> dict[str, Any]:
    """Seed Python, NumPy, PyTorch, CUDA, and the torch-backed cuRobo path."""

    import random

    import numpy as np
    import torch

    if seed < 0:
        raise ValueError("Episode seed must be non-negative")
    numpy_seed = seed % (2**32)
    random.seed(seed)
    np.random.seed(numpy_seed)
    torch.manual_seed(seed)
    cuda_seeded = bool(torch.cuda.is_available())
    if cuda_seeded:
        torch.cuda.manual_seed_all(seed)
    return {
        "episode": seed,
        "python_random": seed,
        "numpy": numpy_seed,
        "torch": seed,
        "torch_cuda": seed if cuda_seeded else None,
        "curobo_via_torch": seed,
    }


def _require_mdl_entry(path: Path, entry: str) -> None:
    text = path.read_text(encoding="utf-8")
    pattern = rf"(?m)^\s*export\s+material\s+{re.escape(entry)}\s*\("
    if re.search(pattern, text) is None:
        raise ValueError(f"{path}: missing exported material {entry!r}")


def _parse_urdf() -> dict[str, Any]:
    root = ET.parse(PIPER_URDF).getroot()
    joints: dict[str, dict[str, Any]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        limit = joint.find("limit")
        mimic = joint.find("mimic")
        parent = joint.find("parent")
        child = joint.find("child")
        joints[name] = {
            "type": joint.attrib["type"],
            "parent": None if parent is None else parent.attrib["link"],
            "child": None if child is None else child.attrib["link"],
            "lower": None if limit is None else float(limit.attrib["lower"]),
            "upper": None if limit is None else float(limit.attrib["upper"]),
            "velocity": None if limit is None else float(limit.attrib["velocity"]),
            "mimic": None if mimic is None else mimic.attrib["joint"],
        }
    return {"robot_name": root.attrib["name"], "joints": joints}


def validate_static_assets() -> dict[str, Any]:
    """Validate paths, metadata, material entries, and the URDF mapping.

    This check is intentionally independent of Isaac Sim so it can run in the
    fast test suite.
    """

    required_paths = (
        PIPER_USD,
        PIPER_URDF,
        PIPER_REFERENCE_CONFIG,
        ROOM_USD,
        HDR_TEXTURE,
        CAMERA_STAND_USD,
        TABLE_MDL,
        GROUND_MDL,
    )
    missing = [str(path) for path in required_paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("Missing required assets: " + ", ".join(missing))

    dolls = get_doll_specs()
    missing_doll_files = [
        str(path)
        for doll in dolls
        for path in (doll.asset_path, doll.metadata_path)
        if not path.is_file()
    ]
    if missing_doll_files:
        raise FileNotFoundError(
            "Missing selected doll files: " + ", ".join(missing_doll_files)
        )
    if any(doll.uuid != MATRYOSHKA_UUID for doll in dolls):
        raise ValueError("The five selected dolls do not share the required UUID")

    _require_mdl_entry(TABLE_MDL, TABLE_MATERIAL_ENTRY)
    _require_mdl_entry(GROUND_MDL, GROUND_MATERIAL_ENTRY)

    urdf = _parse_urdf()
    if urdf["robot_name"] != "Piper":
        raise ValueError(f"Unexpected URDF robot name: {urdf['robot_name']!r}")
    joints = urdf["joints"]
    missing_joints = set(PIPER_ARM_JOINT_NAMES + PIPER_GRIPPER_JOINT_NAMES) - joints.keys()
    if missing_joints:
        raise ValueError(f"URDF is missing joints: {sorted(missing_joints)}")
    if joints["joint8"]["mimic"] != "gripper_joint":
        raise ValueError("joint8 must mimic gripper_joint")
    gripper_center = joints.get("gripper_center_fixed")
    if gripper_center is None or (
        gripper_center["parent"],
        gripper_center["child"],
    ) != ("link6", "gripper_center"):
        raise ValueError("URDF gripper_center must be fixed to link6")
    camera_fixed = joints.get("camera_fixed")
    if camera_fixed is None or (
        camera_fixed["parent"],
        camera_fixed["child"],
    ) != ("link6", "camera"):
        raise ValueError("URDF camera helper must be fixed to link6")

    return {
        "repository_root": str(REPOSITORY_ROOT),
        "world": {
            "meters_per_unit": WORLD_METERS_PER_UNIT,
            "up_axis": WORLD_UP_AXIS,
            "quaternion_order": QUATERNION_ORDER,
        },
        "table": {
            "position": TABLE_POSITION,
            "size": TABLE_SIZE,
            "top_z": TABLE_TOP_Z,
            "material": f"{TABLE_MDL}::{TABLE_MATERIAL_ENTRY}",
        },
        "ground": {
            "position": GROUND_POSITION,
            "size": GROUND_SIZE,
            "material": f"{GROUND_MDL}::{GROUND_MATERIAL_ENTRY}",
        },
        "robots": [asdict(spec) for spec in ROBOT_SPECS],
        "urdf": urdf,
        "dolls": [
            {
                **asdict(doll),
                "asset_path": str(doll.asset_path),
                "metadata_path": str(doll.metadata_path),
            }
            for doll in dolls
        ],
    }


def _aligned_bbox(stage: Any, prim: Any, usd: Any, usd_geom: Any) -> dict[str, list[float]]:
    cache = usd_geom.BBoxCache(
        usd.TimeCode.Default(),
        [
            usd_geom.Tokens.default_,
            usd_geom.Tokens.render,
            usd_geom.Tokens.proxy,
        ],
    )
    box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    return {
        "min": [float(value) for value in box.GetMin()],
        "max": [float(value) for value in box.GetMax()],
    }


def inspect_usd_assets() -> dict[str, Any]:
    """Inspect the actual USD layers.

    A ``SimulationApp`` must already be running before this function is
    called, because Isaac Sim supplies the matching USD Python bindings.
    """

    from pxr import Usd, UsdGeom, UsdPhysics  # type: ignore[import-not-found]

    reports: dict[str, Any] = {}
    for path_string, expected_default_prim in _EXPECTED_USD_DEFAULT_PRIMS.items():
        stage = Usd.Stage.Open(path_string)
        if stage is None:
            raise RuntimeError(f"Unable to open USD asset: {path_string}")
        default_prim = stage.GetDefaultPrim()
        if not default_prim:
            raise ValueError(f"{path_string}: no default prim")
        default_path = str(default_prim.GetPath())
        meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
        up_axis = str(UsdGeom.GetStageUpAxis(stage))
        if default_path != expected_default_prim:
            raise ValueError(
                f"{path_string}: expected default prim {expected_default_prim}, "
                f"found {default_path}"
            )
        if not math.isclose(meters_per_unit, WORLD_METERS_PER_UNIT):
            raise ValueError(
                f"{path_string}: expected metresPerUnit=1, found {meters_per_unit}"
            )
        if up_axis != WORLD_UP_AXIS:
            raise ValueError(f"{path_string}: expected Z-up, found {up_axis}-up")

        joints: list[str] = []
        lights: list[str] = []
        cameras: list[str] = []
        rigid_bodies: list[str] = []
        collisions: list[str] = []
        mass_api_prims: list[str] = []
        physics_material_prims: list[str] = []
        for prim in stage.Traverse():
            prim_path = str(prim.GetPath())
            if prim.IsA(UsdPhysics.Joint):
                joints.append(prim.GetName())
            if str(prim.GetTypeName()).endswith("Light"):
                lights.append(prim_path)
            if prim.IsA(UsdGeom.Camera):
                cameras.append(prim_path)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_bodies.append(prim_path)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                collisions.append(prim_path)
            if prim.HasAPI(UsdPhysics.MassAPI):
                mass_api_prims.append(prim_path)
            if prim.HasAPI(UsdPhysics.MaterialAPI):
                physics_material_prims.append(prim_path)
        reports[path_string] = {
            "default_prim": default_path,
            "meters_per_unit": meters_per_unit,
            "up_axis": up_axis,
            "bbox": _aligned_bbox(stage, default_prim, Usd, UsdGeom),
            "prim_count": sum(1 for _ in stage.Traverse()),
            "joints": joints,
            "lights": lights,
            "cameras": cameras,
            "rigid_bodies": rigid_bodies,
            "collisions": collisions,
            "mass_api_prims": mass_api_prims,
            "physics_material_prims": physics_material_prims,
        }

    piper_report = reports[str(PIPER_USD)]
    if set(piper_report["joints"]) != _EXPECTED_PIPER_USD_JOINTS:
        raise ValueError(
            "Piper USD joint mismatch: "
            f"expected {sorted(_EXPECTED_PIPER_USD_JOINTS)}, "
            f"found {sorted(piper_report['joints'])}"
        )
    piper_stage = Usd.Stage.Open(str(PIPER_USD))
    for rel_path in (PIPER_CAMERA_MOUNT_REL_PATH, PIPER_TOOL_REL_PATH):
        path = f"/Piper/{rel_path}"
        if not piper_stage.GetPrimAtPath(path):
            raise ValueError(f"Piper USD is missing helper prim {path}")
    if piper_report["cameras"]:
        raise ValueError("Piper helper unexpectedly contains an authored USD Camera")

    room_report = reports[str(ROOM_USD)]
    expected_room_light = f"/World/{ROOM_RESIDUAL_LIGHT_REL_PATH}"
    if room_report["lights"] != [expected_room_light]:
        raise ValueError(
            "Room light audit changed: "
            f"expected {[expected_room_light]}, found {room_report['lights']}"
        )

    stand_report = reports[str(CAMERA_STAND_USD)]
    for actual, expected in zip(
        stand_report["bbox"]["min"], CAMERA_STAND_SOURCE_BBOX_MIN
    ):
        if not math.isclose(actual, expected, abs_tol=2.0e-4):
            raise ValueError(f"Camera stand minimum bound changed: {stand_report['bbox']}")
    for actual, expected in zip(
        stand_report["bbox"]["max"], CAMERA_STAND_SOURCE_BBOX_MAX
    ):
        if not math.isclose(actual, expected, abs_tol=2.0e-4):
            raise ValueError(f"Camera stand maximum bound changed: {stand_report['bbox']}")
    if not stand_report["collisions"]:
        raise ValueError("Camera stand has no authored collision geometry")

    for doll in get_doll_specs():
        report = reports[str(doll.asset_path)]
        if report["rigid_bodies"] != ["/root"]:
            raise ValueError(
                f"{doll.asset_path}: expected /root rigid body, "
                f"found {report['rigid_bodies']}"
            )
        if "/root/collision/model" not in report["collisions"]:
            raise ValueError(f"{doll.asset_path}: missing collision model")
        if report["mass_api_prims"] or report["physics_material_prims"]:
            raise ValueError(
                f"{doll.asset_path}: asset now authors mass/material; "
                "review override policy"
            )

    return {
        "assets": reports,
        "known_room_light_to_disable": f"{ROOM_PRIM_PATH}/{ROOM_RESIDUAL_LIGHT_REL_PATH}",
        "doll_mass_and_friction_override_required": True,
        "piper_camera_helper_is_not_sensor": True,
    }


def run_asset_audit() -> dict[str, Any]:
    """Run the combined audit after the caller has started ``SimulationApp``."""

    return {"static": validate_static_assets(), "usd": inspect_usd_assets()}


def _set_xform(
    prim: Any,
    *,
    position: Sequence[float],
    quaternion: Sequence[float],
    scale: Sequence[float] | None = None,
) -> None:
    """Author one unambiguous translate/orient/(optional) scale stack."""

    from pxr import Gf, UsdGeom  # type: ignore[import-not-found]

    normalized = normalize_quaternion(quaternion)
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*(float(value) for value in position))
    )
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(
            normalized[0],
            Gf.Vec3d(normalized[1], normalized[2], normalized[3]),
        )
    )
    if scale is not None:
        xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*(float(value) for value in scale))
        )


def _create_mdl_material(
    stage: Any, material_path: str, mdl_path: Path, material_entry: str
) -> Any:
    """Create a USD material backed by one explicit local MDL entry."""

    from omni.usd.commands import (  # type: ignore[import-not-found]
        CreateMdlMaterialPrimCommand,
    )
    from pxr import UsdShade  # type: ignore[import-not-found]

    CreateMdlMaterialPrimCommand(
        mtl_url=str(mdl_path),
        mtl_name=material_entry,
        mtl_path=material_path,
        stage=stage,
        select_new_prim=False,
    ).do()
    material = UsdShade.Material(stage.GetPrimAtPath(material_path))
    if not material:
        raise RuntimeError(
            f"Failed to create material {material_entry!r} from {mdl_path}"
        )
    return material


def _create_static_cube(
    stage: Any,
    *,
    prim_path: str,
    position: Sequence[float],
    quaternion: Sequence[float],
    size: Sequence[float],
    material: Any,
) -> Any:
    """Create a unit USD cube scaled to metres and make it a static collider."""

    from pxr import UsdGeom, UsdPhysics, UsdShade  # type: ignore[import-not-found]

    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.CreateSizeAttr(1.0)
    _set_xform(
        cube.GetPrim(),
        position=position,
        quaternion=quaternion,
        scale=size,
    )
    collision = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    collision.CreateCollisionEnabledAttr(True)
    binding = UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
    binding.Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
    )
    return cube.GetPrim()


def _add_reference(
    stage: Any,
    *,
    prim_path: str,
    asset_path: Path,
    pose: PoseSpec,
) -> Any:
    prim = stage.DefinePrim(prim_path, "Xform")
    if not prim.GetReferences().AddReference(str(asset_path)):
        raise RuntimeError(f"Failed to reference {asset_path} at {prim_path}")
    _set_xform(
        prim,
        position=pose.position,
        quaternion=pose.quaternion,
    )
    return prim


def _prim_references(prim: Any) -> list[str]:
    references: list[str] = []
    for prim_spec in prim.GetPrimStack():
        for reference in prim_spec.referenceList.prependedItems:
            references.append(str(reference.assetPath))
    return references


def create_scene() -> Any:
    """Build the static room/HDR/ground/table/stand scene and reset physics."""

    import omni.usd  # type: ignore[import-not-found]
    from isaacsim.core.api import World  # type: ignore[import-not-found]
    from pxr import Sdf, UsdGeom, UsdLux  # type: ignore[import-not-found]

    world = World(
        physics_dt=PHYSICS_DT,
        rendering_dt=RENDER_DT,
        stage_units_in_meters=WORLD_METERS_PER_UNIT,
    )
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, WORLD_METERS_PER_UNIT)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.DefinePrim("/World", "Xform")
    stage.DefinePrim("/World/Scene", "Xform")
    stage.DefinePrim("/World/Robots", "Xform")
    stage.DefinePrim("/World/Looks", "Scope")

    room_prim = _add_reference(
        stage,
        prim_path=ROOM_PRIM_PATH,
        asset_path=ROOM_USD,
        pose=PoseSpec((0.0, 0.0, 0.0), IDENTITY_QUATERNION),
    )
    room_light_path = f"{ROOM_PRIM_PATH}/{ROOM_RESIDUAL_LIGHT_REL_PATH}"
    room_light = stage.GetPrimAtPath(room_light_path)
    if not room_light:
        raise RuntimeError(f"Expected authored room light at {room_light_path}")
    room_light.SetActive(False)

    stand_prim = _add_reference(
        stage,
        prim_path=CAMERA_STAND_PRIM_PATH,
        asset_path=CAMERA_STAND_USD,
        pose=PoseSpec(CAMERA_STAND_POSITION, CAMERA_STAND_ORIENTATION),
    )

    table_material = _create_mdl_material(
        stage,
        TABLE_VISUAL_MATERIAL_PATH,
        TABLE_MDL,
        TABLE_MATERIAL_ENTRY,
    )
    ground_material = _create_mdl_material(
        stage,
        GROUND_VISUAL_MATERIAL_PATH,
        GROUND_MDL,
        GROUND_MATERIAL_ENTRY,
    )
    _create_static_cube(
        stage,
        prim_path=TABLE_PRIM_PATH,
        position=TABLE_POSITION,
        quaternion=TABLE_ORIENTATION,
        size=TABLE_SIZE,
        material=table_material,
    )
    _create_static_cube(
        stage,
        prim_path=GROUND_PRIM_PATH,
        position=GROUND_POSITION,
        quaternion=GROUND_ORIENTATION,
        size=GROUND_SIZE,
        material=ground_material,
    )

    dome = UsdLux.DomeLight.Define(stage, HDR_DOME_PRIM_PATH)
    dome.CreateIntensityAttr(HDR_DOME_INTENSITY)
    dome.CreateTextureFileAttr(Sdf.AssetPath(str(HDR_TEXTURE)))
    dome.CreateTextureFormatAttr("latlong")
    dome.GetPrim().CreateAttribute(
        "visibleInPrimaryRay", Sdf.ValueTypeNames.Bool
    ).Set(HDR_DOME_VISIBLE_IN_PRIMARY_RAYS)
    dome_xform = UsdGeom.Xformable(dome.GetPrim())
    dome_xform.ClearXformOpOrder()
    dome_xform.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(
        HDR_DOME_ROTATION_DEGREES
    )

    # Keep strong references until composition is complete; the returned World
    # owns the current stage after reset.
    if not room_prim or not stand_prim:
        raise RuntimeError("Failed to compose the static references")
    world.reset()
    return world


def _world_bbox(stage: Any, prim_path: str) -> dict[str, list[float]]:
    from pxr import Usd, UsdGeom  # type: ignore[import-not-found]

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        raise ValueError(f"Missing prim for bounds check: {prim_path}")
    return _aligned_bbox(stage, prim, Usd, UsdGeom)


def _assert_vector_close(
    actual: Sequence[float],
    expected: Sequence[float],
    *,
    label: str,
    tolerance: float = 1.0e-5,
) -> None:
    if len(actual) != len(expected) or any(
        not math.isclose(float(a), float(e), abs_tol=tolerance)
        for a, e in zip(actual, expected)
    ):
        raise ValueError(f"{label}: expected {list(expected)}, found {list(actual)}")


def _material_report(stage: Any, geometry_path: str, material_path: str) -> dict[str, Any]:
    from pxr import UsdShade  # type: ignore[import-not-found]

    geometry = stage.GetPrimAtPath(geometry_path)
    direct_binding = UsdShade.MaterialBindingAPI(geometry).GetDirectBinding()
    bound_path = str(direct_binding.GetMaterialPath())
    if bound_path != material_path:
        raise ValueError(
            f"{geometry_path}: expected material {material_path}, found {bound_path}"
        )
    shader = stage.GetPrimAtPath(f"{material_path}/Shader")
    if not shader:
        raise ValueError(f"{material_path}: missing MDL Shader prim")
    return {
        "material_path": bound_path,
        "shader_attributes": {
            attribute.GetName(): str(attribute.Get())
            for attribute in shader.GetAttributes()
            if "sourceAsset" in attribute.GetName()
            or "implementationSource" in attribute.GetName()
        },
    }


def validate_scene(world: Any) -> dict[str, Any]:
    """Validate the composed static scene against all fixed scene invariants."""

    import omni.usd  # type: ignore[import-not-found]
    from pxr import Sdf, UsdGeom, UsdPhysics  # type: ignore[import-not-found]

    stage = omni.usd.get_context().get_stage()
    if not math.isclose(
        float(UsdGeom.GetStageMetersPerUnit(stage)), WORLD_METERS_PER_UNIT
    ):
        raise ValueError("Scene stage is not metre-scaled")
    if str(UsdGeom.GetStageUpAxis(stage)) != WORLD_UP_AXIS:
        raise ValueError("Scene stage is not Z-up")
    if not math.isclose(float(world.get_physics_dt()), PHYSICS_DT, abs_tol=1.0e-12):
        raise ValueError("Physics dt differs from the configured 120 Hz")
    if not math.isclose(float(world.get_rendering_dt()), RENDER_DT, abs_tol=1.0e-12):
        raise ValueError("Rendering dt differs from the configured 30 Hz")

    table = stage.GetPrimAtPath(TABLE_PRIM_PATH)
    ground = stage.GetPrimAtPath(GROUND_PRIM_PATH)
    for name, prim in (("table", table), ("ground", ground)):
        if not prim or not prim.HasAPI(UsdPhysics.CollisionAPI):
            raise ValueError(f"{name} is missing its static CollisionAPI")
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f"{name} must be static, not a rigid body")

    table_bbox = _world_bbox(stage, TABLE_PRIM_PATH)
    ground_bbox = _world_bbox(stage, GROUND_PRIM_PATH)
    _assert_vector_close(
        table_bbox["min"],
        (TABLE_X_RANGE[0], TABLE_Y_RANGE[0], TABLE_POSITION[2] - TABLE_SIZE[2] / 2),
        label="table bbox min",
    )
    _assert_vector_close(
        table_bbox["max"],
        (TABLE_X_RANGE[1], TABLE_Y_RANGE[1], TABLE_TOP_Z),
        label="table bbox max",
    )
    _assert_vector_close(
        ground_bbox["min"], (-3.0, -3.0, -0.1), label="ground bbox min"
    )
    _assert_vector_close(
        ground_bbox["max"], (3.0, 3.0, 0.0), label="ground bbox max"
    )

    room = stage.GetPrimAtPath(ROOM_PRIM_PATH)
    stand = stage.GetPrimAtPath(CAMERA_STAND_PRIM_PATH)
    room_references = _prim_references(room)
    stand_references = _prim_references(stand)
    if str(ROOM_USD) not in room_references:
        raise ValueError(f"Room reference missing: {room_references}")
    if str(CAMERA_STAND_USD) not in stand_references:
        raise ValueError(f"Camera-stand reference missing: {stand_references}")

    room_light_path = f"{ROOM_PRIM_PATH}/{ROOM_RESIDUAL_LIGHT_REL_PATH}"
    room_light = stage.GetPrimAtPath(room_light_path)
    if not room_light or room_light.IsActive():
        raise ValueError("The room's authored RectLight was not deactivated")
    active_lights = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if str(prim.GetTypeName()).endswith("Light")
    ]
    if active_lights != [HDR_DOME_PRIM_PATH]:
        raise ValueError(f"Expected exactly the HDR DomeLight, found {active_lights}")

    dome = stage.GetPrimAtPath(HDR_DOME_PRIM_PATH)
    texture = dome.GetAttribute("inputs:texture:file").Get()
    texture_path = texture.path if isinstance(texture, Sdf.AssetPath) else str(texture)
    if texture_path != str(HDR_TEXTURE):
        raise ValueError(f"DomeLight texture mismatch: {texture_path}")
    if not math.isclose(
        float(dome.GetAttribute("inputs:intensity").Get()), HDR_DOME_INTENSITY
    ):
        raise ValueError("DomeLight intensity mismatch")
    if (
        bool(dome.GetAttribute("visibleInPrimaryRay").Get())
        != HDR_DOME_VISIBLE_IN_PRIMARY_RAYS
    ):
        raise ValueError("DomeLight primary-ray visibility mismatch")

    stand_bbox = _world_bbox(stage, CAMERA_STAND_PRIM_PATH)
    stand_source_corners = [
        (x, y, z)
        for x in (
            CAMERA_STAND_SOURCE_BBOX_MIN[0],
            CAMERA_STAND_SOURCE_BBOX_MAX[0],
        )
        for y in (
            CAMERA_STAND_SOURCE_BBOX_MIN[1],
            CAMERA_STAND_SOURCE_BBOX_MAX[1],
        )
        for z in (
            CAMERA_STAND_SOURCE_BBOX_MIN[2],
            CAMERA_STAND_SOURCE_BBOX_MAX[2],
        )
    ]
    stand_world_corners = [
        [
            CAMERA_STAND_POSITION[index] + rotated[index]
            for index in range(3)
        ]
        for rotated in (
            _quaternion_rotate_vector(CAMERA_STAND_ORIENTATION, corner)
            for corner in stand_source_corners
        )
    ]
    expected_stand_bbox_min = tuple(
        min(corner[index] for corner in stand_world_corners)
        for index in range(3)
    )
    expected_stand_bbox_max = tuple(
        max(corner[index] for corner in stand_world_corners)
        for index in range(3)
    )
    _assert_vector_close(
        stand_bbox["min"],
        expected_stand_bbox_min,
        label="camera stand bbox min",
        tolerance=2.0e-4,
    )
    _assert_vector_close(
        stand_bbox["max"],
        expected_stand_bbox_max,
        label="camera stand bbox max",
        tolerance=2.0e-4,
    )
    stand_collisions = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(CAMERA_STAND_PRIM_PATH + "/")
        and prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    if not stand_collisions:
        raise ValueError("Composed camera stand has no collision geometry")
    stand_rigid_bodies = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if (
            str(prim.GetPath()) == CAMERA_STAND_PRIM_PATH
            or str(prim.GetPath()).startswith(CAMERA_STAND_PRIM_PATH + "/")
        )
        and prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    if stand_rigid_bodies:
        raise ValueError(
            f"Camera stand must be static, found rigid bodies: {stand_rigid_bodies}"
        )

    for forbidden_path in (
        "/World/defaultGroundPlane",
        "/World/defaultDomeLight",
        "/World/groundPlane",
    ):
        if stage.GetPrimAtPath(forbidden_path):
            raise ValueError(f"Unexpected default scene primitive: {forbidden_path}")

    return {
        "world": {
            "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
            "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
            "physics_dt": float(world.get_physics_dt()),
            "rendering_dt": float(world.get_rendering_dt()),
        },
        "table": {
            "prim_path": TABLE_PRIM_PATH,
            "bbox": table_bbox,
            "collision": True,
            "static": True,
            "material": _material_report(
                stage, TABLE_PRIM_PATH, TABLE_VISUAL_MATERIAL_PATH
            ),
        },
        "ground": {
            "prim_path": GROUND_PRIM_PATH,
            "bbox": ground_bbox,
            "collision": True,
            "static": True,
            "material": _material_report(
                stage, GROUND_PRIM_PATH, GROUND_VISUAL_MATERIAL_PATH
            ),
        },
        "room": {
            "prim_path": ROOM_PRIM_PATH,
            "reference": str(ROOM_USD),
            "bbox": _world_bbox(stage, ROOM_PRIM_PATH),
            "disabled_light": room_light_path,
        },
        "camera_stand": {
            "prim_path": CAMERA_STAND_PRIM_PATH,
            "reference": str(CAMERA_STAND_USD),
            "pose": asdict(
                PoseSpec(CAMERA_STAND_POSITION, CAMERA_STAND_ORIENTATION)
            ),
            "bbox": stand_bbox,
            "collision_prim_count": len(stand_collisions),
            "static": True,
            "rigid_body_prim_count": len(stand_rigid_bodies),
        },
        "dome_light": {
            "prim_path": HDR_DOME_PRIM_PATH,
            "texture": texture_path,
            "texture_format": str(
                dome.GetAttribute("inputs:texture:format").Get()
            ),
            "intensity": float(dome.GetAttribute("inputs:intensity").Get()),
            "rotation_degrees": HDR_DOME_ROTATION_DEGREES,
            "visible_in_primary_ray": bool(
                dome.GetAttribute("visibleInPrimaryRay").Get()
            ),
        },
        "active_lights": active_lights,
    }


def capture_scene_preview(world: Any, output_path: Path) -> dict[str, Any]:
    """Render a finite overview and save the active viewport as a PNG."""

    from isaacsim.core.utils.viewports import (  # type: ignore[import-not-found]
        set_camera_view,
    )
    from omni.kit.viewport.utility import (  # type: ignore[import-not-found]
        capture_viewport_to_file,
        get_active_viewport,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    set_camera_view(SCENE_PREVIEW_EYE, SCENE_PREVIEW_TARGET)
    for _ in range(8):
        world.step(render=True)
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("No active viewport is available for the scene preview")
    capture = capture_viewport_to_file(viewport, str(output_path), is_hdr=False)
    completed = False
    for _ in range(240):
        world.step(render=True)
        if getattr(capture, "done", lambda: False)():
            completed = True
            break
    if completed:
        capture.result()
    if not output_path.is_file() or output_path.stat().st_size == 0:
        raise RuntimeError(f"Viewport capture did not produce {output_path}")

    from PIL import Image

    with Image.open(output_path) as image:
        image_info = {"size": list(image.size), "mode": image.mode}
    return {
        "path": str(output_path.resolve()),
        "bytes": output_path.stat().st_size,
        **image_info,
    }


def create_robots(world: Any) -> dict[str, Any]:
    """Reference and initialize both Piper articulations at their home pose."""

    import numpy as np
    from isaacsim.core.api.robots import Robot  # type: ignore[import-not-found]

    robots: dict[str, Any] = {}
    for spec in ROBOT_SPECS:
        _add_reference(
            world.stage,
            prim_path=spec.prim_path,
            asset_path=PIPER_USD,
            pose=spec.base_pose,
        )
        robots[spec.name] = world.scene.add(
            Robot(prim_path=spec.prim_path, name=f"{spec.name}_piper")
        )
    world.reset()

    home = np.asarray(PIPER_HOME_DOF_POSITION, dtype=np.float32)
    zero_velocity = np.zeros(len(PIPER_DOF_NAMES), dtype=np.float32)
    for name, robot in robots.items():
        if not robot.handles_initialized:
            raise RuntimeError(f"{name} Piper articulation failed to initialize")
        if tuple(robot.dof_names) != PIPER_DOF_NAMES:
            raise ValueError(
                f"{name} Piper DOF order changed: expected {PIPER_DOF_NAMES}, "
                f"found {robot.dof_names}"
            )
        robot.set_joints_default_state(
            positions=home,
            velocities=zero_velocity,
        )
        robot.set_joint_positions(home)
        robot.set_joint_velocities(zero_velocity)
    for _ in range(16):
        world.step(render=False)
    return robots


def _command_position(
    robot: Any,
    positions: Sequence[float],
    joint_indices: Sequence[int],
) -> None:
    import numpy as np
    from isaacsim.core.utils.types import (  # type: ignore[import-not-found]
        ArticulationAction,
    )

    robot.apply_action(
        ArticulationAction(
            joint_positions=np.asarray(positions, dtype=np.float32),
            joint_indices=np.asarray(joint_indices, dtype=np.int32),
        )
    )


def _set_recording_context(
    episode_recorder: Any | None,
    phase: str,
    *,
    operator: str = "",
    object_id: str = "",
) -> None:
    if episode_recorder is not None:
        episode_recorder.set_task_context(
            phase,
            operator=operator,
            object_id=object_id,
        )


def _advance_control_frame(
    world: Any,
    *,
    render: bool,
    episode_recorder: Any | None = None,
) -> None:
    physics_steps_per_control = (
        PHYSICS_FREQUENCY_HZ // CONTROL_FREQUENCY_HZ
    )
    for substep in range(physics_steps_per_control):
        world.step(
            render=(
                (render or episode_recorder is not None)
                and substep == physics_steps_per_control - 1
            )
        )
    if episode_recorder is not None:
        episode_recorder.capture_frame()


def _step_control_until(
    world: Any,
    predicate: Any,
    *,
    maximum_steps: int,
    description: str,
    render: bool,
    episode_recorder: Any | None,
) -> int:
    physics_steps_per_control = (
        PHYSICS_FREQUENCY_HZ // CONTROL_FREQUENCY_HZ
    )
    if maximum_steps % physics_steps_per_control:
        raise ValueError(
            "A recorded control wait must contain whole 30 Hz control frames"
        )
    for control_frame in range(
        1, maximum_steps // physics_steps_per_control + 1
    ):
        _advance_control_frame(
            world,
            render=render,
            episode_recorder=episode_recorder,
        )
        if predicate():
            return control_frame * physics_steps_per_control
    raise RuntimeError(
        f"Timed out after {maximum_steps} physics steps while waiting for "
        f"{description}"
    )


def execute_curobo_trajectory(
    world: Any,
    robot: Any,
    trajectory_report: dict[str, Any],
    *,
    render: bool = False,
    frame_callback: Any | None = None,
    episode_recorder: Any | None = None,
    final_tolerance_rad: float = CUROBO_FINAL_EXECUTION_TOLERANCE_RAD,
    final_settle_max_control_frames: int = (
        CUROBO_FINAL_SETTLE_MAX_CONTROL_FRAMES
    ),
) -> dict[str, Any]:
    """Execute cuRobo's 30 Hz arm positions through the articulation drive."""

    import numpy as np

    positions = np.asarray(
        trajectory_report["joint_position"],
        dtype=np.float64,
    )
    if positions.ndim != 2 or positions.shape[1] != len(
        PIPER_ARM_JOINT_NAMES
    ):
        raise ValueError(
            f"Expected [T,6] cuRobo trajectory, found {positions.shape}"
        )
    if final_tolerance_rad <= 0.0:
        raise ValueError("Final joint tolerance must be positive")
    if final_settle_max_control_frames < 0:
        raise ValueError("Final settle control-frame limit cannot be negative")
    physics_steps_per_control = (
        PHYSICS_FREQUENCY_HZ // CONTROL_FREQUENCY_HZ
    )
    maximum_tracking_error = 0.0
    for frame_index, command in enumerate(positions):
        if episode_recorder is not None:
            episode_recorder.set_arm_action(robot, command)
        _command_position(robot, command, range(6))
        _advance_control_frame(
            world,
            render=render,
            episode_recorder=episode_recorder,
        )
        actual = np.asarray(robot.get_joint_positions()[:6], dtype=np.float64)
        tracking_error = float(np.max(np.abs(actual - command)))
        maximum_tracking_error = max(maximum_tracking_error, tracking_error)
        if frame_callback is not None:
            frame_callback(frame_index, command, actual)

    final_actual = np.asarray(robot.get_joint_positions()[:6], dtype=np.float64)
    final_command = positions[-1]
    final_error = float(np.max(np.abs(final_actual - final_command)))
    terminal_settle_frames = 0
    while (
        final_error > final_tolerance_rad
        and terminal_settle_frames
        < final_settle_max_control_frames
    ):
        if episode_recorder is not None:
            episode_recorder.set_arm_action(robot, final_command)
        _command_position(robot, final_command, range(6))
        _advance_control_frame(
            world,
            render=render,
            episode_recorder=episode_recorder,
        )
        terminal_settle_frames += 1
        final_actual = np.asarray(
            robot.get_joint_positions()[:6],
            dtype=np.float64,
        )
        final_error = float(np.max(np.abs(final_actual - final_command)))
        maximum_tracking_error = max(maximum_tracking_error, final_error)
    if final_error > CUROBO_MAX_EXECUTION_ERROR_RAD:
        raise RuntimeError(
            f"cuRobo trajectory execution ended with {final_error:.6f} rad "
            f"error after {terminal_settle_frames} terminal settle frames, "
            f"over {CUROBO_MAX_EXECUTION_ERROR_RAD:.6f}"
        )
    return {
        "control_frames": int(
            positions.shape[0] + terminal_settle_frames
        ),
        "trajectory_control_frames": int(positions.shape[0]),
        "terminal_settle_control_frames": terminal_settle_frames,
        "physics_steps": int(
            (positions.shape[0] + terminal_settle_frames)
            * physics_steps_per_control
        ),
        "maximum_tracking_error_rad": maximum_tracking_error,
        "final_tracking_error_rad": final_error,
        "requested_final_tolerance_rad": final_tolerance_rad,
        "terminal_settle_limit_control_frames": (
            final_settle_max_control_frames
        ),
        "final_actual_joint_position": final_actual.tolist(),
        "final_command_joint_position": final_command.tolist(),
    }


def _step_until(
    world: Any,
    predicate: Any,
    *,
    maximum_steps: int,
    description: str,
) -> int:
    for step in range(1, maximum_steps + 1):
        world.step(render=False)
        if predicate():
            return step
    raise RuntimeError(
        f"Timed out after {maximum_steps} physics steps while waiting for {description}"
    )


def _xform_world_pose(prim_path: str, name: str) -> tuple[list[float], list[float]]:
    from isaacsim.core.prims import (  # type: ignore[import-not-found]
        SingleXFormPrim,
    )

    xform = SingleXFormPrim(
        prim_path,
        name=name,
        reset_xform_properties=False,
    )
    position, quaternion = xform.get_world_pose()
    return position.tolist(), quaternion.tolist()


def _piper_finger_center_world_pose(
    robot: RobotSpec,
    label: str,
) -> PoseSpec:
    tool_position, tool_quaternion = _xform_world_pose(
        f"{robot.prim_path}/{PIPER_TOOL_REL_PATH}",
        f"{robot.name}_{label}_distal_tool",
    )
    return piper_finger_center_pose(
        PoseSpec(tuple(tool_position), tuple(tool_quaternion))
    )


def _simulation_grasp_joint_path(
    robot: RobotSpec,
    asset_id: str,
) -> str:
    return f"{GRASP_JOINT_ROOT}/{robot.name}_{asset_id}"


def create_simulation_grasp_joint(
    world: Any,
    robot: RobotSpec,
    asset_id: str,
    doll: Any,
    *,
    relative_pose: PoseSpec | None = None,
    step_after_change: bool = True,
) -> dict[str, Any]:
    """Fix a physically contacted doll to link6 at its current relative pose."""

    from pxr import Gf, Sdf, UsdPhysics  # type: ignore[import-not-found]

    stage = world.stage
    if not stage.GetPrimAtPath(GRASP_JOINT_ROOT):
        stage.DefinePrim(GRASP_JOINT_ROOT, "Xform")
    joint_path = _simulation_grasp_joint_path(robot, asset_id)
    if stage.GetPrimAtPath(joint_path):
        raise ValueError(f"Grasp joint already exists: {joint_path}")

    body_path = f"{robot.prim_path}/{PIPER_WRIST_LINK}"
    if relative_pose is None:
        body_position, body_quaternion = _xform_world_pose(
            body_path,
            f"{robot.name}_{asset_id}_grasp_body",
        )
        doll_position, doll_quaternion = doll.get_world_pose()
        relative_pose = pose_relative_to(
            PoseSpec(tuple(body_position), tuple(body_quaternion)),
            PoseSpec(
                tuple(float(value) for value in doll_position),
                tuple(float(value) for value in doll_quaternion),
            ),
        )

    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(_doll_prim_path(asset_id))])
    joint.CreateLocalPos0Attr(
        Gf.Vec3f(*(float(value) for value in relative_pose.position))
    )
    joint.CreateLocalRot0Attr(
        Gf.Quatf(
            float(relative_pose.quaternion[0]),
            Gf.Vec3f(
                *(float(value) for value in relative_pose.quaternion[1:])
            ),
        )
    )
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    joint.CreateCollisionEnabledAttr(False)
    if step_after_change:
        world.step(render=False)
    return {
        "joint_path": joint_path,
        "body0": body_path,
        "body1": _doll_prim_path(asset_id),
        "body0_to_doll_pose": asdict(relative_pose),
        "collision_enabled": False,
    }


def remove_simulation_grasp_joint(
    world: Any,
    robot: RobotSpec,
    asset_id: str,
    *,
    step_after_change: bool = True,
) -> None:
    joint_path = _simulation_grasp_joint_path(robot, asset_id)
    if not world.stage.GetPrimAtPath(joint_path):
        raise ValueError(f"Grasp joint does not exist: {joint_path}")
    if not world.stage.RemovePrim(joint_path):
        raise RuntimeError(f"Failed to remove grasp joint {joint_path}")
    if step_after_change:
        world.step(render=False)


def run_curobo_motion_smoke(
    world: Any,
    robots: dict[str, Any],
    *,
    seed: int,
    render: bool,
) -> dict[str, Any]:
    """Execute one real cuRobo reach and return trajectory in Isaac Sim."""

    import numpy as np

    active_spec = LEFT_PIPER
    other_spec = RIGHT_PIPER
    active_robot = robots[active_spec.name]
    other_robot = robots[other_spec.name]
    initial = validate_robots_at_home(robots, label="curobo_initial")
    world_goal = (-0.18, -0.02, 0.96)

    with CuroboPlannerWorker(seed) as worker:
        reach_plan = worker.plan_position(
            active_robot=active_spec,
            other_robot=other_spec,
            current_joint_position=active_robot.get_joint_positions()[:6],
            other_joint_position=other_robot.get_joint_positions()[:6],
            world_goal_position=world_goal,
        )
        reach_execution = execute_curobo_trajectory(
            world,
            active_robot,
            reach_plan,
            render=render,
        )
        reached_pose = _piper_finger_center_world_pose(
            active_spec,
            "curobo_reached",
        )
        reached_position = reached_pose.position
        reached_quaternion = reached_pose.quaternion
        position_error = float(
            np.linalg.norm(
                np.asarray(reached_position, dtype=np.float64)
                - np.asarray(world_goal, dtype=np.float64)
            )
        )
        if position_error > 0.02:
            raise RuntimeError(
                f"Executed cuRobo reach missed by {position_error:.6f} m"
            )

        return_plan = worker.plan_joint(
            active_robot=active_spec,
            other_robot=other_spec,
            current_joint_position=active_robot.get_joint_positions()[:6],
            other_joint_position=other_robot.get_joint_positions()[:6],
            goal_joint_position=PIPER_HOME_JOINT_POSITION,
        )
        return_execution = execute_curobo_trajectory(
            world,
            active_robot,
            return_plan,
            render=render,
        )
        diagnostics = list(worker.diagnostic_lines)

    final = validate_robots_at_home(robots, label="curobo_final")
    return {
        "planner": "cuRobo MotionPlanner in isolated Warp process",
        "planner_seed": seed,
        "world_goal_position_m": list(world_goal),
        "reached_tool_world_pose": {
            "position": reached_position,
            "quaternion": reached_quaternion,
        },
        "reach_position_error_m": position_error,
        "reach_plan": reach_plan,
        "reach_execution": reach_execution,
        "return_plan": return_plan,
        "return_execution": return_execution,
        "worker_diagnostic_tail": diagnostics[-20:],
        "initial_robots": initial,
        "final_robots": final,
    }


def _current_doll_poses(dolls: dict[str, Any]) -> dict[str, PoseSpec]:
    states = _doll_state_report(dolls)
    return {
        asset_id: PoseSpec(
            tuple(state["position_m"]),  # type: ignore[arg-type]
            tuple(state["quaternion_wxyz"]),  # type: ignore[arg-type]
        )
        for asset_id, state in states.items()
    }


def validate_grasp_before_attach(
    doll_spec: DollSpec,
    *,
    center_error_m: float,
    axial_center_error_m: float = 0.0,
    finger_separation_m: float,
) -> dict[str, float]:
    """Reject attachment unless a doll lies between and along the fingers."""

    if not math.isfinite(center_error_m) or not math.isfinite(
        axial_center_error_m
    ) or not math.isfinite(
        finger_separation_m
    ):
        raise RuntimeError(
            f"{doll_spec.asset_id}: non-finite physical grasp measurement"
        )
    center_tolerance = min(
        PIPER_GRASP_MAX_CENTER_TOLERANCE_M,
        max(
            PIPER_GRASP_CENTER_TOLERANCE_M,
            2.0
            * doll_spec.footprint_radius
            * PIPER_GRASP_CENTER_DIAMETER_FRACTION,
        ),
    )
    if center_error_m > center_tolerance:
        raise RuntimeError(
            f"{doll_spec.asset_id}: physical finger centre missed the doll "
            f"contact point by {center_error_m:.6f} m; "
            f"allowed={center_tolerance:.6f} m, "
            f"finger_separation={finger_separation_m:.6f} m"
        )
    if axial_center_error_m > PIPER_GRASP_MAX_AXIAL_ERROR_M:
        raise RuntimeError(
            f"{doll_spec.asset_id}: doll lies outside the effective finger "
            f"length; axial_error={axial_center_error_m:.6f} m, "
            f"allowed={PIPER_GRASP_MAX_AXIAL_ERROR_M:.6f} m"
        )
    minimum_captured_separation = max(
        PIPER_GRASP_MIN_SEPARATION_M,
        2.0
        * doll_spec.footprint_radius
        * PIPER_GRASP_MIN_DIAMETER_FRACTION,
    )
    if finger_separation_m < minimum_captured_separation:
        raise RuntimeError(
            f"{doll_spec.asset_id}: gripper closed through the grasp without "
            "trapping the doll; "
            f"separation={finger_separation_m:.6f} m, "
            f"required>={minimum_captured_separation:.6f} m"
        )
    maximum_closed_separation = PIPER_OPEN_FINGER_SEPARATION_M - 0.005
    if finger_separation_m >= maximum_closed_separation:
        raise RuntimeError(
            f"{doll_spec.asset_id}: gripper remained open at the doll; "
            f"separation={finger_separation_m:.6f} m"
        )
    return {
        "center_tolerance_m": center_tolerance,
        "maximum_axial_center_error_m": PIPER_GRASP_MAX_AXIAL_ERROR_M,
        "minimum_captured_separation_m": minimum_captured_separation,
        "maximum_closed_separation_m": maximum_closed_separation,
    }


def run_curobo_pick_place_smoke(
    world: Any,
    robots: dict[str, Any],
    dolls: dict[str, Any],
    *,
    planner_seed: int,
    render: bool,
    asset_id: str = PICK_SMOKE_ASSET_ID,
    active_robot_name: str = "left",
    episode_recorder: Any | None = None,
) -> dict[str, Any]:
    """Pick, transport, place, and return home with one actual Piper."""

    import numpy as np

    active_spec = _robot_spec_by_name(active_robot_name)
    other_spec = next(
        spec for spec in ROBOT_SPECS if spec.name != active_spec.name
    )
    active_robot = robots[active_spec.name]
    other_robot = robots[other_spec.name]
    doll = dolls[asset_id]
    specs_by_id = {spec.asset_id: spec for spec in get_doll_specs()}
    doll_spec = specs_by_id[asset_id]
    grasp_contact_height = piper_grasp_contact_height(doll_spec)
    preplace_clearance = piper_preplace_clearance(doll_spec)
    target_by_id = {
        placement.asset_id: placement
        for placement in compute_doll_target_layout()
    }
    target_center = target_by_id[asset_id].pose.position
    planned_place_center = piper_planned_place_center(target_center)
    initial_robots = validate_robots_at_home(
        robots,
        label="pick_place_initial",
    )
    initial_doll_poses = _current_doll_poses(dolls)
    initial_doll_pose = initial_doll_poses[asset_id]

    plans: dict[str, Any] = {}
    executions: dict[str, Any] = {}
    grasp_joint: dict[str, Any] | None = None
    attachment: dict[str, Any] | None = None

    with CuroboPlannerWorker(planner_seed) as worker:
        initial_contact_offset = _quaternion_rotate_vector(
            initial_doll_pose.quaternion,
            (0.0, 0.0, grasp_contact_height),
        )
        initial_contact_point = tuple(
            initial_doll_pose.position[index]
            + initial_contact_offset[index]
            for index in range(3)
        )
        target_orientation_probe_goal = (
            target_center[0],
            target_center[1],
            target_center[2] + preplace_clearance,
        )
        plans["target_orientation_probe"] = worker.plan_position(
            active_robot=active_spec,
            other_robot=other_spec,
            current_joint_position=active_robot.get_joint_positions()[:6],
            other_joint_position=other_robot.get_joint_positions()[:6],
            world_goal_position=target_orientation_probe_goal,
            doll_poses=_current_doll_poses(dolls),
        )
        target_orientation = _pose_spec_from_mapping(
            plans["target_orientation_probe"]["selected_world_tool_pose"]
        ).quaternion

        # Solve the physical grasp configuration first, then approach along
        # that pose's local finger axis.  A world-Z descent can sweep a finger
        # sideways through the lightweight doll when the selected wrist pose
        # is oblique.
        grasp_candidates: list[tuple[dict[str, Any], PoseSpec]] = []
        grasp_search_failures: list[str] = []
        selected_grasp_seed: dict[str, Any] | None = None
        selected_pregrasp_plan: dict[str, Any] | None = None
        grasp_pose: PoseSpec | None = None
        selected_pregrasp_clearance: float | None = None
        (
            minimum_downward_axis_component,
            grasp_seed_candidate_count,
        ) = piper_grasp_search_parameters(doll_spec)
        large_doll_grasp_search = (
            grasp_seed_candidate_count
            > PIPER_GRASP_SEED_CANDIDATES
        )
        for candidate_index in range(grasp_seed_candidate_count):
            try:
                candidate_seed = worker.plan_position(
                    active_robot=active_spec,
                    other_robot=other_spec,
                    current_joint_position=active_robot.get_joint_positions()[
                        :6
                    ],
                    other_joint_position=other_robot.get_joint_positions()[:6],
                    world_goal_position=initial_contact_point,
                    doll_poses=_current_doll_poses(dolls),
                    excluded_doll_ids=(asset_id,),
                    preferred_world_orientation=target_orientation,
                )
            except RuntimeError as seed_error:
                grasp_search_failures.append(
                    f"seed {candidate_index}: {seed_error}"
                )
                continue
            candidate_pose = _pose_spec_from_mapping(
                candidate_seed["selected_world_tool_pose"]
            )
            candidate_closing_axis_world_z = _quaternion_rotate_vector(
                candidate_pose.quaternion,
                (0.0, 1.0, 0.0),
            )[2]
            candidate_seed["tool_y_world_z"] = (
                candidate_closing_axis_world_z
            )
            if (
                -float(candidate_seed["tool_x_world_z"])
                < minimum_downward_axis_component
            ):
                grasp_search_failures.append(
                    f"seed {candidate_index}: tool axis downward component "
                    f"{-float(candidate_seed['tool_x_world_z']):.6f} is "
                    f"below {minimum_downward_axis_component:.6f}"
                )
                continue
            if large_doll_grasp_search:
                for (
                    rolled_orientation,
                    roll_adjustment,
                ) in piper_horizontal_closing_orientation_candidates(
                    candidate_pose.quaternion
                ):
                    rolled_pose = PoseSpec(
                        candidate_pose.position,
                        rolled_orientation,
                    )
                    rolled_seed = dict(candidate_seed)
                    rolled_seed["source_tool_y_world_z"] = (
                        candidate_closing_axis_world_z
                    )
                    rolled_seed["wrist_roll_adjustment_rad"] = (
                        roll_adjustment
                    )
                    rolled_seed["selected_world_tool_pose"] = asdict(
                        rolled_pose
                    )
                    rolled_seed["tool_x_world_z"] = (
                        _quaternion_rotate_vector(
                            rolled_orientation,
                            (1.0, 0.0, 0.0),
                        )[2]
                    )
                    rolled_seed["tool_y_world_z"] = (
                        _quaternion_rotate_vector(
                            rolled_orientation,
                            (0.0, 1.0, 0.0),
                        )[2]
                    )
                    if (
                        abs(float(rolled_seed["tool_y_world_z"]))
                        > PIPER_LARGE_DOLL_MAX_CLOSING_AXIS_WORLD_Z
                    ):
                        raise RuntimeError(
                            "Wrist-roll compensation did not make the "
                            "large-doll closing axis horizontal"
                        )
                    grasp_candidates.append((rolled_seed, rolled_pose))
                continue
            regular_candidates = [(candidate_seed, candidate_pose)]
            if (
                abs(candidate_closing_axis_world_z)
                > PIPER_REGULAR_DOLL_MAX_CLOSING_AXIS_WORLD_Z
            ):
                regular_candidates = []
                for (
                    rolled_orientation,
                    roll_adjustment,
                ) in piper_horizontal_closing_orientation_candidates(
                    candidate_pose.quaternion
                ):
                    rolled_pose = PoseSpec(
                        candidate_pose.position,
                        rolled_orientation,
                    )
                    rolled_seed = dict(candidate_seed)
                    rolled_seed["source_tool_y_world_z"] = (
                        candidate_closing_axis_world_z
                    )
                    rolled_seed["wrist_roll_adjustment_rad"] = (
                        roll_adjustment
                    )
                    rolled_seed["selected_world_tool_pose"] = asdict(
                        rolled_pose
                    )
                    rolled_seed["tool_x_world_z"] = (
                        _quaternion_rotate_vector(
                            rolled_orientation,
                            (1.0, 0.0, 0.0),
                        )[2]
                    )
                    rolled_seed["tool_y_world_z"] = (
                        _quaternion_rotate_vector(
                            rolled_orientation,
                            (0.0, 1.0, 0.0),
                        )[2]
                    )
                    regular_candidates.append((rolled_seed, rolled_pose))
            for regular_seed, regular_pose in regular_candidates:
                grasp_candidates.append((regular_seed, regular_pose))
                try:
                    candidate_pregrasp = worker.plan_pose(
                        active_robot=active_spec,
                        other_robot=other_spec,
                        current_joint_position=(
                            active_robot.get_joint_positions()[:6]
                        ),
                        other_joint_position=(
                            other_robot.get_joint_positions()[:6]
                        ),
                        world_goal=piper_axis_approach_pose(
                            regular_pose,
                            PIPER_PREGRASP_CLEARANCE_M,
                        ),
                        doll_poses=_current_doll_poses(dolls),
                    )
                except RuntimeError as pregrasp_error:
                    grasp_search_failures.append(
                        f"seed {candidate_index} at "
                        f"{PIPER_PREGRASP_CLEARANCE_M:.3f} m, wrist roll "
                        f"{float(regular_seed.get('wrist_roll_adjustment_rad', 0.0)):.6f}: "
                        f"{pregrasp_error}"
                    )
                    continue
                selected_grasp_seed = regular_seed
                selected_pregrasp_plan = candidate_pregrasp
                grasp_pose = regular_pose
                selected_pregrasp_clearance = (
                    PIPER_PREGRASP_CLEARANCE_M
                )
                break
            if selected_grasp_seed is not None:
                break

        if large_doll_grasp_search:
            grasp_candidates.sort(
                key=lambda candidate: (
                    abs(
                        float(
                            candidate[0]["wrist_roll_adjustment_rad"]
                        )
                    ),
                    float(candidate[0]["tool_x_world_z"]),
                )
            )

            def nominal_upright_preplace_ik(
                candidate_pose: PoseSpec,
            ) -> tuple[PoseSpec, dict[str, Any]] | None:
                terminal_pose = piper_axis_approach_pose(
                    candidate_pose,
                    piper_final_approach_clearances(doll_spec)[-1],
                )
                nominal_object_orientation = upright_yaw_quaternion(
                    initial_doll_pose.quaternion
                )
                for yaw_offset in PIPER_UPRIGHT_YAW_OFFSETS_RAD:
                    yaw_rotation = (
                        math.cos(0.5 * yaw_offset),
                        0.0,
                        0.0,
                        math.sin(0.5 * yaw_offset),
                    )
                    desired_object_pose = PoseSpec(
                        (
                            target_center[0],
                            target_center[1],
                            target_center[2] + preplace_clearance,
                        ),
                        quaternion_multiply(
                            yaw_rotation,
                            nominal_object_orientation,
                        ),
                    )
                    candidate_goal = tool_pose_for_attached_object_pose(
                        terminal_pose,
                        initial_doll_pose,
                        desired_object_pose,
                    )
                    ik_report = worker.check_pose(
                        active_robot=active_spec,
                        other_robot=other_spec,
                        current_joint_position=(
                            active_robot.get_joint_positions()[:6]
                        ),
                        other_joint_position=(
                            other_robot.get_joint_positions()[:6]
                        ),
                        world_goal=candidate_goal,
                        doll_poses=_current_doll_poses(dolls),
                        excluded_doll_ids=(asset_id,),
                    )
                    if bool(ik_report["success"]):
                        return candidate_goal, ik_report
                return None

            for candidate_index, (
                candidate_seed,
                candidate_pose,
            ) in enumerate(grasp_candidates):
                try:
                    candidate_pregrasp = worker.plan_pose(
                        active_robot=active_spec,
                        other_robot=other_spec,
                        current_joint_position=(
                            active_robot.get_joint_positions()[:6]
                        ),
                        other_joint_position=(
                            other_robot.get_joint_positions()[:6]
                        ),
                        world_goal=piper_axis_approach_pose(
                            candidate_pose,
                            PIPER_PREGRASP_CLEARANCE_M,
                        ),
                        doll_poses=_current_doll_poses(dolls),
                    )
                except RuntimeError as pregrasp_error:
                    grasp_search_failures.append(
                        f"ranked candidate {candidate_index} at "
                        f"{PIPER_PREGRASP_CLEARANCE_M:.3f} m: "
                        f"{pregrasp_error}"
                    )
                    continue
                preplace_ik = nominal_upright_preplace_ik(
                    candidate_pose
                )
                if preplace_ik is None:
                    grasp_search_failures.append(
                        f"ranked candidate {candidate_index}: no exact "
                        "upright IK at the large-doll preplace"
                    )
                    continue
                (
                    nominal_preplace_goal,
                    nominal_preplace_ik_report,
                ) = preplace_ik
                candidate_seed["nominal_upright_preplace_goal"] = asdict(
                    nominal_preplace_goal
                )
                candidate_seed["nominal_upright_preplace_ik"] = (
                    nominal_preplace_ik_report
                )
                selected_grasp_seed = candidate_seed
                selected_pregrasp_plan = candidate_pregrasp
                grasp_pose = candidate_pose
                selected_pregrasp_clearance = (
                    PIPER_PREGRASP_CLEARANCE_M
                )
                break

        if selected_grasp_seed is None:
            for clearance_m in (
                PIPER_PREGRASP_CLEARANCE_CANDIDATES_M[1:]
            ):
                for candidate_index, (
                    candidate_seed,
                    candidate_pose,
                ) in enumerate(grasp_candidates):
                    try:
                        candidate_pregrasp = worker.plan_pose(
                            active_robot=active_spec,
                            other_robot=other_spec,
                            current_joint_position=(
                                active_robot.get_joint_positions()[:6]
                            ),
                            other_joint_position=(
                                other_robot.get_joint_positions()[:6]
                            ),
                            world_goal=piper_axis_approach_pose(
                                candidate_pose,
                                clearance_m,
                            ),
                            doll_poses=_current_doll_poses(dolls),
                        )
                    except RuntimeError as pregrasp_error:
                        grasp_search_failures.append(
                            f"candidate {candidate_index} at "
                            f"{clearance_m:.3f} m: {pregrasp_error}"
                        )
                        continue
                    if large_doll_grasp_search:
                        preplace_ik = nominal_upright_preplace_ik(
                            candidate_pose
                        )
                        if preplace_ik is None:
                            grasp_search_failures.append(
                                f"candidate {candidate_index} at "
                                f"{clearance_m:.3f} m: no exact upright "
                                "IK at the large-doll preplace"
                            )
                            continue
                        (
                            nominal_preplace_goal,
                            nominal_preplace_ik_report,
                        ) = preplace_ik
                        candidate_seed[
                            "nominal_upright_preplace_goal"
                        ] = asdict(nominal_preplace_goal)
                        candidate_seed[
                            "nominal_upright_preplace_ik"
                        ] = nominal_preplace_ik_report
                    selected_grasp_seed = candidate_seed
                    selected_pregrasp_plan = candidate_pregrasp
                    grasp_pose = candidate_pose
                    selected_pregrasp_clearance = clearance_m
                    break
                if selected_grasp_seed is not None:
                    break

        if (
            selected_grasp_seed is None
            or selected_pregrasp_plan is None
            or grasp_pose is None
            or selected_pregrasp_clearance is None
        ):
            raise RuntimeError(
                f"{asset_id}: no collision-free axis pregrasp across "
                f"{len(grasp_candidates)} grasp poses; "
                f"failures={grasp_search_failures}"
            )
        plans["grasp_seed"] = selected_grasp_seed
        plans["pregrasp"] = selected_pregrasp_plan
        plans["grasp_seed_search"] = {
            "candidate_count": len(grasp_candidates),
            "requested_candidate_count": grasp_seed_candidate_count,
            "minimum_downward_axis_component": (
                minimum_downward_axis_component
            ),
            "selected_pregrasp_clearance_m": (
                selected_pregrasp_clearance
            ),
            "failures": grasp_search_failures,
        }
        print(
            "CUROBO_GRASP_PLAN "
            f"asset={asset_id} robot={active_spec.name} "
            f"contact_height_m={grasp_contact_height:.6f} "
            f"tool_x_world_z="
            f"{float(selected_grasp_seed['tool_x_world_z']):.6f} "
            f"tool_y_world_z="
            f"{float(selected_grasp_seed['tool_y_world_z']):.6f} "
            f"wrist_roll_adjustment_rad="
            f"{float(selected_grasp_seed.get('wrist_roll_adjustment_rad', 0.0)):.6f} "
            f"pregrasp_clearance_m={selected_pregrasp_clearance:.6f}",
            flush=True,
        )
        _set_recording_context(
            episode_recorder,
            "pregrasp",
            operator=active_spec.name,
            object_id=asset_id,
        )
        executions["pregrasp"] = execute_curobo_trajectory(
            world,
            active_robot,
            plans["pregrasp"],
            render=render,
            episode_recorder=episode_recorder,
            final_tolerance_rad=CUROBO_GRASP_EXECUTION_TOLERANCE_RAD,
            final_settle_max_control_frames=(
                CUROBO_GRASP_SETTLE_MAX_CONTROL_FRAMES
            ),
        )

        plans["near_grasp"] = worker.plan_pose(
            active_robot=active_spec,
            other_robot=other_spec,
            current_joint_position=active_robot.get_joint_positions()[:6],
            other_joint_position=other_robot.get_joint_positions()[:6],
            world_goal=piper_axis_approach_pose(
                grasp_pose,
                PIPER_NEAR_GRASP_CLEARANCE_M,
            ),
            doll_poses=_current_doll_poses(dolls),
        )
        _set_recording_context(
            episode_recorder,
            "pregrasp",
            operator=active_spec.name,
            object_id=asset_id,
        )
        executions["near_grasp"] = execute_curobo_trajectory(
            world,
            active_robot,
            plans["near_grasp"],
            render=render,
            episode_recorder=episode_recorder,
            final_tolerance_rad=CUROBO_GRASP_EXECUTION_TOLERANCE_RAD,
            final_settle_max_control_frames=(
                CUROBO_GRASP_SETTLE_MAX_CONTROL_FRAMES
            ),
        )

        def measure_axis_alignment(
            clearance_m: float,
            label: str,
        ) -> tuple[PoseSpec, float]:
            live_doll_pose = _current_doll_poses(dolls)[asset_id]
            live_contact_offset = _quaternion_rotate_vector(
                live_doll_pose.quaternion,
                (0.0, 0.0, grasp_contact_height),
            )
            live_contact_point = tuple(
                live_doll_pose.position[index]
                + live_contact_offset[index]
                for index in range(3)
            )
            expected_pose = piper_axis_approach_pose(
                PoseSpec(live_contact_point, grasp_pose.quaternion),
                clearance_m,
            )
            measured_pose = _piper_finger_center_world_pose(
                active_spec,
                f"{asset_id}_{label}",
            )
            alignment_error = float(
                np.linalg.norm(
                    np.asarray(
                        measured_pose.position,
                        dtype=np.float64,
                    )
                    - np.asarray(
                        expected_pose.position,
                        dtype=np.float64,
                    )
                )
            )
            return expected_pose, alignment_error

        near_alignment_pose, near_alignment_error = measure_axis_alignment(
            PIPER_NEAR_GRASP_CLEARANCE_M,
            "near_grasp_alignment",
        )
        initial_near_alignment_error = near_alignment_error
        if near_alignment_error > PIPER_GRASP_CORRECTION_TRIGGER_M:
            plans["near_grasp_correction"] = worker.plan_pose(
                active_robot=active_spec,
                other_robot=other_spec,
                current_joint_position=active_robot.get_joint_positions()[:6],
                other_joint_position=other_robot.get_joint_positions()[:6],
                world_goal=near_alignment_pose,
                doll_poses=_current_doll_poses(dolls),
            )
            executions["near_grasp_correction"] = execute_curobo_trajectory(
                world,
                active_robot,
                plans["near_grasp_correction"],
                render=render,
                episode_recorder=episode_recorder,
                final_tolerance_rad=CUROBO_GRASP_EXECUTION_TOLERANCE_RAD,
                final_settle_max_control_frames=(
                    CUROBO_GRASP_SETTLE_MAX_CONTROL_FRAMES
                ),
            )
            (
                near_alignment_pose,
                near_alignment_error,
            ) = measure_axis_alignment(
                PIPER_NEAR_GRASP_CLEARANCE_M,
                "near_grasp_corrected",
            )
        print(
            "CUROBO_NEAR_GRASP_ALIGNMENT "
            f"asset={asset_id} robot={active_spec.name} "
            f"initial_error_m={initial_near_alignment_error:.6f} "
            f"final_error_m={near_alignment_error:.6f}",
            flush=True,
        )
        if near_alignment_error > PIPER_NEAR_GRASP_ALIGNMENT_TOLERANCE_M:
            raise RuntimeError(
                f"{asset_id}: finger centre is not aligned above the doll "
                f"before insertion; error={near_alignment_error:.6f} m, "
                f"allowed={PIPER_NEAR_GRASP_ALIGNMENT_TOLERANCE_M:.6f} m"
            )

        final_approach_clearances = piper_final_approach_clearances(
            doll_spec
        )
        terminal_grasp_clearance = final_approach_clearances[-1]
        for final_step, clearance_m in enumerate(
            final_approach_clearances,
            start=1,
        ):
            plan_key = (
                "grasp"
                if final_step == len(final_approach_clearances)
                else f"grasp_approach_{final_step}"
            )
            plans[plan_key] = worker.plan_pose(
                active_robot=active_spec,
                other_robot=other_spec,
                current_joint_position=active_robot.get_joint_positions()[:6],
                other_joint_position=other_robot.get_joint_positions()[:6],
                world_goal=piper_axis_approach_pose(
                    grasp_pose,
                    clearance_m,
                ),
                doll_poses=_current_doll_poses(dolls),
                excluded_doll_ids=(asset_id,),
            )
            _set_recording_context(
                episode_recorder,
                "grasp",
                operator=active_spec.name,
                object_id=asset_id,
            )
            executions[plan_key] = execute_curobo_trajectory(
                world,
                active_robot,
                plans[plan_key],
                render=render,
                episode_recorder=episode_recorder,
                final_tolerance_rad=CUROBO_GRASP_EXECUTION_TOLERANCE_RAD,
                final_settle_max_control_frames=(
                    CUROBO_GRASP_SETTLE_MAX_CONTROL_FRAMES
                ),
            )
            _, step_alignment_error = measure_axis_alignment(
                clearance_m,
                f"{plan_key}_complete",
            )
            step_doll_pose = _current_doll_poses(dolls)[asset_id]
            step_doll_displacement = float(
                np.linalg.norm(
                    np.asarray(
                        step_doll_pose.position,
                        dtype=np.float64,
                    )
                    - np.asarray(
                        initial_doll_pose.position,
                        dtype=np.float64,
                    )
                )
            )
            step_up = _quaternion_rotate_vector(
                step_doll_pose.quaternion,
                (0.0, 0.0, 1.0),
            )
            step_tilt = math.degrees(
                math.acos(
                    min(1.0, max(-1.0, float(step_up[2])))
                )
            )
            print(
                "CUROBO_GRASP_INSERTION "
                f"asset={asset_id} robot={active_spec.name} "
                f"clearance_m={clearance_m:.6f} "
                f"alignment_error_m={step_alignment_error:.6f} "
                f"doll_displacement_m={step_doll_displacement:.6f} "
                f"doll_tilt_deg={step_tilt:.6f}",
                flush=True,
            )
            if (
                step_doll_displacement
                > PIPER_APPROACH_MAX_DOLL_DISPLACEMENT_M
            ):
                raise RuntimeError(
                    f"{asset_id}: doll moved during open-finger insertion; "
                    f"clearance={clearance_m:.6f} m, "
                    f"displacement={step_doll_displacement:.6f} m, "
                    f"allowed="
                    f"{PIPER_APPROACH_MAX_DOLL_DISPLACEMENT_M:.6f} m"
                )
        def measure_grasp_approach(
            label: str,
        ) -> tuple[PoseSpec, PoseSpec, tuple[float, float, float], float]:
            measured_finger_pose = _piper_finger_center_world_pose(
                active_spec,
                f"{asset_id}_{label}",
            )
            measured_doll_pose = _current_doll_poses(dolls)[asset_id]
            measured_contact_offset = _quaternion_rotate_vector(
                measured_doll_pose.quaternion,
                (0.0, 0.0, grasp_contact_height),
            )
            measured_contact_point = tuple(
                measured_doll_pose.position[index]
                + measured_contact_offset[index]
                for index in range(3)
            )
            measured_contact_point = piper_axis_approach_pose(
                PoseSpec(
                    measured_contact_point,
                    grasp_pose.quaternion,
                ),
                terminal_grasp_clearance,
            ).position
            measured_error = float(
                np.linalg.norm(
                    np.asarray(
                        measured_finger_pose.position,
                        dtype=np.float64,
                    )
                    - np.asarray(
                        measured_contact_point,
                        dtype=np.float64,
                    )
                )
            )
            return (
                measured_finger_pose,
                measured_doll_pose,
                measured_contact_point,
                measured_error,
            )

        (
            approach_finger_pose,
            approach_doll_pose,
            approach_contact_point,
            approach_center_error,
        ) = measure_grasp_approach("approach_complete")
        initial_approach_center_error = approach_center_error
        if (
            approach_center_error
            > PIPER_GRASP_CORRECTION_TRIGGER_M
            and terminal_grasp_clearance == 0.0
        ):
            plans["grasp_correction"] = worker.plan_pose(
                active_robot=active_spec,
                other_robot=other_spec,
                current_joint_position=active_robot.get_joint_positions()[:6],
                other_joint_position=other_robot.get_joint_positions()[:6],
                world_goal=PoseSpec(
                    approach_contact_point,
                    approach_finger_pose.quaternion,
                ),
                doll_poses=_current_doll_poses(dolls),
                excluded_doll_ids=(asset_id,),
            )
            _set_recording_context(
                episode_recorder,
                "grasp",
                operator=active_spec.name,
                object_id=asset_id,
            )
            executions["grasp_correction"] = execute_curobo_trajectory(
                world,
                active_robot,
                plans["grasp_correction"],
                render=render,
                episode_recorder=episode_recorder,
                final_tolerance_rad=CUROBO_GRASP_EXECUTION_TOLERANCE_RAD,
                final_settle_max_control_frames=(
                    CUROBO_GRASP_SETTLE_MAX_CONTROL_FRAMES
                ),
            )
            (
                approach_finger_pose,
                approach_doll_pose,
                approach_contact_point,
                approach_center_error,
            ) = measure_grasp_approach("approach_corrected")
        approach_displacement = float(
            np.linalg.norm(
                np.asarray(
                    approach_doll_pose.position,
                    dtype=np.float64,
                )
                - np.asarray(
                    initial_doll_pose.position,
                    dtype=np.float64,
                )
            )
        )
        approach_up = _quaternion_rotate_vector(
            approach_doll_pose.quaternion,
            (0.0, 0.0, 1.0),
        )
        approach_tilt = math.degrees(
            math.acos(min(1.0, max(-1.0, float(approach_up[2]))))
        )
        print(
            "CUROBO_GRASP_APPROACH "
            f"asset={asset_id} robot={active_spec.name} "
            f"initial_error_m={initial_approach_center_error:.6f} "
            f"final_error_m={approach_center_error:.6f} "
            f"doll_displacement_m={approach_displacement:.6f} "
            f"doll_tilt_deg={approach_tilt:.6f}",
            flush=True,
        )
        if approach_center_error > PIPER_GRASP_CENTER_TOLERANCE_M:
            raise RuntimeError(
                f"{asset_id}: grasp approach missed before closing; "
                f"finger_center={approach_finger_pose.position}, "
                f"contact_point={approach_contact_point}, "
                f"initial_error={initial_approach_center_error:.6f} m, "
                f"corrected_error={approach_center_error:.6f} m, "
                f"doll_displacement={approach_displacement:.6f} m, "
                f"doll_tilt={approach_tilt:.6f} degrees"
            )

        _set_recording_context(
            episode_recorder,
            "close_gripper",
            operator=active_spec.name,
            object_id=asset_id,
        )
        if episode_recorder is not None:
            episode_recorder.set_gripper_action(
                active_robot,
                PIPER_CLOSED_GRIPPER_POSITION,
            )
        _command_position(
            active_robot,
            (PIPER_CLOSED_GRIPPER_POSITION,),
            (6,),
        )
        physics_steps_per_control = (
            PHYSICS_FREQUENCY_HZ // CONTROL_FREQUENCY_HZ
        )
        if PICK_GRIPPER_CLOSE_STEPS % physics_steps_per_control:
            raise ValueError("Gripper close duration is not a whole control frame")
        for _ in range(
            PICK_GRIPPER_CLOSE_STEPS // physics_steps_per_control
        ):
            _advance_control_frame(
                world,
                render=render,
                episode_recorder=episode_recorder,
            )

        finger_center_pose = _piper_finger_center_world_pose(
            active_spec,
            f"{asset_id}_pick_grasp",
        )
        closed_doll_poses = _current_doll_poses(dolls)
        closed_doll_pose = closed_doll_poses[asset_id]
        finger_center = finger_center_pose.position
        closed_contact_offset = _quaternion_rotate_vector(
            closed_doll_pose.quaternion,
            (0.0, 0.0, grasp_contact_height),
        )
        closed_contact_point = tuple(
            closed_doll_pose.position[index]
            + closed_contact_offset[index]
            for index in range(3)
        )
        closed_contact_point = piper_axis_approach_pose(
            PoseSpec(
                closed_contact_point,
                grasp_pose.quaternion,
            ),
            terminal_grasp_clearance,
        ).position
        closed_center_delta = (
            np.asarray(finger_center, dtype=np.float64)
            - np.asarray(closed_contact_point, dtype=np.float64)
        )
        closed_tool_axis = np.asarray(
            _quaternion_rotate_vector(
                finger_center_pose.quaternion,
                (1.0, 0.0, 0.0),
            ),
            dtype=np.float64,
        )
        signed_axial_center_error = float(
            np.dot(closed_center_delta, closed_tool_axis)
        )
        axial_center_error = abs(signed_axial_center_error)
        lateral_center_error = float(
            np.linalg.norm(
                closed_center_delta
                - signed_axial_center_error * closed_tool_axis
            )
        )
        grasp_center_error = float(np.linalg.norm(closed_center_delta))
        closed_separation = _finger_separation(
            active_spec,
            "pick_contact",
        )
        closed_doll_displacement = float(
            np.linalg.norm(
                np.asarray(
                    closed_doll_pose.position,
                    dtype=np.float64,
                )
                - np.asarray(
                    initial_doll_pose.position,
                    dtype=np.float64,
                )
            )
        )
        closed_up = _quaternion_rotate_vector(
            closed_doll_pose.quaternion,
            (0.0, 0.0, 1.0),
        )
        closed_tilt = math.degrees(
            math.acos(min(1.0, max(-1.0, float(closed_up[2]))))
        )
        print(
            "CUROBO_GRASP_PHYSICS "
            f"asset={asset_id} robot={active_spec.name} "
            f"initial_approach_error_m="
            f"{initial_approach_center_error:.6f} "
            f"approach_error_m={approach_center_error:.6f} "
            f"closed_error_m={grasp_center_error:.6f} "
            f"closed_lateral_error_m={lateral_center_error:.6f} "
            f"closed_axial_error_m={axial_center_error:.6f} "
            f"closed_doll_displacement_m="
            f"{closed_doll_displacement:.6f} "
            f"closed_doll_tilt_deg={closed_tilt:.6f} "
            f"closed_separation_m={closed_separation:.6f}",
            flush=True,
        )
        attachment_gate = validate_grasp_before_attach(
            doll_spec,
            center_error_m=lateral_center_error,
            axial_center_error_m=axial_center_error,
            finger_separation_m=closed_separation,
        )

        _set_recording_context(
            episode_recorder,
            "attach",
            operator=active_spec.name,
            object_id=asset_id,
        )
        grasp_joint = create_simulation_grasp_joint(
            world,
            active_spec,
            asset_id,
            doll,
            step_after_change=episode_recorder is None,
        )
        if episode_recorder is not None:
            episode_recorder.queue_grasp_event(
                GRASP_EVENT_ATTACH,
                active_spec.name,
                asset_id,
                relative_pose=_pose_spec_from_mapping(
                    grasp_joint["body0_to_doll_pose"]
                ),
            )
            _advance_control_frame(
                world,
                render=render,
                episode_recorder=episode_recorder,
            )
        attached_tool_pose = _piper_finger_center_world_pose(
            active_spec,
            f"{asset_id}_pick_attached",
        )
        attached_tool_position = attached_tool_pose.position
        attached_tool_quaternion = attached_tool_pose.quaternion
        attached_doll_pose = _current_doll_poses(dolls)[asset_id]
        attachment = worker.attach(
            active_robot=active_spec,
            current_joint_position=active_robot.get_joint_positions()[:6],
            asset_id=asset_id,
            doll_world_pose=attached_doll_pose,
        )
        attached_offset = tuple(
            attached_doll_pose.position[index]
            - attached_tool_position[index]
            for index in range(3)
        )
        transport_orientation = tuple(attached_tool_quaternion)
        tool_to_attached_doll = pose_relative_to(
            attached_tool_pose,
            attached_doll_pose,
        )

        lift_goal = PoseSpec(
            (
                attached_tool_position[0],
                attached_tool_position[1],
                attached_tool_position[2] + PIPER_LIFT_CLEARANCE_M,
            ),
            transport_orientation,
        )
        lift_mode = "exact_pose"
        try:
            plans["lift"] = worker.plan_pose(
                active_robot=active_spec,
                other_robot=other_spec,
                current_joint_position=active_robot.get_joint_positions()[:6],
                other_joint_position=other_robot.get_joint_positions()[:6],
                world_goal=lift_goal,
                doll_poses=_current_doll_poses(dolls),
                excluded_doll_ids=(asset_id,),
            )
        except RuntimeError as exact_error:
            lift_mode = "position_with_upright_object"
            plans["lift_exact_failure"] = {"error": str(exact_error)}
            plans["lift"] = worker.plan_position(
                active_robot=active_spec,
                other_robot=other_spec,
                current_joint_position=active_robot.get_joint_positions()[:6],
                other_joint_position=other_robot.get_joint_positions()[:6],
                world_goal_position=lift_goal.position,
                doll_poses=_current_doll_poses(dolls),
                excluded_doll_ids=(asset_id,),
                preferred_world_orientation=transport_orientation,
                tool_to_attached_object_orientation=(
                    tool_to_attached_doll.quaternion
                ),
                max_attached_object_tilt_degrees=(
                    PIPER_TRANSPORT_UPRIGHT_TOLERANCE_DEGREES
                ),
            )
            planned_lift_tilt = plans["lift"][
                "attached_object_tilt_degrees"
            ]
            if (
                planned_lift_tilt is None
                or planned_lift_tilt
                > PIPER_TRANSPORT_UPRIGHT_TOLERANCE_DEGREES
            ):
                raise RuntimeError(
                    f"{asset_id}: planned lift left object tilt at "
                    f"{planned_lift_tilt} degrees"
                )
        _set_recording_context(
            episode_recorder,
            "lift",
            operator=active_spec.name,
            object_id=asset_id,
        )
        executions["lift"] = execute_curobo_trajectory(
            world,
            active_robot,
            plans["lift"],
            render=render,
            episode_recorder=episode_recorder,
        )
        lifted_doll_pose = _current_doll_poses(dolls)[asset_id]
        lifted_tool_pose = _piper_finger_center_world_pose(
            active_spec,
            f"{asset_id}_lifted",
        )
        lifted_distance = (
            lifted_doll_pose.position[2] - attached_doll_pose.position[2]
        )
        if lifted_distance < PIPER_LIFT_CLEARANCE_M - 0.02:
            raise RuntimeError(
                f"{asset_id}: fixed grasp lifted only {lifted_distance:.6f} m"
            )
        lifted_tilt_before_upright = _doll_state_report(dolls)[asset_id][
            "upright_tilt_degrees"
        ]
        if (
            lifted_tilt_before_upright
            > PIPER_TRANSPORT_UPRIGHT_TOLERANCE_DEGREES
        ):
            raise RuntimeError(
                f"{asset_id}: doll tilted too far during lift to correct "
                f"safely: {lifted_tilt_before_upright:.6f} degrees"
            )
        print(
            "CUROBO_LIFT_STATE "
            f"asset={asset_id} robot={active_spec.name} "
            f"mode={lift_mode} "
            f"lifted_distance_m={lifted_distance:.6f} "
            f"tilt_deg={lifted_tilt_before_upright:.6f}",
            flush=True,
        )

        # The fixed grasp preserves the measured tool-to-object transform.
        # Use that transform to rotate the arm joints about the lifted doll's
        # centre until the doll's local Z axis is vertical.  If the source-side
        # wrist branch cannot rotate in place, keep the bounded transport tilt
        # and require the target-side preplace trajectory to end upright before
        # any descent.
        base_upright_orientation = upright_yaw_quaternion(
            lifted_doll_pose.quaternion
        )
        desired_upright_orientation = base_upright_orientation
        upright_failures: list[str] = []
        selected_upright_yaw_offset: float | None = None
        for yaw_offset in PIPER_UPRIGHT_YAW_OFFSETS_RAD:
            yaw_rotation = (
                math.cos(0.5 * yaw_offset),
                0.0,
                0.0,
                math.sin(0.5 * yaw_offset),
            )
            candidate_upright_orientation = quaternion_multiply(
                yaw_rotation,
                base_upright_orientation,
            )
            upright_goal = tool_pose_for_attached_object_orientation(
                lifted_tool_pose,
                lifted_doll_pose,
                candidate_upright_orientation,
            )
            try:
                plans["upright"] = worker.plan_pose(
                    active_robot=active_spec,
                    other_robot=other_spec,
                    current_joint_position=(
                        active_robot.get_joint_positions()[:6]
                    ),
                    other_joint_position=(
                        other_robot.get_joint_positions()[:6]
                    ),
                    world_goal=upright_goal,
                    doll_poses=_current_doll_poses(dolls),
                    excluded_doll_ids=(asset_id,),
                )
            except RuntimeError as upright_error:
                upright_failures.append(
                    f"yaw_offset={yaw_offset:.6f}: {upright_error}"
                )
                continue
            desired_upright_orientation = (
                candidate_upright_orientation
            )
            selected_upright_yaw_offset = yaw_offset
            break
        upright_correction_deferred_to_preplace = (
            selected_upright_yaw_offset is None
        )
        plans["upright_yaw_search"] = {
            "selected_yaw_offset_rad": selected_upright_yaw_offset,
            "failed_candidate_count": len(upright_failures),
            "failures": upright_failures,
            "deferred_to_preplace": (
                upright_correction_deferred_to_preplace
            ),
        }
        if upright_correction_deferred_to_preplace:
            plans["upright"] = {
                "skipped": True,
                "reason": "source_side_upright_yaw_unreachable",
                "success": True,
            }
            upright_doll_pose = lifted_doll_pose
            upright_tool_pose = lifted_tool_pose
            upright_center_drift = 0.0
            lifted_tilt = lifted_tilt_before_upright
        else:
            _set_recording_context(
                episode_recorder,
                "lift",
                operator=active_spec.name,
                object_id=asset_id,
            )
            executions["upright"] = execute_curobo_trajectory(
                world,
                active_robot,
                plans["upright"],
                render=render,
                episode_recorder=episode_recorder,
            )
            upright_doll_pose = _current_doll_poses(dolls)[asset_id]
            upright_tool_pose = _piper_finger_center_world_pose(
                active_spec,
                f"{asset_id}_upright",
            )
            upright_center_drift = float(
                np.linalg.norm(
                    np.asarray(
                        upright_doll_pose.position,
                        dtype=np.float64,
                    )
                    - np.asarray(
                        lifted_doll_pose.position,
                        dtype=np.float64,
                    )
                )
            )
            lifted_tilt = _doll_state_report(dolls)[asset_id][
                "upright_tilt_degrees"
            ]
        if (
            not upright_correction_deferred_to_preplace
            and lifted_tilt
            > PIPER_POST_GRASP_UPRIGHT_TOLERANCE_DEGREES
        ):
            raise RuntimeError(
                f"{asset_id}: post-grasp joint rotation left doll tilt at "
                f"{lifted_tilt:.6f} degrees"
            )
        if upright_center_drift > PIPER_GRASP_CENTER_TOLERANCE_M:
            raise RuntimeError(
                f"{asset_id}: post-grasp joint rotation displaced the doll "
                f"centre by {upright_center_drift:.6f} m"
            )
        print(
            "CUROBO_UPRIGHT_CORRECTION "
            f"asset={asset_id} robot={active_spec.name} "
            f"before_tilt_deg={lifted_tilt_before_upright:.6f} "
            f"after_tilt_deg={lifted_tilt:.6f} "
            f"center_drift_m={upright_center_drift:.6f} "
            f"deferred_to_preplace="
            f"{str(upright_correction_deferred_to_preplace).lower()}",
            flush=True,
        )
        attached_offset = tuple(
            upright_doll_pose.position[index]
            - upright_tool_pose.position[index]
            for index in range(3)
        )
        transport_orientation = tuple(upright_tool_pose.quaternion)

        preplace_object_center = (
            target_center[0],
            target_center[1],
            target_center[2] + preplace_clearance,
        )
        tool_to_upright_doll = pose_relative_to(
            upright_tool_pose,
            upright_doll_pose,
        )
        probe_object_orientation = quaternion_multiply(
            target_orientation,
            tool_to_upright_doll.quaternion,
        )
        probe_upright_orientation = upright_yaw_quaternion(
            probe_object_orientation
        )
        upright_orientation_candidates: list[
            tuple[float, float, float, float]
        ] = []
        for base_orientation in (
            probe_upright_orientation,
            desired_upright_orientation,
        ):
            for yaw_offset in PIPER_UPRIGHT_YAW_OFFSETS_RAD:
                yaw_rotation = (
                    math.cos(0.5 * yaw_offset),
                    0.0,
                    0.0,
                    math.sin(0.5 * yaw_offset),
                )
                candidate = quaternion_multiply(
                    yaw_rotation,
                    base_orientation,
                )
                if not any(
                    abs(
                        sum(
                            candidate[index] * existing[index]
                            for index in range(4)
                        )
                    )
                    > 1.0 - 1.0e-8
                    for existing in upright_orientation_candidates
                ):
                    upright_orientation_candidates.append(candidate)

        preplace_failures: list[str] = []
        preplace_goal: PoseSpec | None = None
        selected_object_orientation: tuple[
            float, float, float, float
        ] | None = None
        preplace_mode: str | None = None
        for candidate_index, object_orientation in enumerate(
            upright_orientation_candidates
        ):
            candidate_object_pose = PoseSpec(
                preplace_object_center,
                object_orientation,
            )
            candidate_goal = tool_pose_for_attached_object_pose(
                upright_tool_pose,
                upright_doll_pose,
                candidate_object_pose,
            )
            try:
                candidate_plan = worker.plan_pose(
                    active_robot=active_spec,
                    other_robot=other_spec,
                    current_joint_position=active_robot.get_joint_positions()[
                        :6
                    ],
                    other_joint_position=other_robot.get_joint_positions()[:6],
                    world_goal=candidate_goal,
                    doll_poses=_current_doll_poses(dolls),
                    excluded_doll_ids=(asset_id,),
                )
            except RuntimeError as exact_error:
                preplace_failures.append(
                    f"candidate {candidate_index} exact: {exact_error}"
                )
                try:
                    candidate_plan = worker.plan_position(
                        active_robot=active_spec,
                        other_robot=other_spec,
                        current_joint_position=(
                            active_robot.get_joint_positions()[:6]
                        ),
                        other_joint_position=(
                            other_robot.get_joint_positions()[:6]
                        ),
                        world_goal_position=candidate_goal.position,
                        doll_poses=_current_doll_poses(dolls),
                        excluded_doll_ids=(asset_id,),
                        preferred_world_orientation=(
                            candidate_goal.quaternion
                        ),
                        tool_to_attached_object_orientation=(
                            tool_to_upright_doll.quaternion
                        ),
                        max_attached_object_tilt_degrees=(
                            PIPER_PLANNED_UPRIGHT_TOLERANCE_DEGREES
                        ),
                    )
                except RuntimeError as position_error:
                    preplace_failures.append(
                        f"candidate {candidate_index} position: "
                        f"{position_error}"
                    )
                    continue
                selected_tool_pose = _pose_spec_from_mapping(
                    candidate_plan["selected_world_tool_pose"]
                )
                predicted_object_orientation = quaternion_multiply(
                    selected_tool_pose.quaternion,
                    tool_to_upright_doll.quaternion,
                )
                predicted_object_tilt = (
                    attached_object_upright_tilt_degrees(
                        selected_tool_pose.quaternion,
                        tool_to_upright_doll.quaternion,
                    )
                )
                if (
                    predicted_object_tilt
                    > PIPER_TRANSPORT_UPRIGHT_TOLERANCE_DEGREES
                ):
                    preplace_failures.append(
                        f"candidate {candidate_index} position changed "
                        f"predicted doll tilt to "
                        f"{predicted_object_tilt:.6f} degrees"
                    )
                    continue
                candidate_plan["predicted_object_tilt_degrees"] = (
                    predicted_object_tilt
                )
                candidate_plan["predicted_object_orientation"] = list(
                    predicted_object_orientation
                )
                preplace_goal = selected_tool_pose
                selected_object_orientation = (
                    predicted_object_orientation
                )
                preplace_mode = (
                    "position_with_bounded_transport_tilt"
                )
            else:
                preplace_goal = candidate_goal
                selected_object_orientation = object_orientation
                preplace_mode = "exact_pose_upright_yaw_search"
            plans["preplace"] = candidate_plan
            break
        if (
            preplace_goal is None
            or selected_object_orientation is None
            or preplace_mode is None
        ):
            plans["preplace_failures"] = list(preplace_failures)
            raise RuntimeError(
                f"{asset_id}: no collision-free preplace plan kept the "
                f"doll upright across {len(upright_orientation_candidates)} "
                f"yaw candidates; failures={preplace_failures}"
            )
        plans["preplace_upright_yaw_search"] = {
            "candidate_count": len(upright_orientation_candidates),
            "failed_candidate_count": len(preplace_failures),
            "selected_object_orientation": list(
                selected_object_orientation
            ),
            "failures": list(preplace_failures),
            "mode": preplace_mode,
        }
        _set_recording_context(
            episode_recorder,
            "preplace",
            operator=active_spec.name,
            object_id=asset_id,
        )
        executions["preplace"] = execute_curobo_trajectory(
            world,
            active_robot,
            plans["preplace"],
            render=render,
            episode_recorder=episode_recorder,
        )

        preplace_tool_pose = _piper_finger_center_world_pose(
            active_spec,
            f"{asset_id}_preplace",
        )
        preplace_tool_position = preplace_tool_pose.position
        preplace_tool_quaternion = preplace_tool_pose.quaternion
        preplace_doll_pose = _current_doll_poses(dolls)[asset_id]
        preplace_center_error = float(
            np.linalg.norm(
                np.asarray(preplace_doll_pose.position, dtype=np.float64)
                - np.asarray(preplace_object_center, dtype=np.float64)
            )
        )
        if preplace_center_error > 0.005:
            correction_offset = tuple(
                preplace_doll_pose.position[index]
                - preplace_tool_position[index]
                for index in range(3)
            )
            correction_goal = PoseSpec(
                tuple(
                    preplace_object_center[index]
                    - correction_offset[index]
                    for index in range(3)
                ),
                tuple(preplace_tool_quaternion),
            )
            plans["preplace_correction"] = worker.plan_pose(
                active_robot=active_spec,
                other_robot=other_spec,
                current_joint_position=active_robot.get_joint_positions()[:6],
                other_joint_position=other_robot.get_joint_positions()[:6],
                world_goal=correction_goal,
                doll_poses=_current_doll_poses(dolls),
                excluded_doll_ids=(asset_id,),
            )
            _set_recording_context(
                episode_recorder,
                "preplace_correction",
                operator=active_spec.name,
                object_id=asset_id,
            )
            executions["preplace_correction"] = execute_curobo_trajectory(
                world,
                active_robot,
                plans["preplace_correction"],
                render=render,
                episode_recorder=episode_recorder,
            )
            preplace_tool_pose = _piper_finger_center_world_pose(
                active_spec,
                f"{asset_id}_corrected_preplace",
            )
            preplace_tool_position = preplace_tool_pose.position
            preplace_tool_quaternion = preplace_tool_pose.quaternion
            preplace_doll_pose = _current_doll_poses(dolls)[asset_id]
            preplace_center_error = float(
                np.linalg.norm(
                    np.asarray(
                        preplace_doll_pose.position,
                        dtype=np.float64,
                    )
                    - np.asarray(
                        preplace_object_center,
                        dtype=np.float64,
                    )
                )
            )
        preplace_tilt = _doll_state_report(dolls)[asset_id][
            "upright_tilt_degrees"
        ]
        preplace_tilt_before_upright = preplace_tilt
        preplace_upright_center_drift = 0.0
        if (
            preplace_tilt
            > PIPER_TRANSPORT_UPRIGHT_TOLERANCE_DEGREES
        ):
            raise RuntimeError(
                f"{asset_id}: transported doll tilted too far to correct "
                f"safely: {preplace_tilt:.6f} degrees"
            )
        if (
            preplace_tilt
            > PIPER_POST_GRASP_UPRIGHT_TOLERANCE_DEGREES
        ):
            preplace_upright_goal = (
                tool_pose_for_attached_object_orientation(
                    preplace_tool_pose,
                    preplace_doll_pose,
                    upright_yaw_quaternion(
                        preplace_doll_pose.quaternion
                    ),
                )
            )
            plans["preplace_upright"] = worker.plan_pose(
                active_robot=active_spec,
                other_robot=other_spec,
                current_joint_position=active_robot.get_joint_positions()[:6],
                other_joint_position=other_robot.get_joint_positions()[:6],
                world_goal=preplace_upright_goal,
                doll_poses=_current_doll_poses(dolls),
                excluded_doll_ids=(asset_id,),
            )
            _set_recording_context(
                episode_recorder,
                "preplace_upright",
                operator=active_spec.name,
                object_id=asset_id,
            )
            executions["preplace_upright"] = execute_curobo_trajectory(
                world,
                active_robot,
                plans["preplace_upright"],
                render=render,
                episode_recorder=episode_recorder,
            )
            corrected_preplace_doll_pose = _current_doll_poses(dolls)[
                asset_id
            ]
            preplace_upright_center_drift = float(
                np.linalg.norm(
                    np.asarray(
                        corrected_preplace_doll_pose.position,
                        dtype=np.float64,
                    )
                    - np.asarray(
                        preplace_doll_pose.position,
                        dtype=np.float64,
                    )
                )
            )
            preplace_doll_pose = corrected_preplace_doll_pose
            preplace_tool_pose = _piper_finger_center_world_pose(
                active_spec,
                f"{asset_id}_preplace_upright",
            )
            preplace_tool_position = preplace_tool_pose.position
            preplace_tool_quaternion = preplace_tool_pose.quaternion
            preplace_center_error = float(
                np.linalg.norm(
                    np.asarray(
                        preplace_doll_pose.position,
                        dtype=np.float64,
                    )
                    - np.asarray(
                        preplace_object_center,
                        dtype=np.float64,
                    )
                )
            )
            preplace_tilt = _doll_state_report(dolls)[asset_id][
                "upright_tilt_degrees"
            ]
            print(
                "CUROBO_PREPLACE_UPRIGHT_CORRECTION "
                f"asset={asset_id} robot={active_spec.name} "
                f"before_tilt_deg="
                f"{preplace_tilt_before_upright:.6f} "
                f"after_tilt_deg={preplace_tilt:.6f} "
                f"center_drift_m="
                f"{preplace_upright_center_drift:.6f}",
                flush=True,
            )
            if (
                preplace_upright_center_drift
                > PIPER_GRASP_CENTER_TOLERANCE_M
            ):
                raise RuntimeError(
                    f"{asset_id}: preplace joint rotation displaced the "
                    f"doll centre by "
                    f"{preplace_upright_center_drift:.6f} m"
                )
        if (
            preplace_tilt
            > PIPER_POST_GRASP_UPRIGHT_TOLERANCE_DEGREES
        ):
            raise RuntimeError(
                f"{asset_id}: preplace joint rotation left doll tilt at "
                f"{preplace_tilt:.6f} degrees"
            )
        print(
            "CUROBO_PREPLACE_STATE "
            f"asset={asset_id} robot={active_spec.name} "
            f"mode={preplace_mode} "
            f"center_error_m={preplace_center_error:.6f} "
            f"tilt_deg={preplace_tilt:.6f}",
            flush=True,
        )
        place_segment_modes: list[str] = []
        place_segment_states: list[dict[str, Any]] = []
        for segment_index, support_clearance in enumerate(
            PIPER_PLACE_APPROACH_CLEARANCES_M,
            start=1,
        ):
            plan_key = (
                "place"
                if segment_index == len(PIPER_PLACE_APPROACH_CLEARANCES_M)
                else f"place_approach_{segment_index}"
            )
            current_place_doll_pose = _current_doll_poses(dolls)[asset_id]
            current_place_tool_pose = _piper_finger_center_world_pose(
                active_spec,
                f"{asset_id}_{plan_key}_start",
            )
            attached_offset = tuple(
                current_place_doll_pose.position[index]
                - current_place_tool_pose.position[index]
                for index in range(3)
            )
            transport_orientation = tuple(
                current_place_tool_pose.quaternion
            )
            segment_object_center = (
                target_center[0],
                target_center[1],
                target_center[2] + support_clearance,
            )
            place_goal = PoseSpec(
                tuple(
                    segment_object_center[index] - attached_offset[index]
                    for index in range(3)
                ),
                transport_orientation,
            )
            segment_mode = "exact_pose"
            try:
                plans[plan_key] = worker.plan_pose(
                    active_robot=active_spec,
                    other_robot=other_spec,
                    current_joint_position=(
                        active_robot.get_joint_positions()[:6]
                    ),
                    other_joint_position=other_robot.get_joint_positions()[:6],
                    world_goal=place_goal,
                    doll_poses=_current_doll_poses(dolls),
                    excluded_doll_ids=(asset_id,),
                )
            except RuntimeError as exact_error:
                segment_mode = "position_with_upright_object"
                plans[f"{plan_key}_exact_failure"] = {
                    "error": str(exact_error),
                }
                tool_to_attached_object = pose_relative_to(
                    current_place_tool_pose,
                    current_place_doll_pose,
                )
                plans[plan_key] = worker.plan_position(
                    active_robot=active_spec,
                    other_robot=other_spec,
                    current_joint_position=(
                        active_robot.get_joint_positions()[:6]
                    ),
                    other_joint_position=other_robot.get_joint_positions()[:6],
                    world_goal_position=place_goal.position,
                    doll_poses=_current_doll_poses(dolls),
                    excluded_doll_ids=(asset_id,),
                    preferred_world_orientation=transport_orientation,
                    tool_to_attached_object_orientation=(
                        tool_to_attached_object.quaternion
                    ),
                    max_attached_object_tilt_degrees=(
                        PIPER_PLANNED_UPRIGHT_TOLERANCE_DEGREES
                    ),
                )
                orientation_error = plans[plan_key][
                    "preferred_orientation_error_rad"
                ]
                if orientation_error is None or math.degrees(
                    orientation_error
                ) > PIPER_POST_GRASP_UPRIGHT_TOLERANCE_DEGREES:
                    raise RuntimeError(
                        f"{asset_id}: {plan_key} changed tool orientation by "
                        f"{orientation_error} rad"
                    )
            _set_recording_context(
                episode_recorder,
                "place",
                operator=active_spec.name,
                object_id=asset_id,
            )
            executions[plan_key] = execute_curobo_trajectory(
                world,
                active_robot,
                plans[plan_key],
                render=render,
                episode_recorder=episode_recorder,
            )
            segment_pose = _current_doll_poses(dolls)[asset_id]
            segment_state = _doll_state_report(dolls)[asset_id]
            segment_center_error = float(
                np.linalg.norm(
                    np.asarray(segment_pose.position, dtype=np.float64)
                    - np.asarray(segment_object_center, dtype=np.float64)
                )
            )
            segment_tilt = segment_state["upright_tilt_degrees"]
            segment_tilt_before_upright = segment_tilt
            segment_upright_center_drift = 0.0
            if (
                segment_tilt
                > PIPER_PLANNED_UPRIGHT_TOLERANCE_DEGREES
            ):
                if (
                    segment_tilt
                    > PIPER_TRANSPORT_UPRIGHT_TOLERANCE_DEGREES
                ):
                    raise RuntimeError(
                        f"{asset_id}: {plan_key} tilted too far to correct "
                        f"safely: {segment_tilt:.6f} degrees"
                    )
                segment_tool_pose = _piper_finger_center_world_pose(
                    active_spec,
                    f"{asset_id}_{plan_key}_before_upright",
                )
                segment_upright_goal = (
                    tool_pose_for_attached_object_orientation(
                        segment_tool_pose,
                        segment_pose,
                        upright_yaw_quaternion(
                            segment_pose.quaternion
                        ),
                    )
                )
                upright_plan_key = f"{plan_key}_upright"
                plans[upright_plan_key] = worker.plan_pose(
                    active_robot=active_spec,
                    other_robot=other_spec,
                    current_joint_position=(
                        active_robot.get_joint_positions()[:6]
                    ),
                    other_joint_position=other_robot.get_joint_positions()[:6],
                    world_goal=segment_upright_goal,
                    doll_poses=_current_doll_poses(dolls),
                    excluded_doll_ids=(asset_id,),
                )
                _set_recording_context(
                    episode_recorder,
                    "place_upright",
                    operator=active_spec.name,
                    object_id=asset_id,
                )
                executions[upright_plan_key] = execute_curobo_trajectory(
                    world,
                    active_robot,
                    plans[upright_plan_key],
                    render=render,
                    episode_recorder=episode_recorder,
                )
                corrected_segment_pose = _current_doll_poses(dolls)[asset_id]
                segment_upright_center_drift = float(
                    np.linalg.norm(
                        np.asarray(
                            corrected_segment_pose.position,
                            dtype=np.float64,
                        )
                        - np.asarray(
                            segment_pose.position,
                            dtype=np.float64,
                        )
                    )
                )
                segment_pose = corrected_segment_pose
                segment_state = _doll_state_report(dolls)[asset_id]
                segment_tilt = segment_state["upright_tilt_degrees"]
                segment_center_error = float(
                    np.linalg.norm(
                        np.asarray(
                            segment_pose.position,
                            dtype=np.float64,
                        )
                        - np.asarray(
                            segment_object_center,
                            dtype=np.float64,
                        )
                    )
                )
                print(
                    "CUROBO_PLACE_UPRIGHT_CORRECTION "
                    f"asset={asset_id} robot={active_spec.name} "
                    f"segment={segment_index} "
                    f"before_tilt_deg="
                    f"{segment_tilt_before_upright:.6f} "
                    f"after_tilt_deg={segment_tilt:.6f} "
                    f"center_drift_m="
                    f"{segment_upright_center_drift:.6f}",
                    flush=True,
                )
                if (
                    segment_upright_center_drift
                    > PIPER_GRASP_CENTER_TOLERANCE_M
                ):
                    raise RuntimeError(
                        f"{asset_id}: {upright_plan_key} displaced the doll "
                        f"centre by {segment_upright_center_drift:.6f} m"
                    )
            print(
                "CUROBO_PLACE_APPROACH "
                f"asset={asset_id} robot={active_spec.name} "
                f"segment={segment_index} "
                f"support_clearance_m={support_clearance:.6f} "
                f"mode={segment_mode} "
                f"center_error_m={segment_center_error:.6f} "
                f"tilt_deg={segment_tilt:.6f}",
                flush=True,
            )
            if (
                segment_tilt
                > PIPER_POST_GRASP_UPRIGHT_TOLERANCE_DEGREES
            ):
                raise RuntimeError(
                    f"{asset_id}: {plan_key} left doll tilt at "
                    f"{segment_tilt:.6f} degrees"
                )
            if segment_center_error > PIPER_GRASP_CENTER_TOLERANCE_M:
                raise RuntimeError(
                    f"{asset_id}: {plan_key} missed its staged object centre "
                    f"by {segment_center_error:.6f} m"
                )
            place_segment_modes.append(segment_mode)
            place_segment_states.append(
                {
                    "support_clearance_m": support_clearance,
                    "mode": segment_mode,
                    "center_error_m": segment_center_error,
                    "tilt_before_upright_correction_degrees": (
                        segment_tilt_before_upright
                    ),
                    "upright_tilt_degrees": segment_tilt,
                    "upright_correction_center_drift_m": (
                        segment_upright_center_drift
                    ),
                }
            )
        place_mode = place_segment_modes[-1]
        constrained_place_pose = _current_doll_poses(dolls)[asset_id]
        constrained_place_error = float(
            np.linalg.norm(
                np.asarray(constrained_place_pose.position, dtype=np.float64)
                - np.asarray(target_center, dtype=np.float64)
            )
        )
        if constrained_place_error > PIPER_CONSTRAINED_PLACE_TOLERANCE_M:
            constrained_tool_pose = _piper_finger_center_world_pose(
                active_spec,
                f"{asset_id}_constrained_place",
            )
            constrained_offset = tuple(
                constrained_place_pose.position[index]
                - constrained_tool_pose.position[index]
                for index in range(3)
            )
            constrained_correction_goal = PoseSpec(
                tuple(
                    planned_place_center[index] - constrained_offset[index]
                    for index in range(3)
                ),
                constrained_tool_pose.quaternion,
            )
            plans["place_correction"] = worker.plan_pose(
                active_robot=active_spec,
                other_robot=other_spec,
                current_joint_position=active_robot.get_joint_positions()[:6],
                other_joint_position=other_robot.get_joint_positions()[:6],
                world_goal=constrained_correction_goal,
                doll_poses=_current_doll_poses(dolls),
                excluded_doll_ids=(asset_id,),
            )
            _set_recording_context(
                episode_recorder,
                "place",
                operator=active_spec.name,
                object_id=asset_id,
            )
            executions["place_correction"] = execute_curobo_trajectory(
                world,
                active_robot,
                plans["place_correction"],
                render=render,
                episode_recorder=episode_recorder,
            )
            constrained_place_pose = _current_doll_poses(dolls)[asset_id]
            constrained_place_error = float(
                np.linalg.norm(
                    np.asarray(
                        constrained_place_pose.position,
                        dtype=np.float64,
                    )
                    - np.asarray(target_center, dtype=np.float64)
                )
            )
        constrained_place_state = _doll_state_report(dolls)[asset_id]
        constrained_place_tilt = constrained_place_state[
            "upright_tilt_degrees"
        ]
        if (
            constrained_place_tilt
            > PIPER_POST_GRASP_UPRIGHT_TOLERANCE_DEGREES
        ):
            raise RuntimeError(
                f"{asset_id}: constrained place tilt is "
                f"{constrained_place_tilt:.6f} degrees"
            )
        if constrained_place_error > PIPER_CONSTRAINED_PLACE_TOLERANCE_M:
            raise RuntimeError(
                f"{asset_id}: constrained placement error is "
                f"{constrained_place_error:.6f} m"
            )
        constrained_bottom_error = abs(
            constrained_place_state["estimated_bottom_z_m"] - TABLE_TOP_Z
        )
        if constrained_bottom_error > PIPER_CONSTRAINED_PLACE_TOLERANCE_M:
            raise RuntimeError(
                f"{asset_id}: constrained placement bottom is "
                f"{constrained_bottom_error:.6f} m from the tabletop"
            )
        print(
            "CUROBO_RELEASE_STATE "
            f"asset={asset_id} stage=constrained "
            f"position={constrained_place_pose.position} "
            f"error_m={constrained_place_error:.6f} "
            f"tilt_deg={constrained_place_tilt:.6f} "
            f"bottom_error_m={constrained_bottom_error:.6f}",
            flush=True,
        )

        # Fully clear both fingers while the fixed grasp holds the doll upright
        # at the small planning standoff.  After detachment, full PhysX
        # geometry settles onto the tabletop and remains in the collision
        # world for retreat planning.
        release_gripper_position = PIPER_OPEN_GRIPPER_POSITION
        _set_recording_context(
            episode_recorder,
            "open_for_release",
            operator=active_spec.name,
            object_id=asset_id,
        )
        if episode_recorder is not None:
            episode_recorder.set_gripper_action(
                active_robot,
                release_gripper_position,
            )
        _command_position(
            active_robot,
            (release_gripper_position,),
            (6,),
        )
        table_release_steps = _step_control_until(
            world,
            lambda: float(
                np.max(
                    np.abs(
                        active_robot.get_joint_positions()[6:8]
                        - release_gripper_position
                    )
                )
            )
            <= PIPER_GRIPPER_TOLERANCE_M,
            maximum_steps=240,
            description=(
                f"{active_spec.name} Piper gripper to fully clear the "
                "placed doll"
            ),
            render=render,
            episode_recorder=episode_recorder,
        )
        worker.detach(active_robot=active_spec)
        _set_recording_context(
            episode_recorder,
            "detach",
            operator=active_spec.name,
            object_id=asset_id,
        )
        remove_simulation_grasp_joint(
            world,
            active_spec,
            asset_id,
            step_after_change=episode_recorder is None,
        )
        grasp_joint["removed"] = True
        if episode_recorder is not None:
            episode_recorder.queue_grasp_event(
                GRASP_EVENT_DETACH,
                active_spec.name,
                asset_id,
            )
            _advance_control_frame(
                world,
                render=render,
                episode_recorder=episode_recorder,
            )
            remaining_release_steps = (
                PICK_RELEASE_SETTLE_STEPS - physics_steps_per_control
            )
            _set_recording_context(
                episode_recorder,
                "release_settle",
                operator=active_spec.name,
                object_id=asset_id,
            )
            for _ in range(
                remaining_release_steps // physics_steps_per_control
            ):
                _advance_control_frame(
                    world,
                    render=render,
                    episode_recorder=episode_recorder,
                )
        else:
            for step in range(PICK_RELEASE_SETTLE_STEPS):
                world.step(
                    render=(
                        render and step == PICK_RELEASE_SETTLE_STEPS - 1
                    )
                )
        detached_state = _doll_state_report(dolls)[asset_id]
        print(
            "CUROBO_RELEASE_STATE "
            f"asset={asset_id} stage=detached_settle "
            f"position={tuple(detached_state['position_m'])} "
            f"tilt_deg={detached_state['upright_tilt_degrees']:.6f}",
            flush=True,
        )

        released_tool_pose = _piper_finger_center_world_pose(
            active_spec,
            f"{asset_id}_pick_released",
        )
        post_axis_retreat_state = detached_state
        post_axis_retreat_error = float(
            np.linalg.norm(
                np.asarray(detached_state["position_m"], dtype=np.float64)
                - np.asarray(target_center, dtype=np.float64)
            )
        )
        for axis_step, clearance_m in enumerate(
            PIPER_RELEASE_AXIS_CLEARANCES_M,
            start=1,
        ):
            plan_key = f"release_axis_retreat_{axis_step}"
            plans[plan_key] = worker.plan_pose(
                active_robot=active_spec,
                other_robot=other_spec,
                current_joint_position=active_robot.get_joint_positions()[:6],
                other_joint_position=other_robot.get_joint_positions()[:6],
                world_goal=piper_axis_approach_pose(
                    released_tool_pose,
                    clearance_m,
                ),
                doll_poses=_current_doll_poses(dolls),
            )
            _set_recording_context(
                episode_recorder,
                "retreat",
                operator=active_spec.name,
                object_id=asset_id,
            )
            executions[plan_key] = execute_curobo_trajectory(
                world,
                active_robot,
                plans[plan_key],
                render=render,
                episode_recorder=episode_recorder,
            )
            post_axis_retreat_state = _doll_state_report(dolls)[asset_id]
            post_axis_retreat_error = float(
                np.linalg.norm(
                    np.asarray(
                        post_axis_retreat_state["position_m"],
                        dtype=np.float64,
                    )
                    - np.asarray(target_center, dtype=np.float64)
                )
            )
            print(
                "CUROBO_RELEASE_STATE "
                f"asset={asset_id} "
                f"stage=axis_retreat_{axis_step} "
                f"clearance_m={clearance_m:.3f} "
                f"error_m={post_axis_retreat_error:.6f} "
                f"tilt_deg="
                f"{post_axis_retreat_state['upright_tilt_degrees']:.6f}",
                flush=True,
            )
            if (
                post_axis_retreat_state["upright_tilt_degrees"]
                > PIPER_POST_GRASP_UPRIGHT_TOLERANCE_DEGREES
                or post_axis_retreat_error
                > PIPER_CONSTRAINED_PLACE_TOLERANCE_M
            ):
                raise RuntimeError(
                    f"{asset_id}: placed doll moved during axis retreat "
                    f"step {axis_step}; error="
                    f"{post_axis_retreat_error:.6f} m, tilt="
                    f"{post_axis_retreat_state['upright_tilt_degrees']:.6f} "
                    "degrees"
                )

        post_axis_retreat_clearance = piper_post_axis_retreat_clearance(
            doll_spec
        )
        if post_axis_retreat_clearance > 0.0:
            released_tool_pose = _piper_finger_center_world_pose(
                active_spec,
                f"{asset_id}_axis_retreated",
            )
            released_tool_position = released_tool_pose.position
            released_tool_quaternion = released_tool_pose.quaternion
            retreat_goal = PoseSpec(
                (
                    released_tool_position[0],
                    released_tool_position[1],
                    released_tool_position[2]
                    + post_axis_retreat_clearance,
                ),
                tuple(released_tool_quaternion),
            )
            plans["retreat"] = worker.plan_pose(
                active_robot=active_spec,
                other_robot=other_spec,
                current_joint_position=active_robot.get_joint_positions()[:6],
                other_joint_position=other_robot.get_joint_positions()[:6],
                world_goal=retreat_goal,
                doll_poses=_current_doll_poses(dolls),
            )
            _set_recording_context(
                episode_recorder,
                "retreat",
                operator=active_spec.name,
                object_id=asset_id,
            )
            executions["retreat"] = execute_curobo_trajectory(
                world,
                active_robot,
                plans["retreat"],
                render=render,
                episode_recorder=episode_recorder,
            )
        else:
            plans["retreat"] = {
                "skipped": True,
                "reason": "large_doll_axis_retreat_already_clears_fingers",
                "success": True,
            }
        post_retreat_state = _doll_state_report(dolls)[asset_id]
        post_retreat_error = float(
            np.linalg.norm(
                np.asarray(
                    post_retreat_state["position_m"],
                    dtype=np.float64,
                )
                - np.asarray(target_center, dtype=np.float64)
            )
        )
        print(
            "CUROBO_RELEASE_STATE "
            f"asset={asset_id} stage=post_retreat "
            f"extra_clearance_m={post_axis_retreat_clearance:.3f} "
            f"position={tuple(post_retreat_state['position_m'])} "
            f"error_m={post_retreat_error:.6f} "
            f"tilt_deg="
            f"{post_retreat_state['upright_tilt_degrees']:.6f}",
            flush=True,
        )
        if (
            post_retreat_state["upright_tilt_degrees"]
            > PIPER_POST_GRASP_UPRIGHT_TOLERANCE_DEGREES
            or post_retreat_error > PIPER_CONSTRAINED_PLACE_TOLERANCE_M
        ):
            raise RuntimeError(
                f"{asset_id}: placed doll moved during collision-aware "
                f"retreat; error={post_retreat_error:.6f} m, "
                "tilt="
                f"{post_retreat_state['upright_tilt_degrees']:.6f} degrees"
            )
        _set_recording_context(
            episode_recorder,
            "open_gripper",
            operator=active_spec.name,
            object_id=asset_id,
        )
        if episode_recorder is not None:
            episode_recorder.set_gripper_action(
                active_robot,
                PIPER_OPEN_GRIPPER_POSITION,
            )
        _command_position(
            active_robot,
            (PIPER_OPEN_GRIPPER_POSITION,),
            (6,),
        )
        full_open_steps = _step_control_until(
            world,
            lambda: float(
                np.max(
                    np.abs(
                        active_robot.get_joint_positions()[6:8]
                        - PIPER_OPEN_GRIPPER_POSITION
                    )
                )
            )
            <= PIPER_GRIPPER_TOLERANCE_M,
            maximum_steps=240,
            description=(
                f"{active_spec.name} Piper gripper to open after retreat"
            ),
            render=render,
            episode_recorder=episode_recorder,
        )
        plans["home"] = worker.plan_joint(
            active_robot=active_spec,
            other_robot=other_spec,
            current_joint_position=active_robot.get_joint_positions()[:6],
            other_joint_position=other_robot.get_joint_positions()[:6],
            goal_joint_position=PIPER_HOME_JOINT_POSITION,
            doll_poses=_current_doll_poses(dolls),
        )
        _set_recording_context(
            episode_recorder,
            "home",
            operator=active_spec.name,
            object_id=asset_id,
        )
        executions["home"] = execute_curobo_trajectory(
            world,
            active_robot,
            plans["home"],
            render=render,
            episode_recorder=episode_recorder,
        )
        post_home_state = _doll_state_report(dolls)[asset_id]
        post_home_error = float(
            np.linalg.norm(
                np.asarray(post_home_state["position_m"], dtype=np.float64)
                - np.asarray(target_center, dtype=np.float64)
            )
        )
        print(
            "CUROBO_RELEASE_STATE "
            f"asset={asset_id} stage=post_home "
            f"position={tuple(post_home_state['position_m'])} "
            f"error_m={post_home_error:.6f} "
            f"tilt_deg={post_home_state['upright_tilt_degrees']:.6f}",
            flush=True,
        )
        if (
            post_home_state["upright_tilt_degrees"]
            > PIPER_POST_GRASP_UPRIGHT_TOLERANCE_DEGREES
            or post_home_error > PIPER_CONSTRAINED_PLACE_TOLERANCE_M
        ):
            raise RuntimeError(
                f"{asset_id}: placed doll moved during home motion; "
                f"error={post_home_error:.6f} m, "
                f"tilt={post_home_state['upright_tilt_degrees']:.6f} "
                "degrees"
            )
        worker_diagnostics = list(worker.diagnostic_lines)

    _set_recording_context(
        episode_recorder,
        "object_settle",
        operator=active_spec.name,
        object_id=asset_id,
    )
    settled_report = settle_and_validate_dolls(
        world,
        dolls,
        render=render,
        episode_recorder=episode_recorder,
    )
    final_doll_state = settled_report["stable_states"][asset_id]
    final_position_error = float(
        np.linalg.norm(
            np.asarray(final_doll_state["position_m"], dtype=np.float64)
            - np.asarray(target_center, dtype=np.float64)
        )
    )
    if final_position_error > MATRYOSHKA_POSITION_TOLERANCE:
        raise RuntimeError(
            f"{asset_id}: released placement error {final_position_error:.6f} m"
        )
    final_robots = validate_robots_at_home(
        robots,
        label="pick_place_final",
    )
    return {
        "asset_id": asset_id,
        "active_robot": active_spec.name,
        "planner_seed": planner_seed,
        "initial_doll_pose": asdict(initial_doll_pose),
        "target_center_m": list(target_center),
        "planned_place_center_m": list(planned_place_center),
        "grasp": {
            "finger_center_m": list(finger_center),
            "contact_point_m": list(closed_contact_point),
            "contact_height_above_center_m": grasp_contact_height,
            "terminal_axis_clearance_m": terminal_grasp_clearance,
            "initial_approach_center_error_m": (
                initial_approach_center_error
            ),
            "approach_center_error_m": approach_center_error,
            "center_error_m": grasp_center_error,
            "lateral_center_error_m": lateral_center_error,
            "axial_center_error_m": axial_center_error,
            "closed_doll_displacement_m": closed_doll_displacement,
            "closed_doll_tilt_degrees": closed_tilt,
            "closed_finger_separation_m": closed_separation,
            "attachment_gate": attachment_gate,
            "fixed_joint": grasp_joint,
            "curobo_attachment": attachment,
        },
        "transport": {
            "lifted_distance_m": lifted_distance,
            "lift_mode": lift_mode,
            "lifted_tilt_before_upright_correction_degrees": (
                lifted_tilt_before_upright
            ),
            "lifted_upright_tilt_degrees": lifted_tilt,
            "upright_correction_center_drift_m": upright_center_drift,
            "upright_correction_deferred_to_preplace": (
                upright_correction_deferred_to_preplace
            ),
            "upright_correction_yaw_offset_rad": (
                selected_upright_yaw_offset
            ),
            "attached_tool_to_object_world_offset_m": list(attached_offset),
            "preplace_mode": preplace_mode,
            "preplace_center_error_m": preplace_center_error,
            "preplace_tilt_before_upright_correction_degrees": (
                preplace_tilt_before_upright
            ),
            "preplace_upright_tilt_degrees": preplace_tilt,
            "preplace_upright_correction_center_drift_m": (
                preplace_upright_center_drift
            ),
        },
        "release": {
            "planned_support_clearance_m": (
                PIPER_PLANNED_PLACE_SUPPORT_CLEARANCE_M
            ),
            "table_release_gripper_position_m": release_gripper_position,
            "table_release_steps": table_release_steps,
            "full_open_after_retreat_steps": full_open_steps,
            "place_mode": place_mode,
            "place_segment_modes": place_segment_modes,
            "place_segment_states": place_segment_states,
            "post_axis_extra_retreat_clearance_m": (
                post_axis_retreat_clearance
            ),
            "constrained_place_error_m": constrained_place_error,
            "constrained_place_upright_tilt_degrees": (
                constrained_place_tilt
            ),
            "constrained_place_bottom_error_m": constrained_bottom_error,
            "post_axis_retreat_error_m": post_axis_retreat_error,
            "post_axis_retreat_upright_tilt_degrees": (
                post_axis_retreat_state["upright_tilt_degrees"]
            ),
            "post_retreat_error_m": post_retreat_error,
            "post_retreat_upright_tilt_degrees": (
                post_retreat_state["upright_tilt_degrees"]
            ),
            "post_home_error_m": post_home_error,
            "post_home_upright_tilt_degrees": (
                post_home_state["upright_tilt_degrees"]
            ),
            "final_place_error_m": final_position_error,
            "final_doll_state": final_doll_state,
        },
        "plans": plans,
        "executions": executions,
        "initial_robots": initial_robots,
        "final_robots": final_robots,
        "settled_dolls": settled_report,
        "worker_diagnostic_tail": worker_diagnostics[-20:],
    }


def assign_dolls_to_robots(
    doll_poses: dict[str, PoseSpec],
) -> dict[str, str]:
    """Assign by table side while guaranteeing participation by both arms."""

    expected = {spec.asset_id for spec in get_doll_specs()}
    if set(doll_poses) != expected:
        raise ValueError(
            f"Cannot assign changed doll IDs: {sorted(doll_poses)}"
        )
    assignments = {
        asset_id: ("left" if pose.position[0] <= 0.0 else "right")
        for asset_id, pose in doll_poses.items()
    }
    used = set(assignments.values())
    if used == {"left"}:
        asset_id = max(
            doll_poses,
            key=lambda candidate: doll_poses[candidate].position[0],
        )
        assignments[asset_id] = "right"
    elif used == {"right"}:
        asset_id = min(
            doll_poses,
            key=lambda candidate: doll_poses[candidate].position[0],
        )
        assignments[asset_id] = "left"
    return assignments


def validate_sorted_doll_states(
    states: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    """Require the five released dolls at their size-aware central targets."""

    _validate_stable_doll_states(states)
    targets = {
        placement.asset_id: placement
        for placement in compute_doll_target_layout()
    }
    errors: dict[str, float] = {}
    for asset_id, target in targets.items():
        error = math.sqrt(
            sum(
                (
                    float(states[asset_id]["position_m"][index])
                    - target.pose.position[index]
                )
                ** 2
                for index in range(3)
            )
        )
        errors[asset_id] = error
        if error > MATRYOSHKA_POSITION_TOLERANCE:
            raise ValueError(
                f"{asset_id}: final target error {error:.6f} m exceeds "
                f"{MATRYOSHKA_POSITION_TOLERANCE:.6f} m"
            )
    final_y = [
        float(states[asset_id]["position_m"][1])
        for asset_id in MATRYOSHKA_SORT_ORDER
    ]
    if any(
        second <= first for first, second in zip(final_y, final_y[1:])
    ):
        raise ValueError("Final doll centres are not ordered small-to-large")
    return {
        "target_error_m": errors,
        "maximum_target_error_m": max(errors.values()),
        "order_small_to_large": list(MATRYOSHKA_SORT_ORDER),
        "center_line_x_m": [
            float(states[asset_id]["position_m"][0])
            for asset_id in MATRYOSHKA_SORT_ORDER
        ],
        "ordered_y_m": final_y,
    }


def validate_task_success(
    world: Any,
    robots: dict[str, Any],
    dolls: dict[str, Any],
    *,
    label: str,
    render: bool = False,
    episode_recorder: Any | None = None,
) -> dict[str, Any]:
    """Apply the shared original-execution and replay acceptance criteria."""

    final_settled = settle_and_validate_dolls(
        world,
        dolls,
        render=render,
        episode_recorder=episode_recorder,
    )
    return {
        "final_settled_dolls": final_settled,
        "final_validation": validate_sorted_doll_states(
            final_settled["stable_states"]
        ),
        "final_robots": validate_robots_at_home(
            robots,
            label=label,
        ),
        "success": True,
    }


def run_full_curobo_sort(
    world: Any,
    robots: dict[str, Any],
    dolls: dict[str, Any],
    *,
    planner_seed: int,
    render: bool,
    episode_recorder: Any | None = None,
) -> dict[str, Any]:
    """Sequentially sort all five dolls with both Piper arms."""

    initial_poses = _current_doll_poses(dolls)
    assignments = assign_dolls_to_robots(initial_poses)
    if set(assignments.values()) != {"left", "right"}:
        raise RuntimeError("Full sort must assign work to both Piper arms")

    # Clear objects initially adjacent to future target slots before filling
    # those slots.  This leaves physical finger clearance that is not captured
    # fully by the conservative planning spheres.
    pick_order = MATRYOSHKA_PICK_ORDER
    operations: list[dict[str, Any]] = []
    placed_asset_ids: list[str] = []
    for operation_index, asset_id in enumerate(pick_order, start=1):
        print(
            "CUROBO_SORT_PROGRESS "
            f"{operation_index}/{len(pick_order)} "
            f"asset={asset_id} robot={assignments[asset_id]} start",
            flush=True,
        )
        operation = run_curobo_pick_place_smoke(
            world,
            robots,
            dolls,
            planner_seed=planner_seed,
            render=render,
            asset_id=asset_id,
            active_robot_name=assignments[asset_id],
            episode_recorder=episode_recorder,
        )
        operations.append(operation)
        placed_asset_ids.append(asset_id)
        current_poses = _current_doll_poses(dolls)
        target_poses = {
            placement.asset_id: placement.pose
            for placement in compute_doll_target_layout()
        }
        placed_errors = {
            placed_id: math.sqrt(
                sum(
                    (
                        current_poses[placed_id].position[index]
                        - target_poses[placed_id].position[index]
                    )
                    ** 2
                    for index in range(3)
                )
            )
            for placed_id in placed_asset_ids
        }
        print(
            "CUROBO_SORT_PROGRESS "
            f"{operation_index}/{len(pick_order)} "
            f"asset={asset_id} robot={assignments[asset_id]} complete "
            "final_error_m="
            f"{operation['release']['final_place_error_m']:.6f} "
            "tilt_deg="
            f"{operation['release']['final_doll_state']['upright_tilt_degrees']:.6f} "
            "placed_errors_m="
            f"{json.dumps(placed_errors, sort_keys=True)}",
            flush=True,
        )

    _set_recording_context(
        episode_recorder,
        "final_settle",
    )
    final_result = validate_task_success(
        world,
        robots,
        dolls,
        label="full_sort_final",
        render=render,
        episode_recorder=episode_recorder,
    )
    return {
        "planner": "cuRobo MotionPlanner",
        "planner_seed_per_operation": planner_seed,
        "pick_order": list(pick_order),
        "assignments": assignments,
        "participating_robots": sorted(set(assignments.values())),
        "operations": operations,
        **final_result,
    }


def _finger_separation(spec: RobotSpec, label: str) -> float:
    import numpy as np

    left_position, _ = _xform_world_pose(
        f"{spec.prim_path}/link7", f"{spec.name}_{label}_link7"
    )
    right_position, _ = _xform_world_pose(
        f"{spec.prim_path}/link8", f"{spec.name}_{label}_link8"
    )
    return float(
        np.linalg.norm(
            np.asarray(left_position, dtype=np.float64)
            - np.asarray(right_position, dtype=np.float64)
        )
    )


def quaternion_angular_error(
    first: Sequence[float], second: Sequence[float]
) -> float:
    """Smallest rotation angle between two wxyz quaternions, in radians."""

    first_normalized = normalize_quaternion(first)
    second_normalized = normalize_quaternion(second)
    dot = abs(
        sum(
            first_normalized[index] * second_normalized[index]
            for index in range(4)
        )
    )
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def validate_robots_at_home(
    robots: dict[str, Any],
    *,
    label: str,
) -> dict[str, Any]:
    """Validate joint order, base pose, tool direction, open gripper, and home."""

    import numpy as np

    reports: dict[str, Any] = {}
    home = np.asarray(PIPER_HOME_JOINT_POSITION, dtype=np.float64)
    for spec in ROBOT_SPECS:
        robot = robots[spec.name]
        if tuple(robot.dof_names) != PIPER_DOF_NAMES:
            raise ValueError(f"{spec.name}: unexpected DOF order {robot.dof_names}")
        joint_positions = np.asarray(robot.get_joint_positions(), dtype=np.float64)
        maximum_home_error = float(np.max(np.abs(joint_positions[:6] - home)))
        if maximum_home_error > PIPER_HOME_TOLERANCE_RAD:
            raise ValueError(
                f"{spec.name}: home error {maximum_home_error:.6f} rad exceeds "
                f"{PIPER_HOME_TOLERANCE_RAD:.6f} rad"
            )
        gripper_error = float(
            np.max(
                np.abs(
                    joint_positions[6:8]
                    - np.asarray(
                        (
                            PIPER_OPEN_GRIPPER_POSITION,
                            PIPER_OPEN_GRIPPER_POSITION,
                        )
                    )
                )
            )
        )
        if gripper_error > PIPER_GRIPPER_TOLERANCE_M:
            raise ValueError(
                f"{spec.name}: open gripper error {gripper_error:.6f} m"
            )

        base_position, base_quaternion = robot.get_world_pose()
        _assert_vector_close(
            base_position,
            spec.base_pose.position,
            label=f"{spec.name} Piper base position",
            tolerance=2.0e-5,
        )
        base_orientation_error = quaternion_angular_error(
            base_quaternion, spec.base_pose.quaternion
        )
        if base_orientation_error > 2.0e-5:
            raise ValueError(
                f"{spec.name}: base orientation error "
                f"{base_orientation_error:.8f} rad"
            )

        tool_position, tool_quaternion = _xform_world_pose(
            f"{spec.prim_path}/{PIPER_TOOL_REL_PATH}",
            f"{spec.name}_{label}_tool",
        )
        forward_distance = tool_position[1] - spec.base_pose.position[1]
        if not (
            PIPER_HOME_TOOL_FORWARD_RANGE_M[0]
            <= forward_distance
            <= PIPER_HOME_TOOL_FORWARD_RANGE_M[1]
        ):
            raise ValueError(
                f"{spec.name}: retracted gripper-center offset "
                f"{forward_distance:.4f} m is outside "
                f"{PIPER_HOME_TOOL_FORWARD_RANGE_M}"
            )
        if not (
            TABLE_X_RANGE[0] <= tool_position[0] <= TABLE_X_RANGE[1]
            and TABLE_Y_RANGE[0] <= tool_position[1] <= TABLE_Y_RANGE[1]
            and tool_position[2] > TABLE_TOP_Z
        ):
            raise ValueError(
                f"{spec.name}: home gripper center is outside the table workspace: "
                f"{tool_position}"
            )

        separation = _finger_separation(spec, label)
        if not math.isclose(
            separation,
            PIPER_OPEN_FINGER_SEPARATION_M,
            abs_tol=2.0e-3,
        ):
            raise ValueError(
                f"{spec.name}: expected about 0.08 m open separation, "
                f"found {separation:.6f} m"
            )

        properties = robot.dof_properties
        reports[spec.name] = {
            "prim_path": spec.prim_path,
            "dof_names": list(robot.dof_names),
            "dof_limits": [
                [float(lower), float(upper)]
                for lower, upper in zip(
                    properties["lower"], properties["upper"]
                )
            ],
            "base_pose": {
                "position": base_position.tolist(),
                "quaternion": base_quaternion.tolist(),
                "orientation_error_rad": base_orientation_error,
            },
            "home_joint_position": joint_positions.tolist(),
            "maximum_home_error_rad": maximum_home_error,
            "gripper_open_error_m": gripper_error,
            "finger_separation_m": separation,
            "tool_world_pose": {
                "position": tool_position,
                "quaternion": tool_quaternion,
            },
            "tool_forward_from_base_m": forward_distance,
        }
    return reports


def exercise_and_validate_robots(
    world: Any, robots: dict[str, Any]
) -> dict[str, Any]:
    """Exercise both grippers and arm drives, then return both robots home."""

    import numpy as np

    initial = validate_robots_at_home(robots, label="initial_home")

    for robot in robots.values():
        _command_position(robot, (PIPER_CLOSED_GRIPPER_POSITION,), (6,))
    close_steps = _step_until(
        world,
        lambda: all(
            float(np.max(np.abs(robot.get_joint_positions()[6:8])))
            <= PIPER_GRIPPER_TOLERANCE_M
            for robot in robots.values()
        ),
        maximum_steps=240,
        description="both Piper grippers to close through the mimic joint",
    )
    closed_separations = {
        spec.name: _finger_separation(spec, "closed") for spec in ROBOT_SPECS
    }
    if any(value > 0.003 for value in closed_separations.values()):
        raise ValueError(
            f"Closed gripper finger separation is too large: {closed_separations}"
        )

    for robot in robots.values():
        _command_position(robot, (PIPER_OPEN_GRIPPER_POSITION,), (6,))
    open_steps = _step_until(
        world,
        lambda: all(
            float(
                np.max(
                    np.abs(
                        robot.get_joint_positions()[6:8]
                        - PIPER_OPEN_GRIPPER_POSITION
                    )
                )
            )
            <= PIPER_GRIPPER_TOLERANCE_M
            for robot in robots.values()
        ),
        maximum_steps=240,
        description="both Piper grippers to reopen through the mimic joint",
    )
    reopened_separations = {
        spec.name: _finger_separation(spec, "reopened") for spec in ROBOT_SPECS
    }

    home = np.asarray(PIPER_HOME_JOINT_POSITION, dtype=np.float64)
    for sign, spec in zip((1.0, -1.0), ROBOT_SPECS):
        perturbed = home.copy()
        perturbed[0] = sign * 0.2
        _command_position(robots[spec.name], perturbed, range(6))
    perturb_steps = _step_until(
        world,
        lambda: all(
            abs(float(robots[spec.name].get_joint_positions()[0]) - sign * 0.2)
            <= 0.01
            for sign, spec in zip((1.0, -1.0), ROBOT_SPECS)
        ),
        maximum_steps=240,
        description="opposed joint1 perturbations",
    )
    perturbed_joint1 = {
        spec.name: float(robots[spec.name].get_joint_positions()[0])
        for spec in ROBOT_SPECS
    }

    for robot in robots.values():
        _command_position(
            robot,
            PIPER_HOME_JOINT_POSITION + (PIPER_OPEN_GRIPPER_POSITION,),
            range(7),
        )
    home_steps = _step_until(
        world,
        lambda: all(
            float(
                np.max(
                    np.abs(
                        robot.get_joint_positions()[:6]
                        - np.asarray(PIPER_HOME_JOINT_POSITION)
                    )
                )
            )
            <= PIPER_HOME_TOLERANCE_RAD
            and float(
                np.max(
                    np.abs(
                        robot.get_joint_positions()[6:8]
                        - PIPER_OPEN_GRIPPER_POSITION
                    )
                )
            )
            <= PIPER_GRIPPER_TOLERANCE_M
            for robot in robots.values()
        ),
        maximum_steps=480,
        description="both Piper arms to return home with open grippers",
    )
    for _ in range(PIPER_HOME_SETTLE_STEPS):
        world.step(render=False)
    final = validate_robots_at_home(robots, label="final_home")

    return {
        "joint_mapping": {
            "all_sim_dofs": list(PIPER_DOF_NAMES),
            "curobo_arm_joints": list(PIPER_ARM_JOINT_NAMES),
            "commanded_gripper_joint": "gripper_joint",
            "mimic_joint": "joint8",
        },
        "initial_home": initial,
        "gripper_cycle": {
            "close_steps": close_steps,
            "closed_finger_separation_m": closed_separations,
            "open_steps": open_steps,
            "reopened_finger_separation_m": reopened_separations,
        },
        "home_cycle": {
            "perturb_steps": perturb_steps,
            "perturbed_joint1_rad": perturbed_joint1,
            "return_home_steps": home_steps,
            "post_tolerance_settle_steps": PIPER_HOME_SETTLE_STEPS,
        },
        "final_home": final,
    }


def _doll_prim_path(asset_id: str) -> str:
    return f"{MATRYOSHKA_PRIM_ROOT}/Doll_{asset_id}"


def _create_doll_physics_material(stage: Any) -> Any:
    from pxr import UsdPhysics, UsdShade  # type: ignore[import-not-found]

    material = UsdShade.Material.Define(stage, MATRYOSHKA_PHYSICS_MATERIAL_PATH)
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    friction = get_doll_specs()[0].friction
    physics.CreateStaticFrictionAttr(friction)
    physics.CreateDynamicFrictionAttr(friction)
    physics.CreateRestitutionAttr(MATRYOSHKA_PHYSICS_RESTITUTION)
    return material


def create_dolls(
    world: Any,
    placements: Sequence[DollPlacement],
) -> dict[str, Any]:
    """Reference the five rigid USDZ assets and apply metadata mass/friction."""

    import numpy as np
    from isaacsim.core.prims import (  # type: ignore[import-not-found]
        SingleRigidPrim,
    )
    from pxr import (  # type: ignore[import-not-found]
        PhysxSchema,
        Usd,
        UsdPhysics,
        UsdShade,
    )

    validate_initial_doll_layout(placements)
    stage = world.stage
    if stage.GetPrimAtPath(MATRYOSHKA_PRIM_ROOT):
        raise ValueError(f"Doll root already exists: {MATRYOSHKA_PRIM_ROOT}")
    stage.DefinePrim(MATRYOSHKA_PRIM_ROOT, "Xform")
    material = _create_doll_physics_material(stage)
    specs_by_id = {spec.asset_id: spec for spec in get_doll_specs()}
    dolls: dict[str, Any] = {}

    for placement in placements:
        spec = specs_by_id[placement.asset_id]
        prim_path = _doll_prim_path(spec.asset_id)
        prim = _add_reference(
            stage,
            prim_path=prim_path,
            asset_path=spec.asset_path,
            pose=placement.pose,
        )
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(spec.mass)
        collision_prims = [
            descendant
            for descendant in Usd.PrimRange(prim)
            if descendant.HasAPI(UsdPhysics.CollisionAPI)
        ]
        if not collision_prims:
            raise ValueError(f"{spec.asset_id}: composed asset has no collision prim")
        for collision_prim in collision_prims:
            binding_api = UsdShade.MaterialBindingAPI.Apply(collision_prim)
            binding_api.Bind(
                material,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose="physics",
            )
        dolls[spec.asset_id] = world.scene.add(
            SingleRigidPrim(
                prim_path=prim_path,
                name=f"doll_{spec.asset_id}",
                reset_xform_properties=False,
            )
        )

    world.reset()
    zero = np.zeros(3, dtype=np.float32)
    for asset_id, doll in dolls.items():
        prim = stage.GetPrimAtPath(_doll_prim_path(asset_id))
        physx_body = PhysxSchema.PhysxRigidBodyAPI(prim)
        if not physx_body:
            physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physx_body.CreateLinearDampingAttr().Set(
            MATRYOSHKA_LINEAR_DAMPING
        )
        physx_body.CreateAngularDampingAttr().Set(
            MATRYOSHKA_ANGULAR_DAMPING
        )
        physx_body.CreateSleepThresholdAttr().Set(
            MATRYOSHKA_SLEEP_THRESHOLD
        )
        physx_body.CreateStabilizationThresholdAttr().Set(
            MATRYOSHKA_STABILIZATION_THRESHOLD
        )
        doll.set_mass(specs_by_id[asset_id].mass)
        doll.set_linear_velocity(zero)
        doll.set_angular_velocity(zero)
    return dolls


def _doll_state_report(
    dolls: dict[str, Any],
) -> dict[str, dict[str, Any]]:
    import numpy as np

    specs_by_id = {spec.asset_id: spec for spec in get_doll_specs()}
    report: dict[str, dict[str, Any]] = {}
    for asset_id, doll in dolls.items():
        position, quaternion = doll.get_world_pose()
        linear_velocity = np.asarray(doll.get_linear_velocity(), dtype=np.float64)
        angular_velocity = np.asarray(doll.get_angular_velocity(), dtype=np.float64)
        local_up = _quaternion_rotate_vector(quaternion, (0.0, 0.0, 1.0))
        upright_cosine = min(1.0, max(-1.0, float(local_up[2])))
        report[asset_id] = {
            "position_m": np.asarray(position, dtype=np.float64).tolist(),
            "quaternion_wxyz": np.asarray(quaternion, dtype=np.float64).tolist(),
            "linear_velocity_m_s": linear_velocity.tolist(),
            "angular_velocity_rad_s": angular_velocity.tolist(),
            "linear_speed_m_s": float(np.linalg.norm(linear_velocity)),
            "angular_speed_rad_s": float(np.linalg.norm(angular_velocity)),
            "upright_tilt_degrees": math.degrees(math.acos(upright_cosine)),
            "estimated_bottom_z_m": float(
                position[2] - specs_by_id[asset_id].height / 2.0
            ),
        }
    return report


def _validate_stable_doll_states(
    states: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    specs_by_id = {spec.asset_id: spec for spec in get_doll_specs()}
    expected_ids = set(specs_by_id)
    if set(states) != expected_ids:
        raise ValueError(f"Stable-state IDs changed: {sorted(states)}")

    for asset_id, state in states.items():
        if state["linear_speed_m_s"] > MATRYOSHKA_LINEAR_SPEED_TOLERANCE:
            raise ValueError(f"{asset_id}: linear speed is above the stable limit")
        if state["angular_speed_rad_s"] > MATRYOSHKA_ANGULAR_SPEED_TOLERANCE:
            raise ValueError(f"{asset_id}: angular speed is above the stable limit")
        if state["upright_tilt_degrees"] > MATRYOSHKA_UPRIGHT_TOLERANCE_DEGREES:
            raise ValueError(f"{asset_id}: doll is not upright")
        if (
            abs(state["estimated_bottom_z_m"] - TABLE_TOP_Z)
            > MATRYOSHKA_TABLE_HEIGHT_TOLERANCE
        ):
            raise ValueError(
                f"{asset_id}: bottom is not resting on the table, "
                f"z={state['estimated_bottom_z_m']}"
            )
        x, y = state["position_m"][:2]
        radius = specs_by_id[asset_id].footprint_radius
        if not (
            TABLE_X_RANGE[0] + radius <= x <= TABLE_X_RANGE[1] - radius
            and TABLE_Y_RANGE[0] + radius <= y <= TABLE_Y_RANGE[1] - radius
        ):
            raise ValueError(f"{asset_id}: stable doll is outside the tabletop")

    minimum_pair_gap = math.inf
    state_items = list(states.items())
    for index, (first_id, first) in enumerate(state_items):
        for second_id, second in state_items[index + 1 :]:
            distance = math.hypot(
                first["position_m"][0] - second["position_m"][0],
                first["position_m"][1] - second["position_m"][1],
            )
            gap = (
                distance
                - specs_by_id[first_id].footprint_radius
                - specs_by_id[second_id].footprint_radius
            )
            minimum_pair_gap = min(minimum_pair_gap, gap)
            if gap < -1.0e-3:
                raise ValueError(
                    f"{first_id}/{second_id}: stable collision overlap {gap:.5f} m"
                )
    return {"minimum_pair_surface_gap_m": minimum_pair_gap}


def validate_doll_physics(
    world: Any,
    dolls: dict[str, Any],
) -> dict[str, Any]:
    """Validate composed rigid bodies, convex collisions, mass, and material."""

    from pxr import (  # type: ignore[import-not-found]
        PhysxSchema,
        Usd,
        UsdPhysics,
        UsdShade,
    )

    specs_by_id = {spec.asset_id: spec for spec in get_doll_specs()}
    material_prim = world.stage.GetPrimAtPath(MATRYOSHKA_PHYSICS_MATERIAL_PATH)
    if not material_prim or not material_prim.HasAPI(UsdPhysics.MaterialAPI):
        raise ValueError("Matryoshka physics material was not created")
    material_api = UsdPhysics.MaterialAPI(material_prim)
    friction = float(material_api.GetDynamicFrictionAttr().Get())
    if not math.isclose(friction, get_doll_specs()[0].friction, abs_tol=1.0e-7):
        raise ValueError("Matryoshka dynamic friction differs from metadata")

    report: dict[str, Any] = {}
    for asset_id, doll in dolls.items():
        spec = specs_by_id[asset_id]
        prim_path = _doll_prim_path(asset_id)
        prim = world.stage.GetPrimAtPath(prim_path)
        if not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f"{asset_id}: root is not a rigid body")
        physx_body = PhysxSchema.PhysxRigidBodyAPI(prim)
        if not physx_body:
            raise ValueError(f"{asset_id}: PhysX rigid-body API is missing")
        physx_values = {
            "linear_damping": float(
                physx_body.GetLinearDampingAttr().Get()
            ),
            "angular_damping": float(
                physx_body.GetAngularDampingAttr().Get()
            ),
            "sleep_threshold": float(
                physx_body.GetSleepThresholdAttr().Get()
            ),
            "stabilization_threshold": float(
                physx_body.GetStabilizationThresholdAttr().Get()
            ),
        }
        expected_physx_values = {
            "linear_damping": MATRYOSHKA_LINEAR_DAMPING,
            "angular_damping": MATRYOSHKA_ANGULAR_DAMPING,
            "sleep_threshold": MATRYOSHKA_SLEEP_THRESHOLD,
            "stabilization_threshold": MATRYOSHKA_STABILIZATION_THRESHOLD,
        }
        for property_name, expected in expected_physx_values.items():
            if not math.isclose(
                physx_values[property_name],
                expected,
                abs_tol=1.0e-7,
            ):
                raise ValueError(
                    f"{asset_id}: {property_name} "
                    f"{physx_values[property_name]} differs from {expected}"
                )
        mass = float(doll.get_mass())
        if not math.isclose(mass, spec.mass, abs_tol=1.0e-7):
            raise ValueError(f"{asset_id}: expected mass {spec.mass}, found {mass}")
        collision_prims = [
            descendant
            for descendant in Usd.PrimRange(prim)
            if descendant.HasAPI(UsdPhysics.CollisionAPI)
        ]
        bound_material_paths: list[str] = []
        for collision_prim in collision_prims:
            binding = UsdShade.MaterialBindingAPI(
                collision_prim
            ).GetDirectBinding("physics")
            bound_material_paths.append(str(binding.GetMaterialPath()))
        if not collision_prims or set(bound_material_paths) != {
            MATRYOSHKA_PHYSICS_MATERIAL_PATH
        }:
            raise ValueError(
                f"{asset_id}: physics material binding failed: "
                f"{bound_material_paths}"
            )
        report[asset_id] = {
            "prim_path": prim_path,
            "asset_path": str(spec.asset_path),
            "mass_kg": mass,
            "static_friction": float(
                material_api.GetStaticFrictionAttr().Get()
            ),
            "dynamic_friction": friction,
            "restitution": float(material_api.GetRestitutionAttr().Get()),
            **physx_values,
            "collision_prim_paths": [
                str(collision_prim.GetPath()) for collision_prim in collision_prims
            ],
            "physics_material_path": MATRYOSHKA_PHYSICS_MATERIAL_PATH,
        }
    return report


def settle_and_validate_dolls(
    world: Any,
    dolls: dict[str, Any],
    *,
    render: bool = False,
    episode_recorder: Any | None = None,
) -> dict[str, Any]:
    """Advance finite physics steps until all five dolls are stably upright."""

    initial_states = _doll_state_report(dolls)
    consecutive = 0
    final_states: dict[str, dict[str, Any]] | None = None
    last_validation_error = "no physics steps executed"
    settle_steps = 0
    physics_steps_per_control = (
        PHYSICS_FREQUENCY_HZ // CONTROL_FREQUENCY_HZ
    )
    for settle_steps in range(1, MATRYOSHKA_SETTLE_MAX_STEPS + 1):
        control_boundary = (
            settle_steps % physics_steps_per_control == 0
        )
        world.step(
            render=(
                (render or episode_recorder is not None)
                and control_boundary
            )
        )
        if episode_recorder is not None and control_boundary:
            episode_recorder.capture_frame()
        states = _doll_state_report(dolls)
        try:
            _validate_stable_doll_states(states)
        except ValueError as error:
            consecutive = 0
            last_validation_error = str(error)
        else:
            consecutive += 1
            if (
                consecutive >= MATRYOSHKA_STABLE_CONSECUTIVE_STEPS
                and (
                    episode_recorder is None
                    or control_boundary
                )
            ):
                final_states = states
                break
    if final_states is None:
        raise RuntimeError(
            "Dolls did not satisfy the stable pose/velocity limits after "
            f"{MATRYOSHKA_SETTLE_MAX_STEPS} physics steps; "
            f"last_validation_error={last_validation_error}; "
            f"last_states={states}"
        )
    geometry = _validate_stable_doll_states(final_states)
    return {
        "initial_states": initial_states,
        "stable_states": final_states,
        "settling": {
            "physics_steps": settle_steps,
            "stable_consecutive_steps": consecutive,
            "duration_s": settle_steps * PHYSICS_DT,
            **geometry,
        },
        "physics": validate_doll_physics(world, dolls),
    }


def camera_intrinsics(spec: CameraSpec) -> list[list[float]]:
    """Return the configured pinhole matrix without importing Isaac Sim."""

    width, height = spec.resolution
    focal_pixels = width * spec.focal_length_mm / spec.horizontal_aperture_mm
    return [
        [focal_pixels, 0.0, width / 2.0],
        [0.0, focal_pixels, height / 2.0],
        [0.0, 0.0, 1.0],
    ]


def _camera_spec_by_name() -> dict[str, CameraSpec]:
    return {spec.name: spec for spec in CAMERA_SPECS}


def create_cameras(world: Any) -> dict[str, Any]:
    """Create two wrist D435 approximations and one stand-mounted overhead view."""

    import numpy as np
    from isaacsim.sensors.camera import Camera  # type: ignore[import-not-found]

    for parent_path in (
        f"{LEFT_PIPER_PRIM_PATH}/{PIPER_CAMERA_MOUNT_REL_PATH}",
        f"{RIGHT_PIPER_PRIM_PATH}/{PIPER_CAMERA_MOUNT_REL_PATH}",
        CAMERA_STAND_PRIM_PATH,
    ):
        if not world.stage.GetPrimAtPath(parent_path):
            raise ValueError(f"Camera parent prim does not exist: {parent_path}")

    specs = _camera_spec_by_name()
    cameras: dict[str, Any] = {}
    for name in (LEFT_WRIST_CAMERA_NAME, RIGHT_WRIST_CAMERA_NAME):
        spec = specs[name]
        camera = Camera(
            prim_path=CAMERA_PRIM_PATHS[name],
            name=name,
            frequency=spec.frequency_hz,
            resolution=spec.resolution,
            translation=np.zeros(3, dtype=np.float32),
            orientation=np.asarray(
                WRIST_CAMERA_LOCAL_USD_ORIENTATION, dtype=np.float32
            ),
            annotator_device="cpu",
        )
        # Explicitly preserve the authored helper as the parent frame; "usd"
        # avoids an additional camera-axis conversion on this local rotation.
        camera.set_local_pose(
            translation=np.zeros(3, dtype=np.float32),
            orientation=np.asarray(
                WRIST_CAMERA_LOCAL_USD_ORIENTATION, dtype=np.float32
            ),
            camera_axes="usd",
        )
        cameras[name] = camera

    overhead_spec = specs[OVERHEAD_CAMERA_NAME]
    overhead = Camera(
        prim_path=OVERHEAD_CAMERA_PRIM_PATH,
        name=OVERHEAD_CAMERA_NAME,
        frequency=overhead_spec.frequency_hz,
        resolution=overhead_spec.resolution,
        annotator_device="cpu",
    )
    overhead.set_world_pose(
        position=np.asarray(OVERHEAD_CAMERA_POSITION, dtype=np.float32),
        orientation=np.asarray(
            OVERHEAD_CAMERA_USD_ORIENTATION,
            dtype=np.float32,
        ),
        camera_axes="usd",
    )
    cameras[OVERHEAD_CAMERA_NAME] = overhead

    for spec in CAMERA_SPECS:
        camera = cameras[spec.name]
        camera.set_focal_length(spec.focal_length_mm)
        camera.set_horizontal_aperture(spec.horizontal_aperture_mm)
        camera.set_clipping_range(*spec.clipping_range)
        camera.initialize(attach_rgb_annotator=True)
        camera.add_distance_to_image_plane_to_frame()
    return cameras


def _quaternion_rotate_vector(
    quaternion: Sequence[float], vector: Sequence[float]
) -> list[float]:
    """Rotate a 3-vector by a wxyz quaternion."""

    import numpy as np

    w, x, y, z = normalize_quaternion(quaternion)
    rotation = np.asarray(
        (
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ),
            (
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ),
            (
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        ),
        dtype=np.float64,
    )
    return (rotation @ np.asarray(vector, dtype=np.float64)).tolist()


def read_camera_observations(cameras: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Read RGB uint8 and metric image-plane depth, normalizing invalid depth."""

    import numpy as np

    observations: dict[str, dict[str, Any]] = {}
    for spec in CAMERA_SPECS:
        camera = cameras[spec.name]
        rgb_data = camera.get_rgb(device="cpu")
        depth_data = camera.get_depth(device="cpu")
        if rgb_data is None or depth_data is None:
            raise RuntimeError(f"{spec.name}: RGB-D annotators returned no data")
        rgb = np.asarray(rgb_data)
        depth = np.asarray(depth_data)
        if rgb.shape != (spec.resolution[1], spec.resolution[0], 3):
            raise ValueError(f"{spec.name}: unexpected RGB shape {rgb.shape}")
        if depth.shape != (spec.resolution[1], spec.resolution[0]):
            raise ValueError(f"{spec.name}: unexpected depth shape {depth.shape}")
        if rgb.dtype != np.uint8:
            raise ValueError(f"{spec.name}: expected RGB uint8, found {rgb.dtype}")
        if depth.dtype != np.float32:
            raise ValueError(f"{spec.name}: expected depth float32, found {depth.dtype}")
        depth = depth.copy()
        depth[~np.isfinite(depth)] = np.nan
        observations[spec.name] = {
            "rgb": rgb.copy(),
            "depth": depth,
            "rendering_time_s": float(
                camera.get_current_frame().get("rendering_time", 0.0)
            ),
        }
    return observations


def capture_episode_initial_state(
    robots: dict[str, Any],
    dolls: dict[str, Any],
) -> dict[str, dict[str, Any]]:
    """Capture the exact post-warmup state used as the replay reset state."""

    import numpy as np

    robot_position = np.stack(
        [
            np.asarray(
                robots[name].get_joint_positions(),
                dtype=np.float32,
            )
            for name in EPISODE_ROBOT_NAMES
        ]
    )
    robot_velocity = np.stack(
        [
            np.asarray(
                robots[name].get_joint_velocities(),
                dtype=np.float32,
            )
            for name in EPISODE_ROBOT_NAMES
        ]
    )
    robot_action = np.concatenate(
        (robot_position[:, :6], robot_position[:, 6:7]),
        axis=1,
    )
    states = _doll_state_report(dolls)
    object_pose = np.asarray(
        [
            [
                *states[asset_id]["position_m"],
                *states[asset_id]["quaternion_wxyz"],
            ]
            for asset_id in EPISODE_OBJECT_IDS
        ],
        dtype=np.float32,
    )
    object_linear_velocity = np.asarray(
        [
            states[asset_id]["linear_velocity_m_s"]
            for asset_id in EPISODE_OBJECT_IDS
        ],
        dtype=np.float32,
    )
    object_angular_velocity = np.asarray(
        [
            states[asset_id]["angular_velocity_rad_s"]
            for asset_id in EPISODE_OBJECT_IDS
        ],
        dtype=np.float32,
    )
    target_pose = np.asarray(
        [
            [
                *placement.pose.position,
                *placement.pose.quaternion,
            ]
            for placement in compute_doll_target_layout()
        ],
        dtype=np.float32,
    )
    return {
        "robots": {
            "joint_position": robot_position,
            "joint_velocity": robot_velocity,
            "joint_action": robot_action,
        },
        "objects": {
            "pose": object_pose,
            "linear_velocity": object_linear_velocity,
            "angular_velocity": object_angular_velocity,
        },
        "targets": {"object_pose": target_pose},
    }


class IsaacEpisodeRecorder:
    """Bridge live Isaac state and cameras into ``Hdf5EpisodeWriter``."""

    def __init__(
        self,
        writer: Hdf5EpisodeWriter,
        *,
        world: Any,
        robots: dict[str, Any],
        dolls: dict[str, Any],
        cameras: dict[str, Any],
        initial_state: dict[str, dict[str, Any]],
    ) -> None:
        import numpy as np

        if set(robots) != set(EPISODE_ROBOT_NAMES):
            raise ValueError(f"Unexpected robot names: {sorted(robots)}")
        if set(dolls) != set(EPISODE_OBJECT_IDS):
            raise ValueError(f"Unexpected object IDs: {sorted(dolls)}")
        if set(cameras) != set(EPISODE_CAMERA_NAMES):
            raise ValueError(f"Unexpected camera names: {sorted(cameras)}")
        self.writer = writer
        self.world = world
        self.robots = robots
        self.dolls = dolls
        self.cameras = cameras
        self._joint_action = np.asarray(
            initial_state["robots"]["joint_action"],
            dtype=np.float32,
        ).copy()
        self._phase = "initial_hold"
        self._operator = ""
        self._object_id = ""
        self._event_code = np.full(
            len(EPISODE_ROBOT_NAMES),
            GRASP_EVENT_NONE,
            dtype=np.int8,
        )
        self._event_object_index = np.full(
            len(EPISODE_ROBOT_NAMES),
            -1,
            dtype=np.int8,
        )
        self._event_relative_pose = np.full(
            (len(EPISODE_ROBOT_NAMES), 7),
            np.nan,
            dtype=np.float32,
        )

    def _robot_index(self, robot: Any) -> int:
        for index, name in enumerate(EPISODE_ROBOT_NAMES):
            if self.robots[name] is robot:
                return index
        raise ValueError("Action refers to an unregistered robot instance")

    def set_arm_action(
        self,
        robot: Any,
        joint_position: Sequence[float],
    ) -> None:
        import numpy as np

        command = np.asarray(joint_position, dtype=np.float32)
        if command.shape != (len(PIPER_ARM_JOINT_NAMES),):
            raise ValueError(f"Expected six arm targets, found {command.shape}")
        self._joint_action[self._robot_index(robot), :6] = command

    def set_gripper_action(self, robot: Any, position: float) -> None:
        self._joint_action[self._robot_index(robot), 6] = float(position)

    def set_task_context(
        self,
        phase: str,
        *,
        operator: str = "",
        object_id: str = "",
    ) -> None:
        if operator and operator not in EPISODE_ROBOT_NAMES:
            raise ValueError(f"Unknown operator {operator!r}")
        if object_id and object_id not in EPISODE_OBJECT_IDS:
            raise ValueError(f"Unknown object ID {object_id!r}")
        if len(phase.encode("utf-8")) > 48:
            raise ValueError(f"Task phase is too long: {phase!r}")
        self._phase = phase
        self._operator = operator
        self._object_id = object_id

    def queue_grasp_event(
        self,
        event_code: int,
        robot_name: str,
        asset_id: str,
        *,
        relative_pose: PoseSpec | None = None,
    ) -> None:
        import numpy as np

        if event_code not in (GRASP_EVENT_ATTACH, GRASP_EVENT_DETACH):
            raise ValueError(f"Cannot queue grasp event {event_code}")
        robot_index = EPISODE_ROBOT_NAMES.index(robot_name)
        if self._event_code[robot_index] != GRASP_EVENT_NONE:
            raise RuntimeError(
                f"{robot_name} already has a pending grasp event"
            )
        self._event_code[robot_index] = event_code
        self._event_object_index[robot_index] = (
            EPISODE_OBJECT_IDS.index(asset_id)
        )
        if event_code == GRASP_EVENT_ATTACH:
            if relative_pose is None:
                raise ValueError("An attach event requires its relative pose")
            self._event_relative_pose[robot_index] = np.asarray(
                [*relative_pose.position, *relative_pose.quaternion],
                dtype=np.float32,
            )

    def capture_frame(self) -> None:
        """Read all state and all three rendered camera streams at one index."""

        import numpy as np

        observations = read_camera_observations(self.cameras)
        robot_positions: list[Any] = []
        robot_velocities: list[Any] = []
        end_effector_poses: list[list[float]] = []
        for name in EPISODE_ROBOT_NAMES:
            robot = self.robots[name]
            robot_positions.append(
                np.asarray(robot.get_joint_positions(), dtype=np.float32)
            )
            robot_velocities.append(
                np.asarray(robot.get_joint_velocities(), dtype=np.float32)
            )
            spec = _robot_spec_by_name(name)
            position, quaternion = _xform_world_pose(
                f"{spec.prim_path}/{PIPER_TOOL_REL_PATH}",
                f"{name}_episode_tool",
            )
            end_effector_poses.append([*position, *quaternion])

        object_states = _doll_state_report(self.dolls)
        camera_frames: dict[str, dict[str, Any]] = {}
        for camera_name in EPISODE_CAMERA_NAMES:
            camera_position, camera_quaternion = self.cameras[
                camera_name
            ].get_world_pose(camera_axes="usd")
            camera_pose = PoseSpec(
                tuple(float(value) for value in camera_position),
                tuple(float(value) for value in camera_quaternion),
            )
            camera_frames[camera_name] = {
                **observations[camera_name],
                "world_pose": np.asarray(
                    [*camera_pose.position, *camera_pose.quaternion],
                    dtype=np.float32,
                ),
                "world_to_camera": np.asarray(
                    world_to_local_matrix(camera_pose),
                    dtype=np.float32,
                ),
            }

        world_time = getattr(self.world, "current_time", 0.0)
        if callable(world_time):
            world_time = world_time()
        frame = {
            "simulation_time_s": (
                (self.writer.frame_count + 1) * CONTROL_DT
            ),
            "world_time_s": float(world_time),
            "robots": {
                "joint_position": np.asarray(
                    robot_positions,
                    dtype=np.float32,
                ),
                "joint_velocity": np.asarray(
                    robot_velocities,
                    dtype=np.float32,
                ),
                "joint_action": self._joint_action.copy(),
                "end_effector_world_pose": np.asarray(
                    end_effector_poses,
                    dtype=np.float32,
                ),
            },
            "objects": {
                "world_pose": np.asarray(
                    [
                        [
                            *object_states[asset_id]["position_m"],
                            *object_states[asset_id][
                                "quaternion_wxyz"
                            ],
                        ]
                        for asset_id in EPISODE_OBJECT_IDS
                    ],
                    dtype=np.float32,
                ),
                "linear_velocity": np.asarray(
                    [
                        object_states[asset_id]["linear_velocity_m_s"]
                        for asset_id in EPISODE_OBJECT_IDS
                    ],
                    dtype=np.float32,
                ),
                "angular_velocity": np.asarray(
                    [
                        object_states[asset_id][
                            "angular_velocity_rad_s"
                        ]
                        for asset_id in EPISODE_OBJECT_IDS
                    ],
                    dtype=np.float32,
                ),
            },
            "task": {
                "phase": self._phase,
                "operator": self._operator,
                "object_id": self._object_id,
            },
            "control": {
                "grasp_event_code": self._event_code.copy(),
                "grasp_event_object_index": (
                    self._event_object_index.copy()
                ),
                "grasp_event_relative_pose": (
                    self._event_relative_pose.copy()
                ),
            },
            "cameras": camera_frames,
        }
        self.writer.append_frame(frame)
        self._event_code.fill(GRASP_EVENT_NONE)
        self._event_object_index.fill(-1)
        self._event_relative_pose.fill(np.nan)


def _camera_target(name: str) -> tuple[float, float, float]:
    if name == LEFT_WRIST_CAMERA_NAME:
        path = f"{LEFT_PIPER_PRIM_PATH}/{PIPER_TOOL_REL_PATH}"
        return tuple(_xform_world_pose(path, "left_camera_target")[0])
    if name == RIGHT_WRIST_CAMERA_NAME:
        path = f"{RIGHT_PIPER_PRIM_PATH}/{PIPER_TOOL_REL_PATH}"
        return tuple(_xform_world_pose(path, "right_camera_target")[0])
    return OVERHEAD_CAMERA_TARGET


def validate_and_capture_cameras(
    world: Any,
    cameras: dict[str, Any],
    *,
    output_dir: Path | None = None,
) -> dict[str, Any]:
    """Render, validate calibration/poses/data/cadence, and optionally save previews."""

    import numpy as np

    expected_names = {spec.name for spec in CAMERA_SPECS}
    if set(cameras) != expected_names:
        raise ValueError(
            f"Expected cameras {sorted(expected_names)}, found {sorted(cameras)}"
        )

    timestamps: dict[str, list[float]] = {name: [] for name in cameras}
    for _ in range(CAMERA_RENDER_WARMUP_STEPS):
        world.step(render=True)
        for name, camera in cameras.items():
            timestamp = float(
                camera.get_current_frame().get("rendering_time", 0.0)
            )
            if timestamp > 0.0 and (
                not timestamps[name]
                or not math.isclose(
                    timestamp, timestamps[name][-1], abs_tol=1.0e-9
                )
            ):
                timestamps[name].append(timestamp)

    observations = read_camera_observations(cameras)
    specs = _camera_spec_by_name()
    report: dict[str, Any] = {}
    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)

    for name, camera in cameras.items():
        spec = specs[name]
        prim_path = CAMERA_PRIM_PATHS[name]
        prim = world.stage.GetPrimAtPath(prim_path)
        if not prim or prim.GetTypeName() != "Camera":
            raise ValueError(f"{name}: missing Camera prim at {prim_path}")
        if tuple(camera.get_resolution()) != spec.resolution:
            raise ValueError(f"{name}: resolution configuration changed")
        if not math.isclose(
            float(camera.get_frequency()), spec.frequency_hz, abs_tol=1.0e-9
        ):
            raise ValueError(f"{name}: camera frequency is not 30 Hz")
        if not math.isclose(
            float(camera.get_focal_length()),
            spec.focal_length_mm,
            abs_tol=1.0e-5,
        ):
            raise ValueError(f"{name}: focal length configuration changed")
        if not math.isclose(
            float(camera.get_horizontal_aperture()),
            spec.horizontal_aperture_mm,
            abs_tol=1.0e-5,
        ):
            raise ValueError(f"{name}: horizontal aperture configuration changed")
        clipping_range = tuple(float(value) for value in camera.get_clipping_range())
        _assert_vector_close(
            clipping_range,
            spec.clipping_range,
            label=f"{name} clipping range",
            tolerance=1.0e-5,
        )

        actual_intrinsics = np.asarray(
            camera.get_intrinsics_matrix(device="cpu"), dtype=np.float64
        )
        expected_intrinsics = np.asarray(camera_intrinsics(spec), dtype=np.float64)
        if not np.allclose(actual_intrinsics, expected_intrinsics, atol=1.0e-4):
            raise ValueError(
                f"{name}: intrinsics differ: {actual_intrinsics.tolist()}"
            )

        position, quaternion = camera.get_world_pose(camera_axes="world")
        usd_pose: dict[str, list[float]] | None = None
        if name == OVERHEAD_CAMERA_NAME:
            usd_position, usd_quaternion = camera.get_world_pose(camera_axes="usd")
            _assert_vector_close(
                usd_position,
                OVERHEAD_CAMERA_POSITION,
                label="overhead camera world position",
                tolerance=2.0e-5,
            )
            usd_orientation_error = quaternion_angular_error(
                usd_quaternion,
                OVERHEAD_CAMERA_USD_ORIENTATION,
            )
            if usd_orientation_error > 2.0e-5:
                raise ValueError(
                    "overhead camera USD-frame orientation error "
                    f"{usd_orientation_error:.8f} rad"
                )
            usd_pose = {
                "position_m": np.asarray(usd_position).tolist(),
                "quaternion_wxyz": np.asarray(usd_quaternion).tolist(),
            }
        target = np.asarray(_camera_target(name), dtype=np.float64)
        optical_forward = np.asarray(
            _quaternion_rotate_vector(quaternion, (1.0, 0.0, 0.0)),
            dtype=np.float64,
        )
        target_direction = target - np.asarray(position, dtype=np.float64)
        target_distance = float(np.linalg.norm(target_direction))
        target_direction /= target_distance
        target_alignment = float(np.dot(optical_forward, target_direction))
        if target_alignment < math.cos(math.radians(30.0)):
            raise ValueError(
                f"{name}: optical axis misses its target, cosine={target_alignment}"
            )
        target_pixel = np.asarray(
            camera.get_image_coords_from_world_points(target.reshape(1, 3)),
            dtype=np.float64,
        )[0]
        width, height = spec.resolution
        if not (
            CAMERA_TARGET_PIXEL_MARGIN
            <= target_pixel[0]
            < width - CAMERA_TARGET_PIXEL_MARGIN
            and CAMERA_TARGET_PIXEL_MARGIN
            <= target_pixel[1]
            < height - CAMERA_TARGET_PIXEL_MARGIN
        ):
            raise ValueError(
                f"{name}: target projects outside the image at {target_pixel.tolist()}"
            )
        workspace_pixels: list[list[float]] | None = None
        if name == OVERHEAD_CAMERA_NAME:
            doll_specs = get_doll_specs()
            maximum_radius = max(spec.footprint_radius for spec in doll_specs)
            maximum_height = max(spec.height for spec in doll_specs)
            random_envelope_points = [
                (x, y, z)
                for x in (
                    MATRYOSHKA_RANDOM_X_RANGE[0] - maximum_radius,
                    MATRYOSHKA_RANDOM_X_RANGE[1] + maximum_radius,
                )
                for y in (
                    MATRYOSHKA_RANDOM_Y_RANGE[0] - maximum_radius,
                    MATRYOSHKA_RANDOM_Y_RANGE[1] + maximum_radius,
                )
                for z in (
                    TABLE_TOP_Z,
                    TABLE_TOP_Z + maximum_height,
                )
            ]
            specs_by_id = {spec.asset_id: spec for spec in doll_specs}
            target_envelope_points = [
                (
                    placement.pose.position[0] + x_sign * spec.footprint_radius,
                    placement.pose.position[1] + y_sign * spec.footprint_radius,
                    TABLE_TOP_Z + z_fraction * spec.height,
                )
                for placement in compute_doll_target_layout()
                for spec in (specs_by_id[placement.asset_id],)
                for x_sign in (-1.0, 1.0)
                for y_sign in (-1.0, 1.0)
                for z_fraction in (0.0, 1.0)
            ]
            workspace_points = np.asarray(
                random_envelope_points + target_envelope_points,
                dtype=np.float64,
            )
            projected_workspace = np.asarray(
                camera.get_image_coords_from_world_points(workspace_points),
                dtype=np.float64,
            )
            if not np.all(
                (projected_workspace[:, 0] >= CAMERA_TARGET_PIXEL_MARGIN)
                & (
                    projected_workspace[:, 0]
                    < width - CAMERA_TARGET_PIXEL_MARGIN
                )
                & (projected_workspace[:, 1] >= CAMERA_TARGET_PIXEL_MARGIN)
                & (
                    projected_workspace[:, 1]
                    < height - CAMERA_TARGET_PIXEL_MARGIN
                )
            ):
                raise ValueError(
                    "Overhead camera does not cover complete doll bounds in "
                    "the random and target regions: "
                    f"{projected_workspace.tolist()}"
                )
            workspace_pixels = projected_workspace.tolist()

        rgb = observations[name]["rgb"]
        depth = observations[name]["depth"]
        finite_depth = np.isfinite(depth)
        finite_ratio = float(np.mean(finite_depth))
        if finite_ratio <= 0.05:
            raise ValueError(f"{name}: only {finite_ratio:.3%} finite depth pixels")
        if int(np.ptp(rgb.astype(np.int16))) == 0:
            raise ValueError(f"{name}: rendered RGB frame is uniform")
        finite_values = depth[finite_depth]
        if (
            float(np.min(finite_values)) < spec.clipping_range[0] - 1.0e-3
            or float(np.max(finite_values)) > spec.clipping_range[1] + 1.0e-3
        ):
            raise ValueError(f"{name}: finite depth violates the clipping range")

        unique_timestamps = timestamps[name]
        cadence = [
            later - earlier
            for earlier, later in zip(unique_timestamps, unique_timestamps[1:])
        ]
        if len(cadence) < 2:
            raise RuntimeError(
                f"{name}: insufficient timestamp changes to verify camera cadence"
            )
        median_period = float(np.median(np.asarray(cadence)))
        if not math.isclose(
            median_period, 1.0 / spec.frequency_hz, abs_tol=5.0e-3
        ):
            raise ValueError(
                f"{name}: observed period {median_period:.6f} s is not 30 Hz"
            )

        preview: dict[str, Any] = {}
        if output_dir is not None:
            from PIL import Image

            rgb_path = output_dir / f"{name}_rgb.png"
            depth_path = output_dir / f"{name}_depth.png"
            Image.fromarray(rgb).save(rgb_path)
            depth_visual = np.zeros(depth.shape, dtype=np.uint8)
            low, high = np.percentile(finite_values, (1.0, 99.0))
            if high > low:
                normalized = np.clip((depth - low) / (high - low), 0.0, 1.0)
                depth_visual[finite_depth] = (
                    255.0 * (1.0 - normalized[finite_depth])
                ).astype(np.uint8)
            Image.fromarray(depth_visual).save(depth_path)
            preview = {
                "rgb_path": str(rgb_path.resolve()),
                "depth_path": str(depth_path.resolve()),
            }

        report[name] = {
            "prim_path": prim_path,
            "parent_path": str(prim.GetParent().GetPath()),
            "model": "logical Intel RealSense D435 pinhole RGB-D",
            "resolution_wh": list(spec.resolution),
            "frequency_hz": spec.frequency_hz,
            "observed_period_s": median_period,
            "rgb": {
                "shape": list(rgb.shape),
                "dtype": str(rgb.dtype),
                "channel_order": CAMERA_RGB_CHANNEL_ORDER,
                "minimum": int(np.min(rgb)),
                "maximum": int(np.max(rgb)),
            },
            "depth": {
                "shape": list(depth.shape),
                "dtype": str(depth.dtype),
                "definition": spec.depth_definition,
                "unit": CAMERA_DEPTH_UNIT,
                "invalid_value": CAMERA_INVALID_DEPTH,
                "finite_ratio": finite_ratio,
                "finite_minimum_m": float(np.min(finite_values)),
                "finite_maximum_m": float(np.max(finite_values)),
            },
            "calibration": {
                "focal_length_mm": float(camera.get_focal_length()),
                "horizontal_aperture_mm": float(
                    camera.get_horizontal_aperture()
                ),
                "clipping_range_m": list(clipping_range),
                "intrinsics": actual_intrinsics.tolist(),
                "world_position_m": np.asarray(position).tolist(),
                "world_quaternion_wxyz": np.asarray(quaternion).tolist(),
                "usd_world_pose": usd_pose,
            },
            "coverage": {
                "target_world_m": target.tolist(),
                "target_pixel_uv": target_pixel.tolist(),
                "optical_axis_target_cosine": target_alignment,
                "target_distance_m": target_distance,
                "random_and_target_doll_bounds_pixels_uv": workspace_pixels,
            },
            "rendering_time_s": observations[name]["rendering_time_s"],
            **preview,
        }
    return report


def summarize_sort_result(report: dict[str, Any]) -> dict[str, Any]:
    """Strip dense planner trajectories while preserving acceptance evidence."""

    return {
        "success": bool(report["success"]),
        "planner": report["planner"],
        "planner_seed_per_operation": report[
            "planner_seed_per_operation"
        ],
        "pick_order": report["pick_order"],
        "assignments": report["assignments"],
        "participating_robots": report["participating_robots"],
        "placements": [
            {
                "asset_id": operation["asset_id"],
                "active_robot": operation["active_robot"],
                "final_place_error_m": operation["release"][
                    "final_place_error_m"
                ],
                "final_upright_tilt_degrees": operation["release"][
                    "final_doll_state"
                ]["upright_tilt_degrees"],
            }
            for operation in report["operations"]
        ],
        "final_validation": report["final_validation"],
        "final_robots": report["final_robots"],
    }


def run_hdf5_collection_worker(args: argparse.Namespace) -> dict[str, Any]:
    """Run one cuRobo expert episode and leave it pending replay."""

    if args.episode is None:
        raise ValueError("collect-worker requires --episode")
    episode_path = args.episode.resolve()
    seeds = set_episode_random_seeds(args.seed)
    layout = sample_initial_doll_layout(args.seed)
    world = create_scene()
    robots = create_robots(world)
    dolls = create_dolls(world, layout)
    settle_and_validate_dolls(world, dolls)
    cameras = create_cameras(world)
    camera_report = validate_and_capture_cameras(world, cameras)
    initial_dolls = settle_and_validate_dolls(world, dolls)
    initial_state = capture_episode_initial_state(robots, dolls)
    metadata = build_episode_metadata(
        episode_id=episode_path.stem.removesuffix(".partial"),
        seed=args.seed,
        planner_seed=args.planner_seed,
        sampled_layout=layout,
        initial_doll_report=initial_dolls,
        camera_report=camera_report,
    )
    writer = Hdf5EpisodeWriter(
        episode_path,
        metadata=metadata,
        initial_state=initial_state,
    )
    recorder = IsaacEpisodeRecorder(
        writer,
        world=world,
        robots=robots,
        dolls=dolls,
        cameras=cameras,
        initial_state=initial_state,
    )
    try:
        _set_recording_context(recorder, "initial_hold")
        _advance_control_frame(
            world,
            render=not args.headless,
            episode_recorder=recorder,
        )
        sort_report = run_full_curobo_sort(
            world,
            robots,
            dolls,
            planner_seed=args.planner_seed,
            render=not args.headless,
            episode_recorder=recorder,
        )
        expert_summary = summarize_sort_result(sort_report)
        writer.finish_expert(
            success=True,
            summary=expert_summary,
        )
    except Exception as error:
        writer.finish_expert(
            success=False,
            failure_reason=f"{type(error).__name__}: {error}",
        )
        raise
    finally:
        writer.close()

    schema = validate_episode_hdf5(episode_path)
    return {
        "success": True,
        "episode_path": str(episode_path),
        "seeds": {
            **seeds,
            "curobo_motion_planner": args.planner_seed,
        },
        "sampled_layout": [asdict(placement) for placement in layout],
        "expert": expert_summary,
        "schema": schema,
        "bytes": episode_path.stat().st_size,
    }


def _episode_layout_from_metadata(
    metadata: dict[str, Any],
) -> tuple[DollPlacement, ...]:
    objects_by_id = {
        str(item["asset_id"]): item for item in metadata["objects"]
    }
    if set(objects_by_id) != set(EPISODE_OBJECT_IDS):
        raise ValueError(
            "Recorded object IDs differ from this task: "
            f"{sorted(objects_by_id)}"
        )
    placements: list[DollPlacement] = []
    for asset_id in EPISODE_OBJECT_IDS:
        item = objects_by_id[asset_id]
        placements.append(
            DollPlacement(
                asset_id=asset_id,
                pose=_pose_spec_from_mapping(item["sampled_pose"]),
                yaw_rad=float(item["sampled_yaw_rad"]),
            )
        )
    validate_initial_doll_layout(placements)
    return tuple(placements)


def validate_episode_replay_compatibility(
    metadata: dict[str, Any],
) -> dict[str, Any]:
    """Reject replay if the current scene/control contract has changed."""

    timing = metadata["timing"]
    expected_timing = {
        "physics_frequency_hz": PHYSICS_FREQUENCY_HZ,
        "control_frequency_hz": CONTROL_FREQUENCY_HZ,
        "render_frequency_hz": RENDER_FREQUENCY_HZ,
        "camera_frequency_hz": CAMERA_FREQUENCY_HZ,
        "physics_steps_per_control": (
            PHYSICS_FREQUENCY_HZ // CONTROL_FREQUENCY_HZ
        ),
    }
    for key, expected in expected_timing.items():
        if int(timing[key]) != expected:
            raise ValueError(
                f"Recorded {key}={timing[key]} differs from {expected}"
            )
    expected_assets = {
        "piper_usd": str(PIPER_USD.resolve()),
        "piper_urdf": str(PIPER_URDF.resolve()),
        "room_usd": str(ROOM_USD.resolve()),
        "camera_stand_usd": str(CAMERA_STAND_USD.resolve()),
        "hdr_texture": str(HDR_TEXTURE.resolve()),
        "table_mdl": str(TABLE_MDL.resolve()),
        "ground_mdl": str(GROUND_MDL.resolve()),
    }
    for key, expected in expected_assets.items():
        if metadata["assets"].get(key) != expected:
            raise ValueError(f"Recorded asset {key} differs from {expected}")
    robot_names = tuple(robot["name"] for robot in metadata["robots"])
    camera_names = tuple(camera["name"] for camera in metadata["cameras"])
    object_ids = tuple(item["asset_id"] for item in metadata["objects"])
    if robot_names != EPISODE_ROBOT_NAMES:
        raise ValueError(f"Recorded robot order changed: {robot_names}")
    if camera_names != EPISODE_CAMERA_NAMES:
        raise ValueError(f"Recorded camera order changed: {camera_names}")
    if object_ids != EPISODE_OBJECT_IDS:
        raise ValueError(f"Recorded object order changed: {object_ids}")
    for camera in metadata["cameras"]:
        if tuple(camera["resolution_wh"]) != CAMERA_RESOLUTION:
            raise ValueError(
                f"{camera['name']}: recorded resolution changed"
            )
    return {
        "timing": expected_timing,
        "assets": expected_assets,
        "robot_names": list(robot_names),
        "camera_names": list(camera_names),
        "object_ids": list(object_ids),
    }


def restore_episode_initial_state(
    episode: Any,
    robots: dict[str, Any],
    dolls: dict[str, Any],
) -> dict[str, Any]:
    """Restore exact recorded joint/object pose and velocity arrays."""

    import numpy as np

    robot_position = np.asarray(
        episode["initial/robots/joint_position"][:],
        dtype=np.float32,
    )
    robot_velocity = np.asarray(
        episode["initial/robots/joint_velocity"][:],
        dtype=np.float32,
    )
    robot_action = np.asarray(
        episode["initial/robots/joint_action"][:],
        dtype=np.float32,
    )
    for index, robot_name in enumerate(EPISODE_ROBOT_NAMES):
        robot = robots[robot_name]
        robot.set_joint_positions(robot_position[index])
        robot.set_joint_velocities(robot_velocity[index])
        _command_position(
            robot,
            robot_action[index],
            range(len(PIPER_COMMAND_JOINT_NAMES)),
        )

    object_pose = np.asarray(
        episode["initial/objects/pose"][:],
        dtype=np.float32,
    )
    object_linear_velocity = np.asarray(
        episode["initial/objects/linear_velocity"][:],
        dtype=np.float32,
    )
    object_angular_velocity = np.asarray(
        episode["initial/objects/angular_velocity"][:],
        dtype=np.float32,
    )
    for index, asset_id in enumerate(EPISODE_OBJECT_IDS):
        doll = dolls[asset_id]
        doll.set_world_pose(
            position=object_pose[index, :3],
            orientation=object_pose[index, 3:],
        )
        doll.set_linear_velocity(object_linear_velocity[index])
        doll.set_angular_velocity(object_angular_velocity[index])

    maximum_robot_position_error = 0.0
    maximum_robot_velocity_error = 0.0
    for index, robot_name in enumerate(EPISODE_ROBOT_NAMES):
        robot = robots[robot_name]
        maximum_robot_position_error = max(
            maximum_robot_position_error,
            float(
                np.max(
                    np.abs(
                        np.asarray(
                            robot.get_joint_positions(),
                            dtype=np.float64,
                        )
                        - robot_position[index]
                    )
                )
            ),
        )
        maximum_robot_velocity_error = max(
            maximum_robot_velocity_error,
            float(
                np.max(
                    np.abs(
                        np.asarray(
                            robot.get_joint_velocities(),
                            dtype=np.float64,
                        )
                        - robot_velocity[index]
                    )
                )
            ),
        )
    maximum_object_position_error = 0.0
    maximum_object_orientation_error = 0.0
    for index, asset_id in enumerate(EPISODE_OBJECT_IDS):
        position, quaternion = dolls[asset_id].get_world_pose()
        maximum_object_position_error = max(
            maximum_object_position_error,
            float(
                np.max(
                    np.abs(
                        np.asarray(position, dtype=np.float64)
                        - object_pose[index, :3]
                    )
                )
            ),
        )
        maximum_object_orientation_error = max(
            maximum_object_orientation_error,
            quaternion_angular_error(
                quaternion,
                object_pose[index, 3:],
            ),
        )
    if maximum_robot_position_error > 1.0e-6:
        raise RuntimeError(
            "Robot initial-state restore error "
            f"{maximum_robot_position_error:.9f} rad/m"
        )
    if maximum_robot_velocity_error > 1.0e-6:
        raise RuntimeError(
            "Robot velocity restore error "
            f"{maximum_robot_velocity_error:.9f}"
        )
    if maximum_object_position_error > 1.0e-6:
        raise RuntimeError(
            "Object initial-state restore error "
            f"{maximum_object_position_error:.9f} m"
        )
    if maximum_object_orientation_error > 1.0e-6:
        raise RuntimeError(
            "Object orientation restore error "
            f"{maximum_object_orientation_error:.9f} rad"
        )
    return {
        "maximum_robot_position_error": maximum_robot_position_error,
        "maximum_robot_velocity_error": maximum_robot_velocity_error,
        "maximum_object_position_error_m": maximum_object_position_error,
        "maximum_object_orientation_error_rad": (
            maximum_object_orientation_error
        ),
    }


def update_episode_replay_result(
    path: Path,
    *,
    success: bool,
    summary: dict[str, Any] | None = None,
    failure_reason: str = "",
) -> None:
    """Atomically update logical replay/acceptance flags inside one HDF5."""

    import h5py

    if success and failure_reason:
        raise ValueError("A successful replay cannot have a failure reason")
    with h5py.File(path.resolve(), "r+") as episode:
        expert_success = bool(
            episode.attrs.get("expert_success", False)
        )
        replay_success = bool(success)
        accepted = expert_success and replay_success
        episode.attrs["replay_success"] = replay_success
        episode.attrs["accepted"] = accepted
        episode.attrs["failure_reason"] = failure_reason
        episode.attrs["writer_state"] = (
            "accepted" if accepted else "replay_failed"
        )
        episode["results/replay_summary_json"][()] = json.dumps(
            {} if summary is None else summary,
            sort_keys=True,
        )
        episode.flush()


def run_hdf5_replay_worker(args: argparse.Namespace) -> dict[str, Any]:
    """Rebuild a clean scene and replay HDF5 actions without cuRobo."""

    import h5py
    import numpy as np

    if args.episode is None:
        raise ValueError("replay-worker requires --episode")
    episode_path = args.episode.resolve()
    schema_before = validate_episode_hdf5(episode_path)
    if not schema_before["expert_success"]:
        raise ValueError("Cannot replay an episode whose expert run failed")

    try:
        with h5py.File(episode_path, "r") as episode:
            metadata = json.loads(
                _decode_hdf5_text(episode["metadata/json"][()])
            )
            compatibility = validate_episode_replay_compatibility(metadata)
            seed = int(metadata["seeds"]["episode"])
            set_episode_random_seeds(seed)
            layout = _episode_layout_from_metadata(metadata)

            world = create_scene()
            robots = create_robots(world)
            dolls = create_dolls(world, layout)
            cameras = create_cameras(world)
            camera_validation = validate_and_capture_cameras(world, cameras)
            restore_report = restore_episode_initial_state(
                episode,
                robots,
                dolls,
            )

            joint_actions = episode["frames/robots/joint_action"]
            event_codes = episode["frames/control/grasp_event_code"]
            event_object_indices = episode[
                "frames/control/grasp_event_object_index"
            ]
            event_relative_poses = episode[
                "frames/control/grasp_event_relative_pose"
            ]
            phases = episode["frames/task/phase"]
            frame_count = int(episode.attrs["frame_count"])
            active_attachments: dict[str, str | None] = {
                name: None for name in EPISODE_ROBOT_NAMES
            }
            physics_steps_per_control = (
                PHYSICS_FREQUENCY_HZ // CONTROL_FREQUENCY_HZ
            )
            for frame_index in range(frame_count):
                codes = np.asarray(
                    event_codes[frame_index],
                    dtype=np.int8,
                )
                object_indices = np.asarray(
                    event_object_indices[frame_index],
                    dtype=np.int8,
                )
                relative_poses = np.asarray(
                    event_relative_poses[frame_index],
                    dtype=np.float64,
                )
                for robot_index, code_value in enumerate(codes):
                    code = int(code_value)
                    if code == GRASP_EVENT_NONE:
                        continue
                    robot_name = EPISODE_ROBOT_NAMES[robot_index]
                    object_index = int(object_indices[robot_index])
                    if not 0 <= object_index < len(EPISODE_OBJECT_IDS):
                        raise RuntimeError(
                            f"Frame {frame_index}: invalid event object index "
                            f"{object_index}"
                        )
                    asset_id = EPISODE_OBJECT_IDS[object_index]
                    spec = _robot_spec_by_name(robot_name)
                    if code == GRASP_EVENT_ATTACH:
                        if active_attachments[robot_name] is not None:
                            raise RuntimeError(
                                f"Frame {frame_index}: {robot_name} already "
                                "has an attachment"
                            )
                        values = relative_poses[robot_index]
                        if not np.all(np.isfinite(values)):
                            raise RuntimeError(
                                f"Frame {frame_index}: attach pose is invalid"
                            )
                        create_simulation_grasp_joint(
                            world,
                            spec,
                            asset_id,
                            dolls[asset_id],
                            relative_pose=PoseSpec(
                                tuple(float(value) for value in values[:3]),
                                tuple(float(value) for value in values[3:]),
                            ),
                            step_after_change=False,
                        )
                        active_attachments[robot_name] = asset_id
                    elif code == GRASP_EVENT_DETACH:
                        if active_attachments[robot_name] != asset_id:
                            raise RuntimeError(
                                f"Frame {frame_index}: detach {robot_name}/"
                                f"{asset_id} does not match "
                                f"{active_attachments[robot_name]}"
                            )
                        remove_simulation_grasp_joint(
                            world,
                            spec,
                            asset_id,
                            step_after_change=False,
                        )
                        active_attachments[robot_name] = None
                    else:
                        raise RuntimeError(
                            f"Frame {frame_index}: unknown event code {code}"
                        )

                actions = np.asarray(
                    joint_actions[frame_index],
                    dtype=np.float32,
                )
                for robot_index, robot_name in enumerate(
                    EPISODE_ROBOT_NAMES
                ):
                    _command_position(
                        robots[robot_name],
                        actions[robot_index],
                        range(len(PIPER_COMMAND_JOINT_NAMES)),
                    )
                for substep in range(physics_steps_per_control):
                    world.step(
                        render=(
                            not args.headless
                            and substep == physics_steps_per_control - 1
                        )
                    )
                if (
                    (frame_index + 1) % 500 == 0
                    or frame_index + 1 == frame_count
                ):
                    phase = _decode_hdf5_text(phases[frame_index])
                    print(
                        "HDF5_REPLAY_PROGRESS "
                        f"{frame_index + 1}/{frame_count} phase={phase}",
                        flush=True,
                    )

            if any(active_attachments.values()):
                raise RuntimeError(
                    f"Replay ended with attachments: {active_attachments}"
                )
            final_result = validate_task_success(
                world,
                robots,
                dolls,
                label="hdf5_replay_final",
                render=not args.headless,
            )
            replay_summary = {
                "success": True,
                "planner_invocations": 0,
                "frame_count": frame_count,
                "initial_restore": restore_report,
                "final_validation": final_result["final_validation"],
                "final_robots": final_result["final_robots"],
            }
    except Exception as error:
        update_episode_replay_result(
            episode_path,
            success=False,
            failure_reason=f"{type(error).__name__}: {error}",
        )
        raise

    update_episode_replay_result(
        episode_path,
        success=True,
        summary=replay_summary,
    )
    schema_after = validate_episode_hdf5(
        episode_path,
        require_accepted=True,
    )
    return {
        "success": True,
        "episode_path": str(episode_path),
        "compatibility": compatibility,
        "camera_validation": camera_validation,
        "replay": replay_summary,
        "schema": schema_after,
    }


def validate_episode_worker_completion(
    *,
    mode: str,
    return_code: int,
    success_marker_seen: bool,
    log_path: Path,
    tail: Sequence[str],
) -> str:
    """Require both a clean exit and the mode-specific logical success marker."""

    try:
        expected_marker = EPISODE_WORKER_SUCCESS_MARKERS[mode]
    except KeyError as error:
        raise ValueError(f"Unsupported episode worker mode {mode!r}") from error
    if return_code != 0:
        raise RuntimeError(
            f"{mode} failed with exit code {return_code}; "
            f"required_marker={expected_marker}; log={log_path}; "
            f"tail={list(tail[-20:])}"
        )
    if not success_marker_seen:
        error_lines = [
            line
            for line in tail
            if any(
                token in line
                for token in ("Traceback", "RuntimeError", "Error", "ERROR")
            )
        ]
        raise RuntimeError(
            f"{mode} exited with code 0 without required success marker "
            f"{expected_marker}; log={log_path}; "
            f"error_tail={error_lines[-5:]}; tail={list(tail[-20:])}"
        )
    return expected_marker


def _run_episode_worker_process(
    args: argparse.Namespace,
    *,
    mode: str,
    episode_path: Path,
    seed: int,
    log_path: Path,
) -> dict[str, Any]:
    """Run one Isaac worker in a clean process and stream concise progress."""

    try:
        expected_success_marker = EPISODE_WORKER_SUCCESS_MARKERS[mode]
    except KeyError as error:
        raise ValueError(f"Unsupported episode worker mode {mode!r}") from error
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--mode",
        mode,
        "--episode",
        str(episode_path.resolve()),
        "--seed",
        str(seed),
        "--planner-seed",
        str(args.planner_seed),
        "--output-dir",
        str(args.output_dir.resolve()),
    ]
    if args.headless:
        command.append("--headless")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    if process.stdout is None:
        process.terminate()
        raise RuntimeError(f"{mode}: worker stdout is unavailable")
    start_time = time.monotonic()
    tail: list[str] = []
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    progress_prefixes = (
        "CUROBO_SORT_PROGRESS",
        "HDF5_REPLAY_PROGRESS",
        "HDF5_EXPERT_RECORDING_OK",
        "HDF5_ACTION_REPLAY_OK",
    )
    timed_out = False
    success_marker_seen = False
    with log_path.open("w", encoding="utf-8") as log_file:
        try:
            while True:
                if time.monotonic() - start_time > EPISODE_MAX_RUNTIME_S:
                    timed_out = True
                    process.terminate()
                    break
                events = selector.select(1.0)
                if events:
                    line = process.stdout.readline()
                    if line:
                        log_file.write(line)
                        log_file.flush()
                        stripped = line.rstrip()
                        tail.append(stripped)
                        del tail[:-80]
                        if stripped == expected_success_marker:
                            success_marker_seen = True
                        if stripped.startswith(progress_prefixes):
                            print(stripped, flush=True)
                    elif process.poll() is not None:
                        break
                elif process.poll() is not None:
                    remainder = process.stdout.read()
                    if remainder:
                        log_file.write(remainder)
                        for line in remainder.splitlines():
                            tail.append(line)
                            del tail[:-80]
                            if line.rstrip() == expected_success_marker:
                                success_marker_seen = True
                    break
        finally:
            selector.close()
    if timed_out:
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)
        raise TimeoutError(
            f"{mode} exceeded {EPISODE_MAX_RUNTIME_S:.1f} s; "
            f"log={log_path}"
        )
    return_code = process.wait()
    validated_marker = validate_episode_worker_completion(
        mode=mode,
        return_code=return_code,
        success_marker_seen=success_marker_seen,
        log_path=log_path,
        tail=tail,
    )
    return {
        "mode": mode,
        "return_code": return_code,
        "success_marker": validated_marker,
        "duration_s": time.monotonic() - start_time,
        "log_path": str(log_path.resolve()),
    }


def _accepted_episode_path(candidate_path: Path) -> Path:
    suffix = ".partial.h5"
    if candidate_path.name.endswith(suffix):
        return candidate_path.with_name(
            candidate_path.name.removesuffix(suffix) + ".h5"
        )
    return candidate_path


def run_public_replay(args: argparse.Namespace) -> dict[str, Any]:
    if args.episode is None:
        raise ValueError("--mode replay requires --episode")
    episode_path = args.episode.resolve()
    log_path = (
        args.output_dir
        / "diagnostics"
        / f"replay_{episode_path.stem}.log"
    )
    worker = _run_episode_worker_process(
        args,
        mode="replay-worker",
        episode_path=episode_path,
        seed=args.seed,
        log_path=log_path,
    )
    accepted_path = _accepted_episode_path(episode_path)
    if accepted_path != episode_path:
        if accepted_path.exists():
            raise FileExistsError(
                f"Accepted episode path already exists: {accepted_path}"
            )
        episode_path.replace(accepted_path)
    schema = validate_episode_hdf5(
        accepted_path,
        require_accepted=True,
    )
    return {
        "success": True,
        "worker": worker,
        "episode_path": str(accepted_path),
        "schema": schema,
    }


def run_public_collection(args: argparse.Namespace) -> dict[str, Any]:
    """Collect exactly N accepted episodes, not merely N attempts."""

    requested = 1 if args.mode == "demo" else int(args.episodes)
    if requested <= 0:
        raise ValueError("--episodes must be positive")
    if args.max_attempts < requested:
        raise ValueError("--max-attempts must be at least --episodes")
    episode_directory = args.output_dir.resolve() / "episodes"
    diagnostic_directory = args.output_dir.resolve() / "diagnostics"
    episode_directory.mkdir(parents=True, exist_ok=True)
    diagnostic_directory.mkdir(parents=True, exist_ok=True)
    accepted: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []
    for attempt_index in range(args.max_attempts):
        if len(accepted) >= requested:
            break
        attempt_number = attempt_index + 1
        attempt_seed = int(args.seed) + attempt_index
        timestamp = datetime.datetime.now(
            datetime.timezone.utc
        ).strftime("%Y%m%dT%H%M%SZ")
        stem = (
            f"episode_{attempt_seed}_{timestamp}_"
            f"p{os.getpid()}_a{attempt_number:02d}"
        )
        candidate_path = episode_directory / f"{stem}.partial.h5"
        collect_log = diagnostic_directory / f"{stem}_collect.log"
        replay_log = diagnostic_directory / f"{stem}_replay.log"
        print(
            "HDF5_COLLECTION_ATTEMPT "
            f"{attempt_number}/{args.max_attempts} seed={attempt_seed}",
            flush=True,
        )
        try:
            collect_worker = _run_episode_worker_process(
                args,
                mode="collect-worker",
                episode_path=candidate_path,
                seed=attempt_seed,
                log_path=collect_log,
            )
            replay_worker = _run_episode_worker_process(
                args,
                mode="replay-worker",
                episode_path=candidate_path,
                seed=attempt_seed,
                log_path=replay_log,
            )
            accepted_path = _accepted_episode_path(candidate_path)
            if accepted_path.exists():
                raise FileExistsError(
                    f"Accepted episode path exists: {accepted_path}"
                )
            candidate_path.replace(accepted_path)
            schema = validate_episode_hdf5(
                accepted_path,
                require_accepted=True,
            )
        except Exception as error:
            failures.append(
                {
                    "attempt": attempt_number,
                    "seed": attempt_seed,
                    "error": f"{type(error).__name__}: {error}",
                    "candidate_path": str(candidate_path),
                    "collect_log": str(collect_log),
                    "replay_log": str(replay_log),
                }
            )
            print(
                "HDF5_COLLECTION_REJECTED "
                f"attempt={attempt_number} seed={attempt_seed} "
                f"error={type(error).__name__}: {error}",
                flush=True,
            )
            continue
        accepted.append(
            {
                "path": str(accepted_path),
                "seed": attempt_seed,
                "schema": schema,
                "collect_worker": collect_worker,
                "replay_worker": replay_worker,
            }
        )
        print(
            "HDF5_COLLECTION_ACCEPTED "
            f"{len(accepted)}/{requested} path={accepted_path}",
            flush=True,
        )
    if len(accepted) != requested:
        raise RuntimeError(
            f"Collected {len(accepted)}/{requested} accepted episodes after "
            f"{args.max_attempts} attempts; failures={failures}"
        )
    return {
        "success": True,
        "requested_accepted_episodes": requested,
        "attempt_count": len(accepted) + len(failures),
        "accepted": accepted,
        "failures": failures,
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=(
            "audit",
            "scene",
            "robots",
            "cameras",
            "dolls",
            "motion",
            "pick",
            "sort",
            "planner-worker",
            "collect-worker",
            "replay-worker",
            "demo",
            "collect",
            "replay",
            "validate",
        ),
        default="demo",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--planner-seed", type=int, default=1)
    parser.add_argument("--episode", type=Path)
    parser.add_argument(
        "--output-dir", type=Path, default=REPOSITORY_ROOT / "dual_piper_output"
    )
    parser.add_argument("--max-attempts", type=int, default=10)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.mode == "planner-worker":
        return run_curobo_planner_worker(args.seed)
    if args.mode == "validate":
        if args.episode is None:
            raise ValueError("--mode validate requires --episode")
        report = validate_episode_hdf5(args.episode)
        print("HDF5_EPISODE_VALID")
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    if args.mode == "replay":
        report = run_public_replay(args)
        print("HDF5_ACTION_REPLAY_ACCEPTED")
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    if args.mode == "demo" and args.episode is not None:
        report = run_public_replay(args)
        print("DUAL_PIPER_DEMO_ACCEPTED")
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    if args.mode in {"demo", "collect"}:
        report = run_public_collection(args)
        marker = (
            "DUAL_PIPER_DEMO_ACCEPTED"
            if args.mode == "demo"
            else "HDF5_COLLECTION_OK"
        )
        print(marker)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    if args.mode in {
        "audit",
        "scene",
        "robots",
        "cameras",
        "dolls",
        "motion",
        "pick",
        "sort",
        "collect-worker",
        "replay-worker",
    }:
        # SimulationApp.close() shuts down the Python process in Isaac Sim 5.1,
        # so all useful output must be emitted and flushed before that call.
        validate_static_assets()
        from isaacsim import SimulationApp  # type: ignore[import-not-found]

        simulation_app = SimulationApp(
            {"headless": args.headless, "width": 1280, "height": 720}
        )
        if args.mode == "audit":
            report = run_asset_audit()
            marker = "ASSET_AUDIT_OK"
        elif args.mode == "scene":
            world = create_scene()
            report = validate_scene(world)
            preview_name = (
                "scene_headless.png" if args.headless else "scene_headed.png"
            )
            report["preview"] = capture_scene_preview(
                world, args.output_dir / "scene" / preview_name
            )
            marker = "SCENE_SMOKE_OK"
        elif args.mode == "robots":
            world = create_scene()
            robots = create_robots(world)
            report = {
                "scene": validate_scene(world),
                "robots": exercise_and_validate_robots(world, robots),
            }
            preview_name = (
                "robots_headless.png" if args.headless else "robots_headed.png"
            )
            report["preview"] = capture_scene_preview(
                world, args.output_dir / "robots" / preview_name
            )
            marker = "ROBOT_SMOKE_OK"
        elif args.mode == "motion":
            world = create_scene()
            robots = create_robots(world)
            report = {
                "scene": validate_scene(world),
                "motion": run_curobo_motion_smoke(
                    world,
                    robots,
                    seed=args.planner_seed,
                    render=not args.headless,
                ),
            }
            preview_name = (
                "motion_headless.png"
                if args.headless
                else "motion_headed.png"
            )
            report["preview"] = capture_scene_preview(
                world, args.output_dir / "motion" / preview_name
            )
            marker = "CUROBO_MOTION_SMOKE_OK"
        elif args.mode == "pick":
            seeds = set_episode_random_seeds(args.seed)
            layout = sample_initial_doll_layout(args.seed)
            world = create_scene()
            robots = create_robots(world)
            dolls = create_dolls(world, layout)
            initial_dolls = settle_and_validate_dolls(world, dolls)
            report = {
                "seeds": {
                    **seeds,
                    "curobo_motion_planner": args.planner_seed,
                },
                "sampled_layout": [
                    asdict(placement) for placement in layout
                ],
                "initial_dolls": initial_dolls,
                "scene": validate_scene(world),
                "pick_place": run_curobo_pick_place_smoke(
                    world,
                    robots,
                    dolls,
                    planner_seed=args.planner_seed,
                    render=not args.headless,
                ),
            }
            preview_name = (
                "pick_headless.png" if args.headless else "pick_headed.png"
            )
            report["preview"] = capture_scene_preview(
                world, args.output_dir / "pick" / preview_name
            )
            marker = "CUROBO_PICK_PLACE_SMOKE_OK"
        elif args.mode == "sort":
            seeds = set_episode_random_seeds(args.seed)
            layout = sample_initial_doll_layout(args.seed)
            world = create_scene()
            robots = create_robots(world)
            dolls = create_dolls(world, layout)
            initial_dolls = settle_and_validate_dolls(world, dolls)
            report = {
                "seeds": {
                    **seeds,
                    "curobo_motion_planner": args.planner_seed,
                },
                "sampled_layout": [
                    asdict(placement) for placement in layout
                ],
                "initial_dolls": initial_dolls,
                "scene": validate_scene(world),
                "sort": run_full_curobo_sort(
                    world,
                    robots,
                    dolls,
                    planner_seed=args.planner_seed,
                    render=not args.headless,
                ),
            }
            preview_name = (
                "sort_headless.png" if args.headless else "sort_headed.png"
            )
            report["preview"] = capture_scene_preview(
                world, args.output_dir / "sort" / preview_name
            )
            marker = "CUROBO_FULL_SORT_OK"
        elif args.mode == "collect-worker":
            report = run_hdf5_collection_worker(args)
            marker = "HDF5_EXPERT_RECORDING_OK"
        elif args.mode == "replay-worker":
            report = run_hdf5_replay_worker(args)
            marker = "HDF5_ACTION_REPLAY_OK"
        elif args.mode == "cameras":
            world = create_scene()
            robots = create_robots(world)
            cameras = create_cameras(world)
            preview_directory = args.output_dir / "cameras" / (
                "headless" if args.headless else "headed"
            )
            report = {
                "scene": validate_scene(world),
                "robots_at_home": validate_robots_at_home(
                    robots, label="camera_smoke"
                ),
                "cameras": validate_and_capture_cameras(
                    world,
                    cameras,
                    output_dir=preview_directory,
                ),
            }
            marker = "CAMERA_SMOKE_OK"
        else:
            seeds = set_episode_random_seeds(args.seed)
            layout = sample_initial_doll_layout(args.seed)
            world = create_scene()
            robots = create_robots(world)
            dolls = create_dolls(world, layout)
            doll_report = settle_and_validate_dolls(world, dolls)
            cameras = create_cameras(world)
            preview_directory = args.output_dir / "dolls" / (
                "headless" if args.headless else "headed"
            )
            report = {
                "seeds": seeds,
                "sampled_layout": [asdict(placement) for placement in layout],
                "layout_validation": validate_initial_doll_layout(layout),
                "scene": validate_scene(world),
                "robots_at_home": validate_robots_at_home(
                    robots, label="doll_smoke"
                ),
                "dolls": doll_report,
                "cameras": validate_and_capture_cameras(
                    world,
                    cameras,
                    output_dir=preview_directory,
                ),
                "overview": capture_scene_preview(
                    world, preview_directory / "overview.png"
                ),
            }
            marker = "DOLL_SMOKE_OK"
        print(marker)
        print(json.dumps(report, indent=2, sort_keys=True, default=str))
        sys.stdout.flush()
        simulation_app.close()
        return 0  # Kept for type checkers; close() terminates this process.
    raise RuntimeError(f"Unhandled mode {args.mode!r}")


if __name__ == "__main__":
    raise SystemExit(main())
