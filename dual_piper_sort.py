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
import json
import math
import re
import sys
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
# The source asset is a 0.613 m boom along local -Y.  This initial pose puts
# its rendered camera housing over the table centre.  It is intentionally a
# top-level constant and is subject to the required headed visual check.
CAMERA_STAND_POSITION: Final = (0.0, 0.50, 1.55)
CAMERA_STAND_ORIENTATION: Final = IDENTITY_QUATERNION
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
PIPER_TOOL_LINK: Final = "gripper_center"
PIPER_WRIST_LINK: Final = "link6"
PIPER_CAMERA_MOUNT_REL_PATH: Final = "link6/camera"
PIPER_TOOL_REL_PATH: Final = "link6/gripper_center"
# This valid, folded configuration is a candidate home pose.  Its workspace
# direction and collision clearance are verified in the dual-robot milestone.
PIPER_HOME_JOINT_POSITION: Final = (0.0, 1.57, -1.57, 0.0, 0.0, 0.0)
PIPER_OPEN_GRIPPER_POSITION: Final = 0.04
PIPER_CLOSED_GRIPPER_POSITION: Final = 0.0
PIPER_HOME_DOF_POSITION: Final = PIPER_HOME_JOINT_POSITION + (
    PIPER_OPEN_GRIPPER_POSITION,
    PIPER_OPEN_GRIPPER_POSITION,
)
PIPER_HOME_TOLERANCE_RAD: Final = 0.02
PIPER_GRIPPER_TOLERANCE_M: Final = 0.001
PIPER_OPEN_FINGER_SEPARATION_M: Final = 0.08
PIPER_WORKSPACE_FORWARD_MINIMUM_M: Final = 0.40
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
# This lies inside the rendered D455 housing near the centre of its front mask.
OVERHEAD_CAMERA_POSITION: Final = (0.0225, -0.041, 1.600)
OVERHEAD_CAMERA_TARGET: Final = (0.0, -0.05, TABLE_TOP_Z)
OVERHEAD_CAMERA_IMAGE_UP: Final = (0.0, 1.0, 0.0)
CAMERA_RENDER_WARMUP_STEPS: Final = 24
CAMERA_TARGET_PIXEL_MARGIN: Final = 2.0
SCENE_PREVIEW_EYE: Final = (2.1, -2.3, 1.9)
SCENE_PREVIEW_TARGET: Final = (0.0, -0.05, 0.75)

MATRYOSHKA_UUID: Final = "a5a44251-6d8c-4cee-a8d7-90c443e47e53"
MATRYOSHKA_SORT_ORDER: Final = ("00004", "00003", "00002", "00001", "00000")
MATRYOSHKA_TARGET_GAP: Final = 0.025
MATRYOSHKA_INITIAL_GAP: Final = 0.04
MATRYOSHKA_POSITION_TOLERANCE: Final = 0.02
MATRYOSHKA_UPRIGHT_TOLERANCE_DEGREES: Final = 10.0
MATRYOSHKA_LINEAR_SPEED_TOLERANCE: Final = 0.01
MATRYOSHKA_ANGULAR_SPEED_TOLERANCE: Final = 0.10
MATRYOSHKA_PRIM_ROOT: Final = "/World/Objects"
MATRYOSHKA_PHYSICS_MATERIAL_PATH: Final = "/World/Looks/MatryoshkaPhysics"
MATRYOSHKA_PHYSICS_RESTITUTION: Final = 0.05
# This central area remains inside the overhead view and leaves generous room
# for grasp approach, the table edges, and the rear robot bases.
MATRYOSHKA_RANDOM_X_RANGE: Final = (-0.40, 0.40)
MATRYOSHKA_RANDOM_Y_RANGE: Final = (-0.22, 0.28)
MATRYOSHKA_TABLE_EDGE_CLEARANCE: Final = 0.08
MATRYOSHKA_ROBOT_BASE_EXCLUSION_RADIUS: Final = 0.16
MATRYOSHKA_LAYOUT_SAMPLES_PER_OBJECT: Final = 500
MATRYOSHKA_SPAWN_CLEARANCE: Final = 0.003
MATRYOSHKA_TABLE_HEIGHT_TOLERANCE: Final = 0.008
MATRYOSHKA_SETTLE_MAX_STEPS: Final = 1_440
MATRYOSHKA_STABLE_CONSECUTIVE_STEPS: Final = 30


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
    _assert_vector_close(
        stand_bbox["min"],
        tuple(
            CAMERA_STAND_POSITION[index] + CAMERA_STAND_SOURCE_BBOX_MIN[index]
            for index in range(3)
        ),
        label="camera stand bbox min",
        tolerance=2.0e-4,
    )
    _assert_vector_close(
        stand_bbox["max"],
        tuple(
            CAMERA_STAND_POSITION[index] + CAMERA_STAND_SOURCE_BBOX_MAX[index]
            for index in range(3)
        ),
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
        if forward_distance < PIPER_WORKSPACE_FORWARD_MINIMUM_M:
            raise ValueError(
                f"{spec.name}: gripper center is only {forward_distance:.4f} m "
                "in world +Y from its base"
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
    from pxr import Usd, UsdPhysics, UsdShade  # type: ignore[import-not-found]

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

    from pxr import Usd, UsdPhysics, UsdShade  # type: ignore[import-not-found]

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
            "collision_prim_paths": [
                str(collision_prim.GetPath()) for collision_prim in collision_prims
            ],
            "physics_material_path": MATRYOSHKA_PHYSICS_MATERIAL_PATH,
        }
    return report


def settle_and_validate_dolls(
    world: Any,
    dolls: dict[str, Any],
) -> dict[str, Any]:
    """Advance finite physics steps until all five dolls are stably upright."""

    initial_states = _doll_state_report(dolls)
    consecutive = 0
    final_states: dict[str, dict[str, Any]] | None = None
    settle_steps = 0
    for settle_steps in range(1, MATRYOSHKA_SETTLE_MAX_STEPS + 1):
        world.step(render=False)
        states = _doll_state_report(dolls)
        try:
            _validate_stable_doll_states(states)
        except ValueError:
            consecutive = 0
        else:
            consecutive += 1
            if consecutive >= MATRYOSHKA_STABLE_CONSECUTIVE_STEPS:
                final_states = states
                break
    if final_states is None:
        raise RuntimeError(
            "Dolls did not satisfy the stable pose/velocity limits after "
            f"{MATRYOSHKA_SETTLE_MAX_STEPS} physics steps"
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


def _look_at_world_quaternion(
    position: Sequence[float],
    target: Sequence[float],
    image_up: Sequence[float],
) -> Any:
    """Build a wxyz orientation for Isaac's +X-forward world camera axes."""

    import numpy as np
    from isaacsim.core.utils.numpy.rotations import (  # type: ignore[import-not-found]
        rot_matrices_to_quats,
    )

    forward = np.asarray(target, dtype=np.float64) - np.asarray(
        position, dtype=np.float64
    )
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm <= 1.0e-9:
        raise ValueError("Camera position and look-at target must differ")
    forward /= forward_norm
    up = np.asarray(image_up, dtype=np.float64)
    up -= float(np.dot(up, forward)) * forward
    up_norm = float(np.linalg.norm(up))
    if up_norm <= 1.0e-9:
        raise ValueError("Camera image-up direction is parallel to its optical axis")
    up /= up_norm
    left = np.cross(up, forward)
    rotation = np.column_stack((forward, left, up))
    return rot_matrices_to_quats(rotation).astype(np.float32)


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
        orientation=_look_at_world_quaternion(
            OVERHEAD_CAMERA_POSITION,
            OVERHEAD_CAMERA_TARGET,
            OVERHEAD_CAMERA_IMAGE_UP,
        ),
        camera_axes="world",
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
            workspace_points = np.asarray(
                [
                    (x, y, TABLE_TOP_Z)
                    for x in MATRYOSHKA_RANDOM_X_RANGE
                    for y in MATRYOSHKA_RANDOM_Y_RANGE
                ]
                + [
                    placement.pose.position
                    for placement in compute_doll_target_layout()
                ],
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
                    "Overhead camera does not cover the complete random and "
                    f"target regions: {projected_workspace.tolist()}"
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
            },
            "coverage": {
                "target_world_m": target.tolist(),
                "target_pixel_uv": target_pixel.tolist(),
                "optical_axis_target_cosine": target_alignment,
                "target_distance_m": target_distance,
                "random_corners_and_targets_pixels_uv": workspace_pixels,
            },
            "rendering_time_s": observations[name]["rendering_time_s"],
            **preview,
        }
    return report


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
    parser.add_argument("--episode", type=Path)
    parser.add_argument(
        "--output-dir", type=Path, default=REPOSITORY_ROOT / "dual_piper_output"
    )
    parser.add_argument("--max-attempts", type=int, default=10)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.mode in {"audit", "scene", "robots", "cameras", "dolls"}:
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
    print(
        f"Mode {args.mode!r} is not implemented at the asset-audit milestone.",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
