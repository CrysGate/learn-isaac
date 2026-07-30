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
OVERHEAD_CAMERA_TARGET: Final = (0.0, -0.05, TABLE_TOP_Z)
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


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=(
            "audit",
            "scene",
            "robots",
            "cameras",
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
    if args.mode in {"audit", "scene", "robots"}:
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
        else:
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
