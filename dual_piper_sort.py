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
PIPER_HOME_TOLERANCE_RAD: Final = 0.02

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


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("audit", "scene", "cameras", "demo", "collect", "replay", "validate"),
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
    if args.mode == "audit":
        # SimulationApp.close() shuts down the Python process in Isaac Sim 5.1,
        # so all useful output must be emitted and flushed before that call.
        validate_static_assets()
        from isaacsim import SimulationApp  # type: ignore[import-not-found]

        simulation_app = SimulationApp({"headless": args.headless})
        report = run_asset_audit()
        print("ASSET_AUDIT_OK")
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
