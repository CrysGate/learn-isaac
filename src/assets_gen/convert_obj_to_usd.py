from __future__ import annotations

import argparse
import asyncio
import math
import os
import posixpath
import re
import shutil
import sys
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple
from xml.etree import ElementTree as ET

# These modules only become available after SimulationApp starts. Keeping the
# startup out of module import also prevents Kit from consuming this script's
# argparse options before argparse sees them.
simulation_app: Any = None
asset_converter: Any = None
carb: Any = None
omni_usd: Any = None
PhysxSchema: Any = None
pymeshlab: Any = None
Gf: Any = None
Sdf: Any = None
Usd: Any = None
UsdGeom: Any = None
UsdPhysics: Any = None
UsdShade: Any = None


AXES = ("x", "y", "z")
# CLI defaults and output names.
DEFAULT_ASSETS_ROOT = "assets"
DEFAULT_TEST_FOLDER = "vase"
DEFAULT_METADATA_FILENAME = "sample.xlsx"
DEFAULT_OUTPUT_DIRECTORY = "rigid assets"
DEFAULT_USD_FILENAME = "Aligned.usd"
DEFAULT_OBJ_FILENAME = "Aligned.obj"
IGNORED_PRIM_NAMES = {"Looks", "PhysicsMaterial", "_materials", "visual", "collision", "root"}
STAGE_OPEN_MAX_UPDATES = 240
STAGE_CLOSE_MAX_UPDATES = 60
COLLISION_APPROXIMATION = "convexDecomposition"
COLLISION_MAX_CONVEX_HULLS = 64
COLLISION_HULL_VERTEX_LIMIT = 64
COLLISION_MIN_THICKNESS = 0.001
COLLISION_SHRINK_WRAP = True
COLLISION_ERROR_PERCENTAGE = 0.1
PHYSICS_FRICTION = 1.0
DEFAULT_MASS_KG = 0.1
DEFAULT_SCALE = 1.0
DEFAULT_TARGET_FACES = 1000
MASS_GRAMS_PER_KILOGRAM = 1000.0
POST_STAGE_UPDATE_COUNT = 3
# USD topology and authored custom attributes.
ROOT_PATH = "/root"
MATERIALS_PATH = f"{ROOT_PATH}/_materials"
VISUAL_PATH = f"{ROOT_PATH}/visual"
COLLISION_PATH = f"{ROOT_PATH}/collision"
MODEL_NAME = "model"
PHYSICS_MATERIAL_NAME = "PhysicsMaterial"
CUSTOM_SCALE_PREFIX = "scale"
CUSTOM_DIMENSION_PREFIX = "real"
# Metadata parsing and physics defaults.
SUPPORTED_MASS_UNITS = frozenset({"auto", "g", "kg"})
DIMENSION_UNIT_FACTORS = {"m": 1.0, "cm": 0.01, "mm": 0.001}
NAME_COLUMN_HEADERS = ("name", "名称", "物体", "object")
MASS_COLUMN_HEADERS = ("mass(kg)", "mass(g)", "mass", "weight", "重量", "质量")

_XLSX_MAIN_NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
_XLSX_REL_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
_XLSX_NAMESPACES = {"m": _XLSX_MAIN_NS, "r": _XLSX_REL_NS}

MERGE_MESHES = True


def initialize_isaac_runtime() -> None:
    global simulation_app, asset_converter, carb, omni_usd, PhysxSchema, pymeshlab
    global Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    if simulation_app is not None:
        return

    from isaacsim import SimulationApp

    script_argv = sys.argv
    sys.argv = [script_argv[0]]
    try:
        simulation_app = SimulationApp({"headless": True})
        import carb as carb_module
        import omni.kit.asset_converter as asset_converter_module
        import omni.usd as omni_usd_module
        import pymeshlab as pymeshlab_module
        from pxr import (
            Gf as gf_module,
            PhysxSchema as physx_schema_module,
            Sdf as sdf_module,
            Usd as usd_module,
            UsdGeom as usd_geom_module,
            UsdPhysics as usd_physics_module,
            UsdShade as usd_shade_module,
        )
    except BaseException:
        if simulation_app is not None:
            simulation_app.close()
        simulation_app = None
        raise
    finally:
        sys.argv = script_argv

    carb = carb_module
    asset_converter = asset_converter_module
    omni_usd = omni_usd_module
    PhysxSchema = physx_schema_module
    pymeshlab = pymeshlab_module
    Gf = gf_module
    Sdf = sdf_module
    Usd = usd_module
    UsdGeom = usd_geom_module
    UsdPhysics = usd_physics_module
    UsdShade = usd_shade_module


def require_isaac_runtime() -> None:
    if any(
        module is None
        for module in (
            simulation_app,
            asset_converter,
            carb,
            omni_usd,
            PhysxSchema,
            pymeshlab,
            Gf,
            Sdf,
            Usd,
            UsdGeom,
            UsdPhysics,
            UsdShade,
        )
    ):
        raise RuntimeError("Isaac Sim runtime has not been initialized")


@dataclass(frozen=True)
class AssetMetadata:
    name: str
    mass_kg: Optional[float]
    dimensions_cm: Dict[str, float]
    row_number: int


@dataclass(frozen=True)
class AabbInfo:
    aabb: Tuple[float, float, float, float, float, float]
    dimensions: Dict[str, float]
    prim_path: str


@dataclass
class ZipExtractionStats:
    total: int = 0
    extracted: int = 0
    lfs_pointers: int = 0
    failed: int = 0

    def add(self, other: "ZipExtractionStats") -> None:
        self.total += other.total
        self.extracted += other.extracted
        self.lfs_pointers += other.lfs_pointers
        self.failed += other.failed


@dataclass(frozen=True)
class ConversionConfig:
    """Validated settings used by the asset conversion pipeline."""

    assets_root: Path
    folders: tuple[Path, ...]
    metadata_xlsx: Path
    output_root: Path
    fallback_mass_kg: float
    fallback_scale: float
    metadata_mass_unit: str
    dimension_unit: str
    max_models: int
    force: bool
    extract_zips: bool
    target_faces: int


@dataclass(frozen=True)
class ConvertedAsset:
    metadata: Optional[AssetMetadata]
    source_aabb: AabbInfo
    scale_values: Dict[str, float]
    scale_source: str
    real_dimensions: Dict[str, float]


async def _run_asset_converter(
    input_path: str,
    output_path: str,
    context: Any,
) -> bool:
    """Run one AssetConverter task and report its error details."""
    require_isaac_runtime()

    task = asset_converter.get_instance().create_converter_task(
        input_path,
        output_path,
        None,
        context,
    )
    success = await task.wait_until_finished()
    if not success:
        carb.log_error(
            "Asset Converter task failed: "
            f"{task.get_status()} - {task.get_detailed_error()}"
        )
    return success


async def convert_asset_to_usd(input_obj: str, output_usd: str) -> bool:
    """Convert an OBJ file to USD using Isaac Sim's AssetConverter."""
    require_isaac_runtime()
    context = asset_converter.AssetConverterContext()
    context.ignore_materials = False
    context.ignore_animations = True
    context.ignore_camera = True
    context.ignore_light = True
    context.use_meter_as_world_unit = True
    context.merge_all_meshes = MERGE_MESHES
    context.use_double_precision_to_usd_transform_op = True
    return await _run_asset_converter(input_obj, output_usd, context)


async def convert_usd_to_obj(input_usd: str, output_obj: str) -> bool:
    """Convert a USD file back to OBJ using Isaac Sim's AssetConverter."""
    require_isaac_runtime()
    return await _run_asset_converter(
        input_usd,
        output_obj,
        asset_converter.AssetConverterContext(),
    )


def _walk_error(error: OSError) -> None:
    print(f"Warning: cannot access {error.filename}: {error.strerror}")


def _safe_extract_zip(zip_path: Path, output_dir: Path) -> None:
    output_dir = output_dir.resolve()
    with zipfile.ZipFile(zip_path, "r") as zf:
        for member in zf.infolist():
            member_path = (output_dir / member.filename).resolve()
            if member_path != output_dir and output_dir not in member_path.parents:
                raise ValueError(f"Unsafe zip member path in {zip_path}: {member.filename}")
        zf.extractall(output_dir)


def _is_git_lfs_pointer(file_path: Path) -> bool:
    try:
        header = file_path.read_bytes()[:128]
    except OSError:
        return False
    return header.startswith(b"version https://git-lfs.github.com/spec/v1")


def extract_zips_in_tree(directory: Path) -> ZipExtractionStats:
    stats = ZipExtractionStats()
    zip_paths: List[Path] = []
    for root, _, files in os.walk(directory, onerror=_walk_error):
        for file_name in files:
            if file_name.lower().endswith(".zip"):
                zip_paths.append(Path(root) / file_name)

    for zip_path in sorted(zip_paths):
        stats.total += 1
        if _is_git_lfs_pointer(zip_path):
            stats.lfs_pointers += 1
            print(f"Skipping Git LFS pointer file, not a real zip: {zip_path}")
            continue
        try:
            print(f"Extracting zip: {zip_path}")
            _safe_extract_zip(zip_path, zip_path.parent)
            stats.extracted += 1
        except Exception as exc:
            stats.failed += 1
            print(f"Warning: failed to extract {zip_path}: {exc}")
    return stats


def iter_obj_files(folders: Sequence[Path]) -> Iterable[Path]:
    seen = set()
    for folder in folders:
        for root, _, files in os.walk(folder, onerror=_walk_error):
            for file_name in files:
                if not file_name.lower().endswith(".obj"):
                    continue
                obj_path = Path(root) / file_name
                real_path = obj_path.resolve()
                if real_path in seen:
                    continue
                seen.add(real_path)
                yield real_path


def _xlsx_target_to_path(target: str) -> str:
    target = target.lstrip("/")
    return posixpath.normpath(target if target.startswith("xl/") else posixpath.join("xl", target))


def _cell_column_index(cell_ref: str) -> int:
    match = re.match(r"([A-Z]+)", cell_ref)
    if not match:
        return 0

    index = 0
    for char in match.group(1):
        index = index * 26 + ord(char) - ord("A") + 1
    return index - 1


def _parse_xlsx_rows(xlsx_path: Path) -> List[List[object]]:
    """Read the first worksheet from an XLSX file without a heavyweight dependency."""
    with zipfile.ZipFile(xlsx_path, "r") as zf:
        shared_strings = _read_shared_strings(zf)

        workbook = ET.fromstring(zf.read("xl/workbook.xml"))
        relationships = ET.fromstring(zf.read("xl/_rels/workbook.xml.rels"))
        relationship_map = {
            relationship.attrib["Id"]: relationship.attrib["Target"]
            for relationship in relationships
            if relationship.attrib.get("Id") and relationship.attrib.get("Target")
        }
        sheets = workbook.find("m:sheets", _XLSX_NAMESPACES)
        if sheets is None:
            return []

        first_sheet = next(iter(sheets), None)
        if first_sheet is None:
            return []
        relationship_id = first_sheet.attrib.get(f"{{{_XLSX_REL_NS}}}id")
        if relationship_id not in relationship_map:
            raise ValueError(f"First worksheet relationship is missing in {xlsx_path}")
        sheet_path = _xlsx_target_to_path(relationship_map[relationship_id])
        sheet = ET.fromstring(zf.read(sheet_path))

        rows: List[List[object]] = []
        for row in sheet.findall(".//m:sheetData/m:row", _XLSX_NAMESPACES):
            values_by_col = {
                _cell_column_index(cell.attrib.get("r", "")): _xlsx_cell_value(
                    cell, shared_strings
                )
                for cell in row.findall("m:c", _XLSX_NAMESPACES)
            }
            if values_by_col:
                rows.append([
                    values_by_col.get(index, "") for index in range(max(values_by_col) + 1)
                ])
            else:
                rows.append([])
        return rows


def _read_shared_strings(xlsx: zipfile.ZipFile) -> List[str]:
    try:
        shared_strings_xml = xlsx.read("xl/sharedStrings.xml")
    except KeyError:
        return []

    root = ET.fromstring(shared_strings_xml)
    return [
        "".join(text_node.text or "" for text_node in string_item.iter(f"{{{_XLSX_MAIN_NS}}}t"))
        for string_item in root.findall("m:si", _XLSX_NAMESPACES)
    ]


def _xlsx_cell_value(cell: ET.Element, shared_strings: Sequence[str]) -> object:
    cell_type = cell.attrib.get("t")
    value_node = cell.find("m:v", _XLSX_NAMESPACES)
    if cell_type == "s" and value_node is not None:
        index = int(value_node.text or 0)
        if not 0 <= index < len(shared_strings):
            raise ValueError(f"Shared string index out of range: {index}")
        return shared_strings[index]

    if cell_type == "inlineStr":
        inline_string = cell.find("m:is", _XLSX_NAMESPACES)
        if inline_string is not None:
            return "".join(
                text_node.text or ""
                for text_node in inline_string.iter(f"{{{_XLSX_MAIN_NS}}}t")
            )

    return value_node.text if value_node is not None else ""


def _normalize_header(value: object) -> str:
    return re.sub(r"\s+", "", str(value or "").lower())


def _parse_number(value: object) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            return None
        return float(value)

    text = str(value).strip()
    if not text:
        return None

    match = re.search(
        r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?",
        text.replace(",", ""),
    )
    if not match:
        return None
    return float(match.group(0))


def _find_column(headers: Sequence[str], candidates: Sequence[str]) -> Optional[int]:
    candidate_set = set(candidates)
    for index, header in enumerate(headers):
        if header in candidate_set:
            return index
    return None


def _row_value(row: Sequence[object], index: Optional[int]) -> object:
    if index is None or index >= len(row):
        return ""
    return row[index]


def _mass_unit_from_header(header: str, fallback_unit: str) -> str:
    if header == "kg" or header.endswith("(kg)") or header.endswith("_kg"):
        return "kg"
    if header == "g" or header.endswith("(g)") or header.endswith("_g"):
        return "g"
    if fallback_unit in {"g", "kg"}:
        return fallback_unit
    return "kg"


def load_asset_metadata(xlsx_path: Path, fallback_mass_unit: str = "auto") -> List[AssetMetadata]:
    fallback_mass_unit = str(fallback_mass_unit).lower()
    if fallback_mass_unit not in SUPPORTED_MASS_UNITS:
        raise ValueError(
            f"Unsupported metadata mass unit {fallback_mass_unit!r}; "
            f"expected one of {sorted(SUPPORTED_MASS_UNITS)}"
        )
    if not xlsx_path.exists():
        print(f"Warning: metadata xlsx not found: {xlsx_path}")
        return []

    rows = _parse_xlsx_rows(xlsx_path)
    if not rows:
        print(f"Warning: metadata xlsx is empty: {xlsx_path}")
        return []

    headers = [_normalize_header(value) for value in rows[0]]
    name_col = _find_column(headers, NAME_COLUMN_HEADERS)
    mass_col = _find_column(headers, MASS_COLUMN_HEADERS)
    mass_unit = (
        _mass_unit_from_header(headers[mass_col], fallback_mass_unit)
        if mass_col is not None
        else fallback_mass_unit
    )
    axis_cols = {axis: _find_column(headers, [axis]) for axis in AXES}

    if name_col is None:
        raise ValueError(f"Cannot find a name column in {xlsx_path}")

    records: List[AssetMetadata] = []
    current_name = ""
    for row_number, row in enumerate(rows[1:], start=2):
        row_name = str(_row_value(row, name_col) or "").strip()
        if row_name:
            current_name = row_name
        elif not current_name:
            continue

        mass_value = _parse_number(_row_value(row, mass_col))
        if mass_value is None:
            mass_kg = None
        elif mass_unit == "g":
            mass_kg = mass_value / MASS_GRAMS_PER_KILOGRAM
        elif mass_unit == "kg":
            mass_kg = mass_value
        else:
            raise ValueError(f"Unsupported metadata mass unit: {mass_unit}")

        dimensions_cm = {
            axis: value
            for axis, col in axis_cols.items()
            if (value := _parse_number(_row_value(row, col))) is not None and value > 0
        }

        if mass_kg is None and not dimensions_cm:
            continue

        records.append(
            AssetMetadata(
                name=current_name,
                mass_kg=mass_kg,
                dimensions_cm=dimensions_cm,
                row_number=row_number,
            )
        )

    print(f"Loaded {len(records)} metadata rows from {xlsx_path}")
    return records


def _path_parts_for_matching(obj_path: Path, assets_root: Path) -> List[str]:
    try:
        relative = obj_path.relative_to(assets_root)
        return list(relative.parts)
    except ValueError:
        return list(obj_path.parts)


def _numbers_in_parts(parts: Sequence[str]) -> List[float]:
    numbers: List[float] = []
    for part in parts:
        for match in re.finditer(r"\d+(?:\.\d+)?", part):
            numbers.append(float(match.group(0)))
    return numbers


def match_metadata(
    obj_path: Path, records: Sequence[AssetMetadata], assets_root: Path
) -> Optional[AssetMetadata]:
    if not records:
        return None

    parts = _path_parts_for_matching(obj_path, assets_root)
    lower_parts = [part.lower() for part in parts]
    by_name: Dict[str, List[AssetMetadata]] = {}
    for record in records:
        by_name.setdefault(record.name.lower(), []).append(record)

    candidates: List[AssetMetadata] = []
    if lower_parts and lower_parts[0] in by_name:
        candidates = by_name[lower_parts[0]]
    else:
        for name, name_records in by_name.items():
            if name in lower_parts or obj_path.stem.lower() == name:
                candidates.extend(name_records)

    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]

    path_numbers = _numbers_in_parts(parts)
    scored: List[Tuple[int, AssetMetadata]] = []
    for record in candidates:
        score = 0
        if record.mass_kg is not None:
            mass_numbers = [record.mass_kg, record.mass_kg * MASS_GRAMS_PER_KILOGRAM]
            for number in path_numbers:
                if any(abs(number - mass_number) < 1e-6 for mass_number in mass_numbers):
                    score += 10
        scored.append((score, record))

    best_score, best_record = max(scored, key=lambda item: item[0])
    if best_score <= 0:
        print(
            "Warning: ambiguous metadata for "
            f"{obj_path}; using row {best_record.row_number} ({best_record.name})"
        )
    return best_record


def _has_mesh_descendant(prim: Usd.Prim) -> bool:
    if prim.IsA(UsdGeom.Mesh):
        return True
    return any(_has_mesh_descendant(child) for child in prim.GetChildren())


def find_model_prim(stage: Usd.Stage) -> Optional[Usd.Prim]:
    root_prim = stage.GetPrimAtPath(ROOT_PATH)
    if root_prim and root_prim.IsValid():
        return root_prim

    default_prim = stage.GetDefaultPrim()
    roots = [default_prim] if default_prim and default_prim.IsValid() else []
    if not roots:
        roots = [child for child in stage.GetPseudoRoot().GetChildren()]

    for root in roots:
        children = [
            child
            for child in root.GetChildren()
            if child.GetName() not in IGNORED_PRIM_NAMES and _has_mesh_descendant(child)
        ]
        if children:
            return children[0]
        if _has_mesh_descendant(root):
            return root

    return None


def iter_mesh_prims(prim: Usd.Prim) -> Iterable[Usd.Prim]:
    if prim.IsA(UsdGeom.Mesh):
        yield prim
    for child in prim.GetChildren():
        yield from iter_mesh_prims(child)


def _pump_app_updates(max_updates: int) -> None:
    require_isaac_runtime()
    for _ in range(max_updates):
        simulation_app.update()


def _wait_for_stage_idle(max_updates: int = STAGE_OPEN_MAX_UPDATES) -> bool:
    require_isaac_runtime()
    context = omni_usd.get_context()
    for _ in range(max_updates):
        _, _, loading = context.get_stage_loading_status()
        if loading <= 0:
            return True
        simulation_app.update()
    return False


def open_current_stage(usd_file: Path) -> Usd.Stage:
    require_isaac_runtime()
    context = omni_usd.get_context()
    if not context.open_stage(str(usd_file)):
        raise RuntimeError(f"Failed to open stage: {usd_file}")

    if not _wait_for_stage_idle():
        raise RuntimeError(f"Timed out waiting for stage to load: {usd_file}")

    stage = context.get_stage()
    if stage is None:
        raise RuntimeError(f"Failed to get current stage: {usd_file}")
    return stage


def close_current_stage() -> None:
    require_isaac_runtime()
    context = omni_usd.get_context()
    if context.get_stage() is None:
        return

    context.close_stage()
    _wait_for_stage_idle(STAGE_CLOSE_MAX_UPDATES)


def compute_stage_aabb(stage: Usd.Stage) -> AabbInfo:
    """Compute the model AABB with every authored transform applied."""
    target_prim = find_model_prim(stage)
    if not target_prim:
        raise RuntimeError("No mesh model prim found in current stage")

    # World bounds are intentional here. ComputeUntransformedBound() ignores
    # the target prim's own xform ops, which made the reported post-scale size
    # differ from the geometry that Isaac Sim actually loaded.
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bbox = bbox_cache.ComputeWorldBound(target_prim)
    aligned_range = bbox.ComputeAlignedRange()
    
    min_pt = aligned_range.GetMin()
    max_pt = aligned_range.GetMax()

    aabb = (
        float(min_pt[0]), float(min_pt[1]), float(min_pt[2]),
        float(max_pt[0]), float(max_pt[1]), float(max_pt[2])
    )
    
    dimensions = {
        "x": aabb[3] - aabb[0],
        "y": aabb[4] - aabb[1],
        "z": aabb[5] - aabb[2],
    }
    return AabbInfo(aabb=aabb, dimensions=dimensions, prim_path=target_prim.GetPath().pathString)


def _dimension_unit_factor(unit: str) -> float:
    unit = str(unit).lower()
    try:
        return DIMENSION_UNIT_FACTORS[unit]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported dimension unit {unit!r}; "
            f"expected one of {sorted(DIMENSION_UNIT_FACTORS)}"
        ) from exc


def _metadata_dimensions_in_stage_units(
    metadata: Optional[AssetMetadata], dimension_unit: str
) -> Dict[str, float]:
    if not metadata or not metadata.dimensions_cm:
        return {}

    unit_factor = _dimension_unit_factor(dimension_unit)
    return {
        axis: dimension * unit_factor
        for axis, dimension in metadata.dimensions_cm.items()
        if dimension > 0
    }


def resolve_axis_scales(
    metadata: Optional[AssetMetadata],
    aabb: AabbInfo,
    fallback_scale: float,
    dimension_unit: str,
) -> Tuple[Dict[str, float], str]:
    if fallback_scale <= 0:
        raise ValueError("fallback_scale must be greater than zero")
    real_dims = _metadata_dimensions_in_stage_units(metadata, dimension_unit)
    obj_dims = {
        axis: dimension
        for axis, dimension in aabb.dimensions.items()
        if dimension > 0
    }

    scale_values: Dict[str, float] = {}
    fallback_axes: List[str] = []
    for axis in AXES:
        real_dim = real_dims.get(axis)
        obj_dim = obj_dims.get(axis)
        if real_dim is not None and obj_dim is not None:
            scale_values[axis] = real_dim / obj_dim
        else:
            scale_values[axis] = fallback_scale
            fallback_axes.append(axis)

    if not real_dims or not obj_dims:
        return scale_values, "fallback"
    if fallback_axes:
        return scale_values, f"mixed (fallback axes: {', '.join(fallback_axes)})"
    return scale_values, "metadata"


def resolve_mass_kg(metadata: Optional[AssetMetadata], fallback_mass_kg: float) -> float:
    if fallback_mass_kg <= 0:
        raise ValueError("fallback_mass_kg must be greater than zero")
    if metadata is None or metadata.mass_kg is None:
        return fallback_mass_kg
    if metadata.mass_kg <= 0:
        raise ValueError(f"Metadata mass must be greater than zero: {metadata.mass_kg}")
    return metadata.mass_kg


def _set_scalar_custom_attr(target_prim: Usd.Prim, attr_name: str, value: float) -> None:
    attr = target_prim.GetAttribute(attr_name)
    if not attr or not attr.IsValid():
        attr = target_prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Double, True)
    attr.Set(float(value))


def _set_axis_custom_attrs(
    target_prim: Usd.Prim, prefix: str, axis_values: Dict[str, float]
) -> None:
    for axis in AXES:
        if axis in axis_values:
            _set_scalar_custom_attr(target_prim, f"{prefix}_{axis}", axis_values[axis])


def _apply_mesh_collider(mesh_prim: Usd.Prim) -> None:
    require_isaac_runtime()
    collision_api = UsdPhysics.CollisionAPI.Apply(mesh_prim)
    collision_api.CreateCollisionEnabledAttr(True)

    PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)
    convex_api = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(mesh_prim)
    mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
    mesh_collision_api.CreateApproximationAttr(COLLISION_APPROXIMATION)

    convex_api.CreateMaxConvexHullsAttr(COLLISION_MAX_CONVEX_HULLS)
    convex_api.CreateHullVertexLimitAttr(COLLISION_HULL_VERTEX_LIMIT)
    convex_api.CreateMinThicknessAttr(COLLISION_MIN_THICKNESS)
    convex_api.CreateShrinkWrapAttr(COLLISION_SHRINK_WRAP)
    convex_api.CreateErrorPercentageAttr(COLLISION_ERROR_PERCENTAGE)


def _set_identity_transform(prim: Usd.Prim) -> None:
    """Author the same identity xform stack used by the existing assets."""

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0))
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0))


def _z_up_rotation(source_up_axis: str) -> Gf.Rotation:
    if source_up_axis == UsdGeom.Tokens.z:
        return Gf.Rotation()
    if source_up_axis == UsdGeom.Tokens.y:
        return Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), 90.0)
    if source_up_axis == UsdGeom.Tokens.x:
        return Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), -90.0)
    raise ValueError(f"Unsupported USD up axis: {source_up_axis}")


def _transform_point(
    point: Gf.Vec3f,
    source_to_world: Gf.Matrix4d,
    scale_values: Dict[str, float],
    z_up_rotation: Gf.Rotation,
) -> Gf.Vec3f:
    world_point = source_to_world.Transform(Gf.Vec3d(point))
    scaled_point = Gf.Vec3d(
        world_point[0] * scale_values["x"],
        world_point[1] * scale_values["y"],
        world_point[2] * scale_values["z"],
    )
    aligned_point = z_up_rotation.TransformDir(scaled_point)
    if not all(math.isfinite(float(component)) for component in aligned_point):
        raise ValueError("Mesh transform produced a non-finite point")
    return Gf.Vec3f(aligned_point)


def _transform_normal(
    normal: Gf.Vec3f,
    source_normal_matrix: Gf.Matrix4d,
    scale_values: Dict[str, float],
    z_up_rotation: Gf.Rotation,
) -> Gf.Vec3f:
    world_normal = source_normal_matrix.TransformDir(Gf.Vec3d(normal))
    # Normals use the inverse transpose of the per-axis scale.
    scaled_normal = Gf.Vec3d(
        world_normal[0] / scale_values["x"],
        world_normal[1] / scale_values["y"],
        world_normal[2] / scale_values["z"],
    )
    aligned_normal = z_up_rotation.TransformDir(scaled_normal)
    length = aligned_normal.GetLength()
    if not math.isfinite(length) or length <= 1.0e-12:
        raise ValueError("Mesh contains an invalid or zero-length normal")
    return Gf.Vec3f(aligned_normal / length)


def _bake_mesh_transform(
    mesh_prim: Usd.Prim,
    source_to_world: Gf.Matrix4d,
    scale_values: Dict[str, float],
    z_up_rotation: Gf.Rotation,
) -> None:
    """Bake source hierarchy, physical scale, and up-axis into mesh data."""

    mesh = UsdGeom.Mesh(mesh_prim)
    points = mesh.GetPointsAttr().Get(Usd.TimeCode.Default())
    if not points:
        raise RuntimeError(f"Mesh has no points: {mesh_prim.GetPath()}")

    transformed_points = [
        _transform_point(point, source_to_world, scale_values, z_up_rotation)
        for point in points
    ]
    mesh.GetPointsAttr().Set(transformed_points)

    normals = mesh.GetNormalsAttr().Get(Usd.TimeCode.Default())
    if normals:
        source_normal_matrix = source_to_world.GetInverse().GetTranspose()
        mesh.GetNormalsAttr().Set(
            [
                _transform_normal(
                    normal,
                    source_normal_matrix,
                    scale_values,
                    z_up_rotation,
                )
                for normal in normals
            ]
        )

    minimum = Gf.Vec3f(
        *(min(float(point[axis]) for point in transformed_points) for axis in range(3))
    )
    maximum = Gf.Vec3f(
        *(max(float(point[axis]) for point in transformed_points) for axis in range(3))
    )
    mesh.CreateExtentAttr().Set([minimum, maximum])
    UsdGeom.Xformable(mesh_prim).ClearXformOpOrder()


def _remap_property_paths(prim: Usd.Prim, path_map: Dict[Sdf.Path, Sdf.Path]) -> None:
    """Retarget copied material bindings and shader connections."""

    def remap(path: Sdf.Path) -> Sdf.Path:
        for source, destination in sorted(
            path_map.items(), key=lambda item: len(item[0].pathString), reverse=True
        ):
            if path.HasPrefix(source):
                return path.ReplacePrefix(source, destination)
        return path

    for descendant in Usd.PrimRange(prim):
        for relationship in descendant.GetRelationships():
            targets = relationship.GetTargets()
            remapped = [remap(path) for path in targets]
            if remapped != targets:
                relationship.SetTargets(remapped)
        for attribute in descendant.GetAttributes():
            connections = attribute.GetConnections()
            remapped = [remap(path) for path in connections]
            if remapped != connections:
                attribute.SetConnections(remapped)


def _copy_materials(
    stage: Usd.Stage,
    materials_scope: Usd.Prim,
    source_materials: Sequence[Usd.Prim],
) -> Dict[Sdf.Path, Sdf.Path]:
    root_layer = stage.GetRootLayer()
    path_map: Dict[Sdf.Path, Sdf.Path] = {}
    for index, source_material in enumerate(source_materials):
        destination = materials_scope.GetPath().AppendChild(f"mat{index}")
        if not Sdf.CopySpec(
            root_layer,
            source_material.GetPath(),
            root_layer,
            destination,
        ):
            raise RuntimeError(f"Failed to copy material: {source_material.GetPath()}")
        path_map[source_material.GetPath()] = destination

    for destination in path_map.values():
        _remap_property_paths(stage.GetPrimAtPath(destination), path_map)
    return path_map


def _top_level_path(path: Sdf.Path) -> Sdf.Path:
    prefixes = path.GetPrefixes()
    if not prefixes:
        raise ValueError(f"Cannot find top-level prim for path: {path}")
    return prefixes[0]


def create_standard_structure_and_move_mesh(
    stage: Usd.Stage,
    scale_values: Dict[str, float],
) -> Usd.Prim:
    """Create /root/{_materials,visual,collision} and bake one merged mesh."""

    source_meshes = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
    if len(source_meshes) != 1:
        raise RuntimeError(
            "Expected one merged mesh from AssetConverter, "
            f"found {len(source_meshes)}"
        )
    source_mesh = source_meshes[0]
    source_materials = [
        prim for prim in stage.Traverse() if prim.IsA(UsdShade.Material)
    ]
    source_to_world = UsdGeom.XformCache(
        Usd.TimeCode.Default()
    ).GetLocalToWorldTransform(source_mesh)
    z_up_rotation = _z_up_rotation(UsdGeom.GetStageUpAxis(stage))
    root_layer = stage.GetRootLayer()

    existing_root = stage.GetPrimAtPath(ROOT_PATH)
    if existing_root and existing_root.IsValid():
        raise RuntimeError(f"AssetConverter output already contains {ROOT_PATH}")

    root_prim = UsdGeom.Xform.Define(stage, ROOT_PATH).GetPrim()
    materials_prim = UsdGeom.Scope.Define(stage, MATERIALS_PATH).GetPrim()
    visual_prim = UsdGeom.Xform.Define(stage, VISUAL_PATH).GetPrim()
    collision_prim = UsdGeom.Xform.Define(stage, COLLISION_PATH).GetPrim()
    for prim in (root_prim, visual_prim, collision_prim):
        _set_identity_transform(prim)

    visual_mesh_path = Sdf.Path(f"{VISUAL_PATH}/{MODEL_NAME}")
    collision_mesh_path = Sdf.Path(f"{COLLISION_PATH}/{MODEL_NAME}")
    for destination in (visual_mesh_path, collision_mesh_path):
        if not Sdf.CopySpec(
            root_layer,
            source_mesh.GetPath(),
            root_layer,
            destination,
        ):
            raise RuntimeError(
                f"Failed to copy mesh {source_mesh.GetPath()} to {destination}"
            )

    material_path_map = _copy_materials(stage, materials_prim, source_materials)
    for destination in (visual_mesh_path, collision_mesh_path):
        destination_prim = stage.GetPrimAtPath(destination)
        _remap_property_paths(
            destination_prim,
            {
                source_mesh.GetPath(): destination,
                **material_path_map,
            },
        )
        _bake_mesh_transform(
            destination_prim,
            source_to_world,
            scale_values,
            z_up_rotation,
        )

    source_roots = {
        _top_level_path(source_mesh.GetPath()),
        *(_top_level_path(material.GetPath()) for material in source_materials),
    }
    for source_root in source_roots:
        if source_root != root_prim.GetPath():
            stage.RemovePrim(source_root)

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(root_prim)
    return root_prim


def _add_physics_material(stage: Usd.Stage, target_prim: Usd.Prim) -> None:
    materials_scope = stage.GetPrimAtPath(MATERIALS_PATH)
    material_parent = (
        materials_scope.GetPath().pathString
        if materials_scope.IsValid()
        else ROOT_PATH
    )
    material_path = Sdf.Path(f"{material_parent}/{PHYSICS_MATERIAL_NAME}")

    material = UsdShade.Material.Define(stage, material_path)
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api.CreateStaticFrictionAttr(PHYSICS_FRICTION)
    material_api.CreateDynamicFrictionAttr(PHYSICS_FRICTION)

    binding_api = UsdShade.MaterialBindingAPI.Apply(target_prim)
    binding_api.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")


def apply_physics(
    stage: Usd.Stage,
    mass_value: float,
    scale_values: Dict[str, float],
    *,
    save: bool = True,
) -> None:
    # Keep the body at the root so visual and collision meshes share one pose.
    root_prim = stage.GetPrimAtPath(ROOT_PATH)
    if not root_prim or not root_prim.IsValid():
        raise RuntimeError(f"Root prim {ROOT_PATH!r} not found in stage")

    _set_axis_custom_attrs(root_prim, CUSTOM_SCALE_PREFIX, scale_values)

    # Bind RigidBody and MassAPI to the root prim.
    UsdPhysics.RigidBodyAPI.Apply(root_prim)
    mass_api = UsdPhysics.MassAPI.Apply(root_prim)
    mass_api.CreateMassAttr(mass_value)

    # Collision attributes belong only to meshes below the collision group.
    collision_group_prim = stage.GetPrimAtPath(COLLISION_PATH)
    mesh_prims = (
        list(iter_mesh_prims(collision_group_prim))
        if collision_group_prim.IsValid()
        else []
    )
    if not mesh_prims:
        print(f"Warning: no collision mesh prim found under {COLLISION_PATH}")
    for mesh_prim in mesh_prims:
        _apply_mesh_collider(mesh_prim)

    _add_physics_material(stage, root_prim)
    if save:
        stage.GetRootLayer().Save()


def write_real_dimensions(
    stage: Usd.Stage,
    real_dimensions: Dict[str, float],
    *,
    save: bool = True,
) -> None:
    root_prim = stage.GetPrimAtPath(ROOT_PATH)
    if not root_prim or not root_prim.IsValid():
        raise RuntimeError(f"Root prim {ROOT_PATH!r} not found in stage")

    _set_axis_custom_attrs(root_prim, CUSTOM_DIMENSION_PREFIX, real_dimensions)
    if save:
        stage.GetRootLayer().Save()


def simplify_mesh_with_pymeshlab(
    input_obj: Path,
    target_faces: int = DEFAULT_TARGET_FACES,
) -> Optional[Path]:
    if target_faces <= 0:
        raise ValueError("target_faces must be greater than zero")
    try:
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(str(input_obj))

        current_mesh = ms.current_mesh()
        num_faces = current_mesh.face_number()

        if num_faces <= target_faces:
            return input_obj

        print(
            f"  [MeshLab] reducing faces: {input_obj.name} "
            f"({num_faces} -> {target_faces})"
        )
        ms.meshing_decimation_quadric_edge_collapse(
            targetfacenum=target_faces,
            preservenormal=True,
            preserveboundary=True,
            preservetopology=True,
            autoclean=True,
        )
        temp_dir = Path(tempfile.mkdtemp(prefix="obj-simplify-"))
        temp_obj = temp_dir / input_obj.name
        try:
            ms.save_current_mesh(str(temp_obj))
        except Exception:
            shutil.rmtree(temp_dir, ignore_errors=True)
            raise
        return temp_obj

    except Exception as exc:
        print(f"  [MeshLab] process failed {input_obj}: {exc}")
        return None


def build_conversion_config(args: argparse.Namespace) -> ConversionConfig:
    """Convert CLI values into normalized, validated pipeline settings."""
    assets_root = Path(args.assets_root).expanduser().resolve()
    folder_values = args.folders or [str(assets_root / DEFAULT_TEST_FOLDER)]
    folders = tuple(Path(folder).expanduser().resolve() for folder in folder_values)
    metadata_xlsx = Path(
        args.metadata_xlsx or assets_root / DEFAULT_METADATA_FILENAME
    ).expanduser().resolve()
    output_root = Path(
        getattr(args, "output_root", None)
        or assets_root.parent / DEFAULT_OUTPUT_DIRECTORY
    ).expanduser().resolve()

    metadata_mass_unit = str(args.metadata_mass_unit).lower()
    dimension_unit = str(args.dimension_unit).lower()
    if metadata_mass_unit not in SUPPORTED_MASS_UNITS:
        raise ValueError(
            f"Unsupported --metadata-mass-unit {metadata_mass_unit!r}; "
            f"expected one of {sorted(SUPPORTED_MASS_UNITS)}"
        )
    if dimension_unit not in DIMENSION_UNIT_FACTORS:
        raise ValueError(
            f"Unsupported --dimension-unit {dimension_unit!r}; "
            f"expected one of {sorted(DIMENSION_UNIT_FACTORS)}"
        )

    fallback_mass_kg = DEFAULT_MASS_KG if args.mass is None else args.mass
    fallback_scale = DEFAULT_SCALE if args.scale is None else args.scale
    target_faces = getattr(args, "target_faces", DEFAULT_TARGET_FACES)
    if fallback_mass_kg <= 0:
        raise ValueError("--mass must be greater than zero")
    if fallback_scale <= 0:
        raise ValueError("--scale must be greater than zero")
    if target_faces <= 0:
        raise ValueError("--target-faces must be greater than zero")
    if args.max_models < 0:
        raise ValueError("--max-models cannot be negative")

    return ConversionConfig(
        assets_root=assets_root,
        folders=folders,
        metadata_xlsx=metadata_xlsx,
        output_root=output_root,
        fallback_mass_kg=fallback_mass_kg,
        fallback_scale=fallback_scale,
        metadata_mass_unit=metadata_mass_unit,
        dimension_unit=dimension_unit,
        max_models=args.max_models,
        force=args.force,
        extract_zips=args.extract_zips,
        target_faces=target_faces,
    )


def _run_async(loop: asyncio.AbstractEventLoop, coroutine: Any) -> Any:
    """Run one converter coroutine on the pipeline's dedicated event loop."""
    return loop.run_until_complete(coroutine)


def _export_path(input_path: Path, config: ConversionConfig) -> Path:
    """Build a mirrored OBJ output path without allowing ``..`` traversal."""
    try:
        relative_parent = input_path.relative_to(config.assets_root).parent
    except ValueError:
        relative_parent = Path(input_path.parent.name)
    return config.output_root / relative_parent / DEFAULT_OBJ_FILENAME


def _cleanup_temporary_mesh(mesh_path: Optional[Path], source_path: Path) -> None:
    if mesh_path is None or mesh_path == source_path:
        return
    try:
        if mesh_path.exists():
            mesh_path.unlink()
        if mesh_path.parent.exists() and mesh_path.parent != source_path.parent:
            shutil.rmtree(mesh_path.parent)
    except OSError as exc:
        print(f"Warning: failed to clean temporary mesh {mesh_path}: {exc}")


def _remove_partial_outputs(paths: Sequence[Optional[Path]]) -> None:
    for path in paths:
        if path is None:
            continue
        try:
            path.unlink(missing_ok=True)
        except OSError as exc:
            print(f"Warning: failed to remove partial output {path}: {exc}")


def _convert_one_asset(
    input_path: Path,
    config: ConversionConfig,
    metadata_records: Sequence[AssetMetadata],
    loop: asyncio.AbstractEventLoop,
) -> ConvertedAsset:
    output_path = input_path.with_name(DEFAULT_USD_FILENAME)
    if output_path.exists() and config.force:
        output_path.unlink()

    mesh_path = simplify_mesh_with_pymeshlab(input_path, config.target_faces)
    if mesh_path is None:
        raise RuntimeError("Mesh simplification failed")

    export_obj: Optional[Path] = None
    try:
        if not _run_async(loop, convert_asset_to_usd(str(mesh_path), str(output_path))):
            raise RuntimeError("OBJ to USD conversion failed")

        metadata = match_metadata(input_path, metadata_records, config.assets_root)
        stage = open_current_stage(output_path)
        try:
            source_aabb = compute_stage_aabb(stage)
            scale_values, scale_source = resolve_axis_scales(
                metadata,
                source_aabb,
                config.fallback_scale,
                config.dimension_unit,
            )
            mass_kg = resolve_mass_kg(metadata, config.fallback_mass_kg)

            create_standard_structure_and_move_mesh(stage, scale_values)
            apply_physics(stage, mass_kg, scale_values, save=False)

            real_dimensions = compute_stage_aabb(stage).dimensions
            write_real_dimensions(stage, real_dimensions, save=False)
            stage.GetRootLayer().Save()

            export_obj = _export_path(input_path, config)
            export_obj.parent.mkdir(parents=True, exist_ok=True)
            if not _run_async(
                loop,
                convert_usd_to_obj(str(output_path), str(export_obj)),
            ):
                raise RuntimeError("USD to OBJ conversion failed")
        finally:
            _pump_app_updates(POST_STAGE_UPDATE_COUNT)
            close_current_stage()

        return ConvertedAsset(
            metadata=metadata,
            source_aabb=source_aabb,
            scale_values=scale_values,
            scale_source=scale_source,
            real_dimensions=real_dimensions,
        )
    except Exception:
        _remove_partial_outputs((output_path, export_obj))
        raise
    finally:
        _cleanup_temporary_mesh(mesh_path, input_path)


def _print_conversion_result(
    input_path: Path,
    output_path: Path,
    result: ConvertedAsset,
) -> None:
    metadata = result.metadata
    metadata_label = (
        f"{metadata.name} row {metadata.row_number}" if metadata else "fallback"
    )
    dimensions = result.source_aabb.dimensions
    scales = result.scale_values
    real = result.real_dimensions
    print(
        "AABB dims(m): "
        f"x={dimensions['x']:.6g}, y={dimensions['y']:.6g}, z={dimensions['z']:.6g}; "
        f"scale_x={scales['x']:.6g}, scale_y={scales['y']:.6g}, "
        f"scale_z={scales['z']:.6g} ({result.scale_source}); "
        f"real_x={real['x']:.6g}, real_y={real['y']:.6g}, "
        f"real_z={real['z']:.6g}; metadata={metadata_label}"
    )
    print(f"--- Success: {output_path} (source: {input_path})")


def asset_convert_pipeline(args: argparse.Namespace) -> None:
    config = build_conversion_config(args)
    metadata_records = load_asset_metadata(config.metadata_xlsx, config.metadata_mass_unit)

    if getattr(args, "scale_axis", "auto") != "auto":
        print(
            "Warning: --scale-axis is deprecated and ignored; "
            "scale_x/scale_y/scale_z are computed independently from x/y/z metadata."
        )

    existing_folders = [folder for folder in config.folders if folder.exists()]
    for folder in set(config.folders) - set(existing_folders):
        print(f"Folder not found: {folder}")
    if not existing_folders:
        print("No valid folders to scan.")
        return

    if config.extract_zips:
        zip_stats = ZipExtractionStats()
        for folder in existing_folders:
            print(f"\n[Extracting] {folder}")
            zip_stats.add(extract_zips_in_tree(folder))
        print(
            "Zip summary: "
            f"total={zip_stats.total}, extracted={zip_stats.extracted}, "
            f"git_lfs_pointers={zip_stats.lfs_pointers}, failed={zip_stats.failed}"
        )

    obj_paths = sorted(iter_obj_files(existing_folders))
    print(f"\nFound {len(obj_paths)} OBJ files.")
    converted = skipped_existing = failed = 0
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        for input_path in obj_paths:
            if config.max_models > 0 and converted >= config.max_models:
                print(f"Reached max models limit ({config.max_models})")
                break

            output_path = input_path.with_name(DEFAULT_USD_FILENAME)
            if output_path.exists() and not config.force:
                print(f"Skipping existing USD: {output_path}")
                skipped_existing += 1
                continue

            print(f"\nConverting: {input_path}")
            try:
                result = _convert_one_asset(input_path, config, metadata_records, loop)
                _print_conversion_result(input_path, output_path, result)
                converted += 1
            except Exception as exc:
                failed += 1
                print(f"--- Failed: {input_path}: {exc}")
            finally:
                simulation_app.update()
    finally:
        loop.close()
        asyncio.set_event_loop(None)

    print(
        "\nSummary: "
        f"obj_files={len(obj_paths)}, converted={converted}, "
        f"skipped_existing={skipped_existing}, failed={failed}, "
        f"attempted={converted + failed}"
    )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert OBJ assets to USD, apply per-axis metadata scale, and "
            "write post-scale dimensions."
        )
    )
    parser.add_argument("--assets-root", type=str, default=DEFAULT_ASSETS_ROOT)
    parser.add_argument(
        "--folders",
        nargs="+",
        default=None,
        help="Folders to scan; defaults to <assets-root>/vase.",
    )
    parser.add_argument(
        "--metadata-xlsx",
        type=str,
        default=None,
        help=f"Metadata workbook; defaults to <assets-root>/{DEFAULT_METADATA_FILENAME}.",
    )
    parser.add_argument(
        "--output-root",
        type=str,
        default=None,
        help=f"Directory for exported OBJ files; defaults to ../{DEFAULT_OUTPUT_DIRECTORY}.",
    )
    parser.add_argument("--mass", type=float, default=None)
    parser.add_argument("--scale", type=float, default=None)
    parser.add_argument("--metadata-mass-unit", type=str, default="auto")
    parser.add_argument("--dimension-unit", type=str, default="cm")
    parser.add_argument("--scale-axis", type=str, default="auto")
    parser.add_argument("--max-models", type=int, default=0)
    parser.add_argument(
        "--target-faces",
        type=int,
        default=DEFAULT_TARGET_FACES,
        help="Maximum faces passed to MeshLab simplification.",
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--extract-zips", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    initialize_isaac_runtime()
    try:
        asset_convert_pipeline(args)
    finally:
        if simulation_app is not None:
            simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
