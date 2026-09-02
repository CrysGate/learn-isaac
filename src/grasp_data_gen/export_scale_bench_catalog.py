"""Export physics-validated grasps as a ScaleBench runtime catalog."""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from pathlib import Path

import yaml

from grasp_data_gen.models import GraspFileData
from grasp_data_gen.results import load_grasp_file
from scale_bench.config.loader import load_config
from scale_bench.config.models.grasp import (
    GraspCandidateConfig,
    GraspCatalogConfig,
)
from scale_bench.config.models.robot import RobotConfig
from scale_bench.skills.geometry import quaternion_angular_distance_rad


PROJECT_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True, slots=True)
class CatalogExportRequest:
    robot_config_path: Path
    output_path: Path
    max_candidates_per_object: int
    object_grasp_files: tuple[tuple[str, Path], ...]


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def _object_grasp_file(value: str) -> tuple[str, Path]:
    object_name, separator, path_text = value.partition("=")
    if not separator or not object_name or not path_text:
        raise argparse.ArgumentTypeError("must use OBJECT_NAME=GRASP_FILE")
    path = Path(path_text).expanduser().resolve()
    if not path.is_file():
        raise argparse.ArgumentTypeError(f"grasp file does not exist: {path}")
    return object_name, path


def _parse_args() -> CatalogExportRequest:
    parser = argparse.ArgumentParser(
        description=(
            "Export grasp_data_gen results as a compact ScaleBench catalog."
        )
    )
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--max-candidates-per-object",
        type=_positive_int,
        required=True,
    )
    parser.add_argument(
        "--object-grasp",
        type=_object_grasp_file,
        action="append",
        required=True,
        metavar="OBJECT_NAME=GRASP_FILE",
    )
    args = parser.parse_args()
    object_grasp_files = tuple(args.object_grasp)
    names = tuple(name for name, _ in object_grasp_files)
    if len(names) != len(set(names)):
        parser.error("--object-grasp object names must be unique")
    return CatalogExportRequest(
        robot_config_path=args.robot_config.expanduser().resolve(),
        output_path=args.output.expanduser().resolve(),
        max_candidates_per_object=args.max_candidates_per_object,
        object_grasp_files=object_grasp_files,
    )


def _validate_source(
    data: GraspFileData,
    robot: RobotConfig,
    path: Path,
) -> float:
    tcp = robot.kinematics.tcp
    if data.candidate_pose_frame != f"{robot.name}_tcp":
        raise ValueError(
            f"{path} candidate pose frame does not match robot {robot.name!r}"
        )
    if (
        data.tcp_definition.parent_frame != tcp.parent_frame
        or data.tcp_definition.offset_in_parent_m != tcp.position_m
        or data.tcp_definition.tcp_orientation_parent_xyzw
        != tcp.orientation_xyzw
    ):
        raise ValueError(f"{path} TCP definition does not match RobotConfig")
    approach_distance_m = float(data.tabletop_filter["approach_distance_m"])
    if not math.isfinite(approach_distance_m) or approach_distance_m <= 0.0:
        raise ValueError(f"{path} has an invalid approach distance")
    return approach_distance_m


def _orientation_distance(
    left: GraspCandidateConfig,
    right: GraspCandidateConfig,
) -> float:
    return quaternion_angular_distance_rad(
        left.orientation_object_xyzw,
        right.orientation_object_xyzw,
    )


def _select_diverse(
    candidates: tuple[GraspCandidateConfig, ...],
    limit: int,
) -> tuple[GraspCandidateConfig, ...]:
    """Keep high-quality grasps while covering distinct wrist orientations."""

    if len(candidates) <= limit:
        return candidates
    selected = [0]
    remaining = set(range(1, len(candidates)))
    while len(selected) < limit:
        index = max(
            remaining,
            key=lambda candidate_index: (
                min(
                    _orientation_distance(
                        candidates[candidate_index],
                        candidates[selected_index],
                    )
                    for selected_index in selected
                ),
                candidates[candidate_index].score,
                -candidate_index,
            ),
        )
        selected.append(index)
        remaining.remove(index)
    return tuple(candidates[index] for index in sorted(selected))


def _catalog_candidates(
    data: GraspFileData,
    limit: int,
) -> tuple[GraspCandidateConfig, ...]:
    candidates = tuple(
        GraspCandidateConfig(
            candidate_id=grasp.candidate_id,
            position_object_m=grasp.position_object_m,
            orientation_object_xyzw=grasp.orientation_object_xyzw,
            approach_axis_tcp=grasp.approach_axis_tcp,
            score=grasp.score,
        )
        for grasp in data.grasps
    )
    return _select_diverse(candidates, limit)


def _write_catalog(path: Path, catalog: GraspCatalogConfig) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            catalog.model_dump(mode="json"),
            stream,
            sort_keys=False,
        )
    os.replace(temporary, path)


def main() -> None:
    request = _parse_args()
    robot = load_config(
        request.robot_config_path,
        RobotConfig,
        asset_root=PROJECT_ROOT,
    )
    objects = {}
    approach_distances = set()
    source_counts = {}
    for object_name, grasp_file in request.object_grasp_files:
        data = load_grasp_file(grasp_file)
        approach_distances.add(_validate_source(data, robot, grasp_file))
        objects[object_name] = _catalog_candidates(
            data,
            request.max_candidates_per_object,
        )
        source_counts[object_name] = len(data.grasps)
    if len(approach_distances) != 1:
        raise ValueError("all grasp files must use the same approach distance")

    tcp = robot.kinematics.tcp
    catalog = GraspCatalogConfig(
        robot_name=robot.name,
        tcp_parent_frame=tcp.parent_frame,
        tcp_position_m=tcp.position_m,
        tcp_orientation_xyzw=tcp.orientation_xyzw,
        approach_distance_m=next(iter(approach_distances)),
        objects=objects,
    )
    _write_catalog(request.output_path, catalog)
    selected_counts = {
        name: len(candidates) for name, candidates in catalog.objects.items()
    }
    print(
        "GRASP_CATALOG_RESULT "
        f"sources={source_counts} selected={selected_counts} "
        f"output={request.output_path}",
        flush=True,
    )


if __name__ == "__main__":
    main()
