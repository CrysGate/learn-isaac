"""Physics-validated grasp collection helpers."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

import yaml
from pydantic import ValidationError

from scale_bench.config.models.grasp import (
    GraspCandidateConfig,
    GraspCatalogConfig,
)
from scale_bench.config.models.robot import RobotConfig
from scale_bench.skills.context import (
    GraspCandidate,
    GraspState,
    SceneSnapshot,
    SkillContext,
)
from scale_bench.skills.errors import SkillError
from scale_bench.skills.models import Arm


@dataclass(frozen=True, slots=True)
class SingleCandidateSkillContext:
    """Expose exactly one collected candidate while delegating live geometry."""

    context: SkillContext
    object_name: str
    arm: Arm
    candidate: GraspCandidate

    def snapshot(self) -> SceneSnapshot:
        return self.context.snapshot()

    def grasp_candidates(
        self,
        object_name: str,
        arm: Arm,
    ) -> tuple[GraspCandidate, ...]:
        if object_name != self.object_name or arm != self.arm:
            raise SkillError(
                "collected grasp candidate is bound to "
                f"{self.object_name!r}/{self.arm}, got {object_name!r}/{arm}"
            )
        return (self.candidate,)

    def measure_grasp(self, object_name: str, arm: Arm) -> GraspState:
        return self.context.measure_grasp(object_name, arm)


def grasp_annotation_path(object_usd_path: Path) -> Path:
    """Place the runtime-compatible grasp catalog beside its object USD."""

    return object_usd_path.with_name(f"{object_usd_path.stem}_grasps.yml")


def append_physics_validated_grasps(
    path: Path,
    object_name: str,
    robot_config: RobotConfig,
    candidates: tuple[GraspCandidate, ...],
) -> GraspCatalogConfig:
    """Validate and atomically append successful candidates to one catalog."""

    if not candidates:
        raise ValueError("at least one successful grasp candidate is required")
    approach_distance_m = candidates[0].approach_distance_m
    if any(
        not math.isclose(
            candidate.approach_distance_m,
            approach_distance_m,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
        for candidate in candidates[1:]
    ):
        raise ValueError("successful candidates use different approach distances")

    previous_candidates: tuple[GraspCandidateConfig, ...] = ()
    if path.exists():
        existing = _load_grasp_catalog(path)
        _validate_existing_catalog(
            existing,
            path,
            object_name,
            robot_config,
            approach_distance_m,
        )
        previous_candidates = existing.objects[object_name]

    next_candidate_id = (
        max(candidate.candidate_id for candidate in previous_candidates) + 1
        if previous_candidates
        else 0
    )
    appended_candidates = tuple(
        GraspCandidateConfig(
            candidate_id=next_candidate_id + index,
            position_object_m=candidate.tcp_pose_object.position_m,
            orientation_object_xyzw=candidate.tcp_pose_object.orientation_xyzw,
            approach_axis_tcp=candidate.approach_axis_tcp,
            score=candidate.score,
        )
        for index, candidate in enumerate(candidates)
    )
    tcp = robot_config.kinematics.tcp
    catalog = GraspCatalogConfig(
        robot_name=robot_config.name,
        tcp_parent_frame=tcp.parent_frame,
        tcp_position_m=tcp.position_m,
        tcp_orientation_xyzw=tcp.orientation_xyzw,
        approach_distance_m=approach_distance_m,
        objects={object_name: (*previous_candidates, *appended_candidates)},
    )
    _write_grasp_catalog(path, catalog)
    return catalog


def _load_grasp_catalog(path: Path) -> GraspCatalogConfig:
    try:
        with path.open(encoding="utf-8") as stream:
            return GraspCatalogConfig.model_validate(yaml.safe_load(stream))
    except (OSError, yaml.YAMLError, ValidationError) as error:
        raise ValueError(f"could not load grasp annotation {path}: {error}") from error


def _validate_existing_catalog(
    catalog: GraspCatalogConfig,
    path: Path,
    object_name: str,
    robot_config: RobotConfig,
    approach_distance_m: float,
) -> None:
    tcp = robot_config.kinematics.tcp
    if (
        catalog.robot_name != robot_config.name
        or catalog.tcp_parent_frame != tcp.parent_frame
        or catalog.tcp_position_m != tcp.position_m
        or catalog.tcp_orientation_xyzw != tcp.orientation_xyzw
    ):
        raise ValueError(f"grasp annotation robot/TCP does not match: {path}")
    if not math.isclose(
        catalog.approach_distance_m,
        approach_distance_m,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError(f"grasp annotation approach distance does not match: {path}")
    if set(catalog.objects) != {object_name}:
        raise ValueError(
            f"grasp annotation must contain only object {object_name!r}: {path}"
        )


def _write_grasp_catalog(path: Path, catalog: GraspCatalogConfig) -> None:
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    try:
        with temporary_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(
                catalog.model_dump(mode="json"),
                stream,
                sort_keys=False,
            )
        os.replace(temporary_path, path)
    except OSError:
        temporary_path.unlink(missing_ok=True)
        raise


__all__ = [
    "SingleCandidateSkillContext",
    "append_physics_validated_grasps",
    "grasp_annotation_path",
]
