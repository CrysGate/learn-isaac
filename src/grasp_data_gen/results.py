"""Build output schemas and atomically write generated grasp YAML."""

from __future__ import annotations

from collections import Counter
import os
from pathlib import Path
from typing import Any, cast
import yaml

from grasp_data_gen.config import GraspGenerationConfig
from grasp_data_gen.models import (
    CandidateRecord,
    GenerationMetadata,
    GraspFileData,
    PhysicsEvaluation,
    PoseData,
    ReportFileData,
    SuccessfulGrasp,
)
from scale_bench.config.models.robot import RobotConfig


def load_grasp_file(path: Path) -> GraspFileData:
    """Load and validate a generated successful-grasp file."""

    try:
        with path.open(encoding="utf-8") as stream:
            return GraspFileData.model_validate(yaml.safe_load(stream))
    except yaml.YAMLError as error:
        raise ValueError(f"invalid YAML in {path}: {error}") from error


def resolve_asset_path(
    stored_path: Path,
    grasp_file: Path,
    asset_name: str,
) -> Path:
    """Resolve metadata paths relative to their grasp file."""

    path = stored_path.expanduser()
    if not path.is_absolute():
        path = grasp_file.parent / path
    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"{asset_name} does not exist: {resolved}")
    return resolved


def build_metadata(
    *,
    generation: GraspGenerationConfig,
    robot: RobotConfig,
    generation_config_path: Path,
    robot_usd: Path,
    object_usd: Path,
    base_to_tcp: PoseData,
    sampler_config: dict[str, Any],
    records: tuple[CandidateRecord, ...],
    feasible_count: int,
    successful: tuple[CandidateRecord, ...],
    support_height: float,
) -> GenerationMetadata:
    """Build metadata shared by the diagnostic and accepted-grasp files."""

    task_rejections = Counter(
        reason
        for record in records
        for reason in record.task_filter.failures
    )
    physics_rejections = Counter(
        reason
        for record in records
        if record.evaluation is not None
        for reason in record.evaluation.failures
    )
    gripper = robot.gripper
    return GenerationMetadata(
        generation_config=generation_config_path,
        robot_profile=generation.robot_profile,
        robot_usd=robot_usd,
        object_usd=object_usd,
        candidate_frame="object",
        candidate_pose_frame=f"{robot.name}_tcp",
        quaternion_order="xyzw",
        gripper_definition={
            "source_root_prim": generation.source_root_prim,
            "base_link": generation.base_link,
            "link_names": generation.link_names,
            "joint_names": gripper.joint_names,
            "command_joint_names": gripper.command_joint_names,
            "material_prim_paths": generation.material_prim_paths,
        },
        tcp_definition={
            "parent_frame": robot.kinematics.tcp.parent_frame,
            "offset_in_parent_m": robot.kinematics.tcp.position_m,
            "base_to_tcp": base_to_tcp,
            "approach_axis_tcp": generation.sampler.approach_axis_tcp,
        },
        generated_count=len(records),
        task_feasible_count=feasible_count,
        task_rejected_count=len(records) - feasible_count,
        evaluated_count=feasible_count,
        accepted_count=len(successful),
        task_rejection_counts=dict(task_rejections),
        physics_rejection_counts=dict(physics_rejections),
        sampler_config={
            key: list(value) if isinstance(value, tuple) else value
            for key, value in sampler_config.items()
        },
        tabletop_filter={
            **generation.tabletop_filter.model_dump(),
            "object_support_height_m": support_height,
        },
        validation=generation.validation.model_dump(),
    )


def _write_yaml(path: Path, data: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
    os.replace(temporary, path)


def write_results(
    output_dir: Path,
    metadata: GenerationMetadata,
    records: tuple[CandidateRecord, ...],
    successful: tuple[CandidateRecord, ...],
) -> None:
    """Atomically write full diagnostics and accepted TCP poses."""

    metadata_data = metadata.model_dump()
    report = ReportFileData(
        **metadata_data,
        candidates=records,
    )
    grasps = tuple(
        SuccessfulGrasp(
            candidate_id=item.candidate_id,
            position_object_m=item.tcp_pose_object.position_m,
            orientation_object_xyzw=item.tcp_pose_object.orientation_xyzw,
            approach_axis_tcp=item.approach_axis_tcp,
            score=cast(PhysicsEvaluation, item.evaluation).score,
            task_filter=item.task_filter,
            evaluation=cast(PhysicsEvaluation, item.evaluation),
        )
        for item in successful
    )
    grasp_file = GraspFileData(**metadata_data, grasps=grasps)
    _write_yaml(output_dir / "report.yaml", report.model_dump(mode="json"))
    _write_yaml(output_dir / "successful_grasps.yaml", grasp_file.model_dump(mode="json"))
