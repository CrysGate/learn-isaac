"""Define typed runtime records and result files."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import (
    AfterValidator,
    BaseModel,
    ConfigDict,
    Field,
    FiniteFloat,
    model_validator,
)

def _nonzero_quaternion(values: QuaternionValues) -> QuaternionValues:
    if sum(value * value for value in values) <= 1.0e-18:
        raise ValueError("must be a nonzero quaternion")
    return values

class _Model(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


Vector3 = tuple[FiniteFloat, FiniteFloat, FiniteFloat]
QuaternionValues = tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]
Score = Annotated[FiniteFloat, Field(ge=0.0, le=1.0)]
Quaternion = Annotated[QuaternionValues, AfterValidator(_nonzero_quaternion)]


class PoseData(_Model):
    position_m: Vector3
    orientation_xyzw: Quaternion


class TaskFilterResult(_Model):
    accepted: bool
    failures: tuple[str, ...]
    approach_direction_object: Vector3
    approach_up_dot: FiniteFloat
    minimum_gripper_clearance_m: FiniteFloat
    camera_height_above_tcp_m: FiniteFloat | None = None
    camera_side_up_dot: FiniteFloat | None = None


class PhysicsEvaluation(_Model):
    accepted: bool
    failures: tuple[str, ...]
    score: Score
    open_joint_positions: dict[str, FiniteFloat]
    open_aperture_m: FiniteFloat
    joint_positions: dict[str, FiniteFloat]
    command_openness: dict[str, FiniteFloat]
    residual_aperture_m: FiniteFloat
    closure_travel_m: FiniteFloat
    command_openness_spread: FiniteFloat
    hold_position_drift_m: FiniteFloat
    hold_orientation_drift_rad: FiniteFloat
    link_poses_base: dict[str, PoseData]


class CandidateRecord(_Model):
    candidate_id: int = Field(ge=0)
    tcp_pose_object: PoseData
    approach_axis_tcp: Vector3
    base_pose_object: PoseData
    task_filter: TaskFilterResult
    evaluation: PhysicsEvaluation | None = None


class GripperDefinition(_Model):
    source_root_prim: str
    base_link: str
    link_names: tuple[str, ...] = Field(min_length=1)
    joint_names: tuple[str, ...] = ()
    command_joint_names: tuple[str, ...] = ()
    material_prim_paths: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_paths(self) -> "GripperDefinition":
        if not self.source_root_prim.startswith("/"):
            raise ValueError("source_root_prim must be an absolute prim path")
        paths = (*self.link_names, *self.material_prim_paths)
        if any(not path or path.startswith("/") for path in paths):
            raise ValueError("link and material prim paths must be relative")
        if len(set(self.link_names)) != len(self.link_names):
            raise ValueError("link_names must be unique")
        if self.base_link not in self.link_names:
            raise ValueError("link_names must include base_link")
        return self


class TcpDefinition(_Model):
    parent_frame: str
    offset_in_parent_m: Vector3
    # Generator files created before this field all used Piper's identity TCP orientation.
    tcp_orientation_parent_xyzw: Quaternion = (0.0, 0.0, 0.0, 1.0)
    base_to_tcp: PoseData
    approach_axis_tcp: Vector3


class GenerationMetadata(_Model):
    generation_config: Path
    robot_profile: Path
    robot_usd: Path
    object_usd: Path
    candidate_frame: Literal["object"]
    candidate_pose_frame: str
    quaternion_order: Literal["xyzw"]
    gripper_definition: GripperDefinition
    tcp_definition: TcpDefinition
    generated_count: int = Field(ge=0)
    task_feasible_count: int = Field(ge=0)
    task_rejected_count: int = Field(ge=0)
    evaluated_count: int = Field(ge=0)
    accepted_count: int = Field(ge=0)
    task_rejection_counts: dict[str, int]
    physics_rejection_counts: dict[str, int]
    sampler_config: dict[str, Any]
    tabletop_filter: dict[str, Any]
    validation: dict[str, Any]

    @model_validator(mode="after")
    def validate_counts(self) -> "GenerationMetadata":
        if self.generated_count != (
            self.task_feasible_count + self.task_rejected_count
        ):
            raise ValueError("generated count must equal feasible plus rejected")
        if self.evaluated_count != self.task_feasible_count:
            raise ValueError("all task-feasible candidates must be evaluated")
        if self.accepted_count > self.evaluated_count:
            raise ValueError("accepted count cannot exceed evaluated count")
        return self


class SuccessfulGrasp(_Model):
    candidate_id: int = Field(ge=0)
    position_object_m: Vector3
    orientation_object_xyzw: Quaternion
    approach_axis_tcp: Vector3
    score: Score
    task_filter: TaskFilterResult
    evaluation: PhysicsEvaluation

    @property
    def tcp_pose(self) -> PoseData:
        return PoseData(
            position_m=self.position_object_m,
            orientation_xyzw=self.orientation_object_xyzw,
        )

    @model_validator(mode="after")
    def validate_acceptance(self) -> "SuccessfulGrasp":
        if not self.task_filter.accepted:
            raise ValueError("task_filter.accepted must be true")
        if not self.evaluation.accepted:
            raise ValueError("evaluation.accepted must be true")
        if not math.isclose(
            self.score,
            self.evaluation.score,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise ValueError("score must match evaluation.score")
        return self


class ReportFileData(GenerationMetadata):
    candidates: tuple[CandidateRecord, ...]

    @model_validator(mode="after")
    def validate_candidates(self) -> "ReportFileData":
        if len(self.candidates) != self.generated_count:
            raise ValueError("generated_count must equal the number of candidates")
        return self


class GraspFileData(GenerationMetadata):
    grasps: tuple[SuccessfulGrasp, ...]

    @model_validator(mode="after")
    def validate_grasps(self) -> "GraspFileData":
        if self.accepted_count != len(self.grasps):
            raise ValueError("accepted_count must equal the number of grasps")
        candidate_ids = [grasp.candidate_id for grasp in self.grasps]
        if len(set(candidate_ids)) != len(candidate_ids):
            raise ValueError("candidate_id values must be unique")
        scores = [grasp.score for grasp in self.grasps]
        if any(left < right for left, right in zip(scores, scores[1:])):
            raise ValueError("grasps must be sorted by descending score")
        expected_links = set(self.gripper_definition.link_names)
        for rank, grasp in enumerate(self.grasps):
            if set(grasp.evaluation.link_poses_base) != expected_links:
                raise ValueError(
                    f"grasps[{rank}] link poses must exactly match link_names"
                )
        return self
