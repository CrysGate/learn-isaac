"""Physically validate geometrically feasible grasp candidates."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, cast

import omni.physx
from isaacsim.replicator.grasping.grasping_manager import GraspingManager
from pxr import Gf, PhysxSchema, UsdPhysics

from grasp_data_gen.config import GraspPhysicsValidationConfig
from grasp_data_gen.generation import CandidateBatch
from grasp_data_gen.isaac.geometry import Pose, to_pose_data
from grasp_data_gen.isaac.prim_poses import set_world_pose, world_pose
from grasp_data_gen.isaac.scene import (
    EvaluationScene,
    gripper_overlaps_object,
    step_physics,
)
from grasp_data_gen.models import CandidateRecord, PhysicsEvaluation


@dataclass(frozen=True)
class EvaluationBatch:
    records: tuple[CandidateRecord, ...]
    successful: tuple[CandidateRecord, ...]


def evaluate_candidate(
    manager: GraspingManager,
    scene: EvaluationScene,
    object_world_pose: Pose,
    gripper_world_pose: Pose,
    validation: GraspPhysicsValidationConfig,
    simulation_app: Any,
) -> PhysicsEvaluation:
    """Close and hold one candidate, returning its measured result."""

    manager.clear_simulation(simulate_using_timeline=False)
    object_prim = scene.object_prim
    object_rigid_body = UsdPhysics.RigidBodyAPI(object_prim)
    set_world_pose(object_prim, object_world_pose)
    object_rigid_body.GetVelocityAttr().Set(Gf.Vec3f(0.0))
    object_rigid_body.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0))
    object_physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(object_prim)
    object_physx_body.CreateDisableGravityAttr(True).Set(True)
    manager.set_gripper_pose(*gripper_world_pose)

    gripper = scene.gripper
    gripper.open()
    omni.physx.get_physx_simulation_interface().flush_changes()
    step_physics(simulation_app, validation.physics_dt, 5)
    initial_overlap = gripper_overlaps_object(scene)
    open_joint_positions = gripper.joint_positions()
    open_aperture = gripper.aperture()

    gripper.close()
    step_physics(simulation_app, validation.physics_dt, validation.close_steps)
    joint_positions = gripper.joint_positions()
    command_openness = gripper.command_openness()
    residual_aperture = gripper.aperture()
    closure_travel = open_aperture - residual_aperture
    openness_spread = max(command_openness.values()) - min(
        command_openness.values()
    )
    object_pose_at_close = world_pose(object_prim)
    link_poses_base = {
        name: to_pose_data(pose)
        for name, pose in gripper.link_poses_base().items()
    }

    object_physx_body.GetDisableGravityAttr().Set(False)
    step_physics(simulation_app, validation.physics_dt, validation.hold_steps)
    object_pose_after_hold = world_pose(object_prim)
    hold_position_drift = float(
        (object_pose_at_close[0] - object_pose_after_hold[0]).GetLength()
    )
    relative = object_pose_at_close[1].GetInverse() * object_pose_after_hold[1]
    real = min(1.0, max(-1.0, abs(float(relative.GetReal()))))
    hold_orientation_drift = 2.0 * math.acos(real)

    failures = []
    if initial_overlap:
        failures.append("initial_gripper_object_overlap")
    if closure_travel < validation.minimum_closure_travel_m:
        failures.append("gripper_did_not_close")
    if residual_aperture < validation.minimum_residual_aperture_m:
        failures.append("gripper_closed_without_obstruction")
    if openness_spread > validation.maximum_command_openness_spread:
        failures.append("asymmetric_command_joint_closure")
    if hold_position_drift > validation.hold_position_tolerance_m:
        failures.append("object_slipped_during_gravity_hold")
    if hold_orientation_drift > validation.hold_orientation_tolerance_rad:
        failures.append("object_rotated_during_gravity_hold")

    balance_score = max(
        0.0,
        1.0 - openness_spread / validation.maximum_command_openness_spread,
    )
    position_score = math.exp(
        -hold_position_drift / validation.hold_position_tolerance_m
    )
    orientation_score = math.exp(
        -hold_orientation_drift / validation.hold_orientation_tolerance_rad
    )
    return PhysicsEvaluation(
        accepted=not failures,
        failures=tuple(failures),
        score=float(
            0.4 * balance_score
            + 0.4 * position_score
            + 0.2 * orientation_score
        ),
        open_joint_positions=open_joint_positions,
        open_aperture_m=float(open_aperture),
        joint_positions=joint_positions,
        command_openness=command_openness,
        residual_aperture_m=float(residual_aperture),
        closure_travel_m=float(closure_travel),
        command_openness_spread=float(openness_spread),
        hold_position_drift_m=hold_position_drift,
        hold_orientation_drift_rad=hold_orientation_drift,
        link_poses_base=link_poses_base,
    )


def evaluate_candidates(
    manager: GraspingManager,
    scene: EvaluationScene,
    batch: CandidateBatch,
    validation: GraspPhysicsValidationConfig,
    simulation_app: Any,
) -> EvaluationBatch:
    """Evaluate every feasible candidate and rank the accepted records."""

    records = list(batch.records)
    object_world_pose = world_pose(scene.object_prim)
    for ordinal, candidate in enumerate(batch.feasible, 1):
        record = records[candidate.record_index]
        print(
            f"EVALUATING candidate={ordinal}/{len(batch.feasible)} "
            f"id={record.candidate_id}",
            flush=True,
        )
        evaluation = evaluate_candidate(
            manager,
            scene,
            object_world_pose,
            candidate.world_pose,
            validation,
            simulation_app,
        )
        records[candidate.record_index] = record.model_copy(
            update={"evaluation": evaluation}
        )

    manager.clear_simulation(simulate_using_timeline=False)
    accepted = [
        record
        for record in records
        if record.evaluation is not None and record.evaluation.accepted
    ]
    successful = sorted(
        accepted,
        key=lambda record: cast(PhysicsEvaluation, record.evaluation).score,
        reverse=True,
    )
    return EvaluationBatch(records=tuple(records), successful=tuple(successful))
