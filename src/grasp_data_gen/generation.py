"""Generate and task-filter geometric grasp candidates."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from isaacsim.replicator.grasping.grasping_manager import GraspingManager
from pxr import Gf

from grasp_data_gen.config import GraspGenerationConfig, GraspSamplerConfig
from grasp_data_gen.isaac.geometry import Pose, compose_pose, to_pose_data
from grasp_data_gen.isaac.scene import EvaluationScene, step_physics
from grasp_data_gen.models import CandidateRecord
from grasp_data_gen.tabletop import (
    evaluate_tabletop_candidate,
    measure_tabletop_geometry,
)


@dataclass(frozen=True)
class FeasibleCandidate:
    record_index: int
    world_pose: Pose


@dataclass(frozen=True)
class CandidateBatch:
    records: tuple[CandidateRecord, ...]
    feasible: tuple[FeasibleCandidate, ...]
    support_height_m: float


def build_sampler_config(
    scene: EvaluationScene,
    sampler: GraspSamplerConfig,
    *,
    num_candidates: int,
    num_orientations: int,
    random_seed: int,
) -> dict[str, Any]:
    """Translate typed project config into Isaac's sampler dictionary."""

    standoff_m = float(
        Gf.Dot(
            scene.gripper.base_to_tcp[0],
            Gf.Vec3d(*sampler.standoff_axis_base),
        )
    )
    if standoff_m <= 0.0:
        raise RuntimeError(
            f"derived gripper fingertip standoff is invalid: {standoff_m:.6f} m"
        )
    return {
        "sampler_type": sampler.sampler_type,
        "num_candidates": num_candidates,
        "num_orientations": num_orientations,
        "gripper_maximum_aperture": scene.gripper.robot.gripper.max_aperture_m,
        "gripper_standoff_fingertips": standoff_m,
        "gripper_approach_direction": sampler.gripper_approach_direction,
        "grasp_align_axis": sampler.grasp_align_axis,
        "orientation_sample_axis": sampler.orientation_sample_axis,
        "lateral_sigma": sampler.lateral_sigma,
        "random_seed": random_seed,
        "verbose": False,
    }


def generate_candidates(
    scene: EvaluationScene,
    manager: GraspingManager,
    generation: GraspGenerationConfig,
    simulation_app: Any,
    *,
    num_candidates: int,
    num_orientations: int,
    random_seed: int,
) -> CandidateBatch:
    """Sample object-local poses and retain those feasible above the support."""

    step_physics(simulation_app, generation.validation.physics_dt, 3)
    tabletop_geometry = measure_tabletop_geometry(
        scene,
        generation.tabletop_filter,
    )
    manager.sampler_config = build_sampler_config(
        scene,
        generation.sampler,
        num_candidates=num_candidates,
        num_orientations=num_orientations,
        random_seed=random_seed,
    )
    if not manager.generate_grasp_poses():
        raise RuntimeError("Isaac Sim did not generate any grasp poses")

    local_poses = manager.get_grasp_poses(in_world_frame=False)
    world_poses = manager.get_grasp_poses(in_world_frame=True)
    records = []
    feasible = []
    for index, (local_pose, world_pose) in enumerate(
        zip(local_poses, world_poses, strict=True)
    ):
        tcp_pose = compose_pose(local_pose, scene.gripper.base_to_tcp)
        task_filter = evaluate_tabletop_candidate(
            local_pose,
            tcp_pose,
            tabletop_geometry,
            generation.tabletop_filter,
            generation.sampler.approach_axis_tcp,
        )
        records.append(
            CandidateRecord(
                candidate_id=index,
                tcp_pose_object=to_pose_data(tcp_pose),
                approach_axis_tcp=generation.sampler.approach_axis_tcp,
                base_pose_object=to_pose_data(local_pose),
                task_filter=task_filter,
            )
        )
        if task_filter.accepted:
            feasible.append(
                FeasibleCandidate(record_index=index, world_pose=world_pose)
            )

    return CandidateBatch(
        records=tuple(records),
        feasible=tuple(feasible),
        support_height_m=tabletop_geometry.support_height_m,
    )
