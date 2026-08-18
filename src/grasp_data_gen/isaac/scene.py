"""Build and advance the isolated grasp evaluation scene."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import omni.physx
from isaacsim.replicator.grasping.grasping_manager import GraspingManager
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics

from grasp_data_gen.config import GraspGenerationConfig
from grasp_data_gen.isaac.gripper import DynamicGripper
from grasp_data_gen.isaac.gripper_assembly import assemble_gripper
from grasp_data_gen.isaac.usd import (
    configure_dynamic_body,
    new_stage,
    reference_asset,
    set_local_pose,
)
from scale_bench.config.models.robot import RobotConfig


OBJECT_PATH = "/World/Object"
PHYSICS_SCENE_PATH = "/World/PhysicsScene"


@dataclass(frozen=True)
class EvaluationScene:
    stage: Usd.Stage
    object_prim: Usd.Prim
    gripper: DynamicGripper


def build_evaluation_scene(
    robot_usd: Path,
    object_usd: Path,
    generation: GraspGenerationConfig,
    robot: RobotConfig,
) -> EvaluationScene:
    """Create the physics scene and assemble its isolated gripper."""

    stage = new_stage()
    physics_scene = UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    physx_scene.CreateEnableCCDAttr(True)
    physx_scene.CreateEnableStabilizationAttr(True)

    object_prim = reference_asset(stage, OBJECT_PATH, object_usd)
    set_local_pose(object_prim)
    configure_dynamic_body(object_prim)
    gripper = assemble_gripper(stage, robot_usd, generation, robot)
    return EvaluationScene(stage=stage, object_prim=object_prim, gripper=gripper)


def export_stage(scene: EvaluationScene, path: Path) -> None:
    if not scene.stage.GetRootLayer().Export(str(path)):
        raise RuntimeError(f"failed to export dynamic evaluation stage: {path}")


def create_grasping_manager(scene: EvaluationScene) -> GraspingManager:
    """Bind Isaac Sim's grasping manager to the evaluation scene actors."""

    manager = GraspingManager()
    if not manager.set_gripper(scene.gripper.root_prim):
        raise RuntimeError(
            f"failed to set dynamic gripper: {scene.gripper.root_prim.GetPath()}"
        )
    manager.set_object_prim_path(OBJECT_PATH)
    return manager


def gripper_overlaps_object(scene: EvaluationScene) -> bool:
    """Return whether any gripper collision shape overlaps the object."""

    query = omni.physx.get_physx_scene_query_interface()
    object_path = str(scene.object_prim.GetPath())

    for link in scene.gripper.link_prims.values():
        for prim in Usd.PrimRange(link, Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue

            overlaps_object = False

            def report_hit(hit: Any) -> bool:
                nonlocal overlaps_object
                overlaps_object = hit.rigid_body == object_path
                return not overlaps_object

            path_0, path_1 = PhysicsSchemaTools.encodeSdfPath(prim.GetPath())
            query.overlap_shape(path_0, path_1, report_hit)
            if overlaps_object:
                return True

    return False


def step_physics(
    simulation_app: Any,
    physics_dt: float,
    num_frames: int,
) -> None:
    """Advance PhysX deterministically and publish the resulting stage state."""

    interface = omni.physx.get_physx_simulation_interface()
    for _ in range(num_frames):
        interface.simulate(physics_dt, 0)
        interface.fetch_results()
    simulation_app.update()
