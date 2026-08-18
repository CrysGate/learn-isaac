"""Assemble an isolated dynamic gripper from a complete robot USD."""

from __future__ import annotations

from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

from grasp_data_gen.config import GraspGenerationConfig
from grasp_data_gen.isaac.geometry import Pose, compose_pose
from grasp_data_gen.isaac.gripper import DynamicGripper
from grasp_data_gen.isaac.usd import (
    configure_dynamic_body,
    reference_asset,
    reference_materials,
    set_local_pose,
)
from scale_bench.config.models.robot import RobotConfig


GRIPPER_PATH = "/World/EvaluationGripper"


def _relative_source_pose(
    source_stage: Usd.Stage,
    base_path: Sdf.Path,
    link_path: Sdf.Path,
) -> Pose:
    cache = UsdGeom.XformCache()
    base_world = cache.GetLocalToWorldTransform(source_stage.GetPrimAtPath(base_path))
    link_world = cache.GetLocalToWorldTransform(source_stage.GetPrimAtPath(link_path))
    base_to_link = (link_world * base_world.GetInverse()).RemoveScaleShear()
    return base_to_link.ExtractTranslation(), base_to_link.ExtractRotationQuat()


def _configured_frame_pose(
    source_stage: Usd.Stage,
    base_path: Sdf.Path,
    frame_path: Sdf.Path,
    position_m: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
) -> Pose:
    if not source_stage.GetPrimAtPath(frame_path).IsValid():
        raise RuntimeError(f"robot USD is missing configured frame prim: {frame_path}")
    x, y, z, w = orientation_xyzw
    offset = Gf.Vec3d(*position_m), Gf.Quatd(w, Gf.Vec3d(x, y, z))
    return compose_pose(
        _relative_source_pose(source_stage, base_path, frame_path),
        offset,
    )


def _base_to_tcp_pose(
    source_stage: Usd.Stage,
    generation: GraspGenerationConfig,
    robot: RobotConfig,
) -> Pose:
    source_root = Sdf.Path(generation.source_root_prim)
    tcp = robot.kinematics.tcp
    return _configured_frame_pose(
        source_stage,
        source_root.AppendPath(generation.base_link),
        source_root.AppendPath(generation.tcp_prim_path),
        tcp.position_m,
        tcp.orientation_xyzw,
    )


def _base_to_camera_pose(
    source_stage: Usd.Stage,
    generation: GraspGenerationConfig,
    robot: RobotConfig,
) -> Pose | None:
    camera = robot.camera
    if camera is None:
        return None
    source_root = Sdf.Path(generation.source_root_prim)
    return _configured_frame_pose(
        source_stage,
        source_root.AppendPath(generation.base_link),
        source_root.AppendPath(camera.parent_prim_path),
        camera.position_m,
        camera.orientation_xyzw,
    )


def _create_articulation_root(stage: Usd.Stage, base_link: str) -> None:
    root_joint = UsdPhysics.FixedJoint.Define(stage, f"{GRIPPER_PATH}/root_joint")
    root_joint.CreateBody1Rel().SetTargets(
        [Sdf.Path(f"{GRIPPER_PATH}/{base_link}")]
    )
    root_joint.CreateLocalPos0Attr(Gf.Vec3f(0.0))
    root_joint.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    root_joint.CreateLocalRot0Attr(Gf.Quatf(1.0))
    root_joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
    root_joint.CreateCollisionEnabledAttr(False)
    UsdPhysics.ArticulationRootAPI.Apply(root_joint.GetPrim())
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(root_joint.GetPrim())
    articulation.CreateEnabledSelfCollisionsAttr(False)
    articulation.CreateSolverPositionIterationCountAttr(64)
    articulation.CreateSolverVelocityIterationCountAttr(4)
    articulation.CreateSleepThresholdAttr(5.0e-5)
    articulation.CreateStabilizationThresholdAttr(1.0e-5)


def _copy_joints(
    stage: Usd.Stage,
    source_stage: Usd.Stage,
    generation: GraspGenerationConfig,
    robot: RobotConfig,
) -> dict[str, Usd.Prim]:
    source_root = generation.source_root_prim.rstrip("/")
    joint_scope = generation.joint_scope
    stage.DefinePrim(f"{GRIPPER_PATH}/{joint_scope}", "Scope")
    source_layer = source_stage.Flatten()
    joints = {}
    for joint_name in robot.gripper.joint_names:
        source = Sdf.Path(source_root).AppendPath(f"{joint_scope}/{joint_name}")
        destination = Sdf.Path(f"{GRIPPER_PATH}/{joint_scope}/{joint_name}")
        if not source_stage.GetPrimAtPath(source).IsValid():
            raise RuntimeError(f"robot USD is missing gripper joint: {source}")
        if not Sdf.CopySpec(
            source_layer,
            source,
            stage.GetRootLayer(),
            destination,
        ):
            raise RuntimeError(f"failed to copy gripper joint: {source}")
        joint = stage.GetPrimAtPath(destination)
        for relationship in joint.GetRelationships():
            remapped = []
            changed = False
            for target in relationship.GetTargets():
                target_text = str(target)
                if target_text == source_root or target_text.startswith(
                    source_root + "/"
                ):
                    target = Sdf.Path(
                        GRIPPER_PATH + target_text[len(source_root) :]
                    )
                    changed = True
                remapped.append(target)
            if changed:
                relationship.SetTargets(remapped)
        joints[joint_name] = joint
    return joints


def assemble_gripper(
    stage: Usd.Stage,
    robot_usd: Path,
    generation: GraspGenerationConfig,
    robot: RobotConfig,
) -> DynamicGripper:
    """Assemble configured gripper links and joints from a complete robot USD."""

    source_stage = Usd.Stage.Open(str(robot_usd))

    root = UsdGeom.Xform.Define(stage, GRIPPER_PATH).GetPrim()
    set_local_pose(root, Gf.Vec3d(0.0, 0.0, 0.5))
    source_root = Sdf.Path(generation.source_root_prim)
    base_source_path = source_root.AppendPath(generation.base_link)
    links = {}
    for link_name in generation.link_names:
        source_path = source_root.AppendPath(link_name)
        link = reference_asset(
            stage,
            f"{GRIPPER_PATH}/{link_name}",
            robot_usd,
            source_path,
        )
        set_local_pose(
            link,
            *_relative_source_pose(source_stage, base_source_path, source_path),
        )
        configure_dynamic_body(link)
        if not any(
            prim.HasAPI(UsdPhysics.CollisionAPI)
            for prim in Usd.PrimRange(link, Usd.TraverseInstanceProxies())
        ):
            raise RuntimeError(
                f"configured link has no collision geometry: {link.GetPath()}"
            )
        links[link_name] = link

    _create_articulation_root(stage, generation.base_link)
    joints = _copy_joints(stage, source_stage, generation, robot)
    base_to_tcp = _base_to_tcp_pose(source_stage, generation, robot)
    base_to_camera = _base_to_camera_pose(source_stage, generation, robot)
    tcp_prim = UsdGeom.Xform.Define(stage, f"{GRIPPER_PATH}/tcp").GetPrim()
    set_local_pose(tcp_prim, *base_to_tcp)
    reference_materials(
        stage,
        robot_usd,
        source_root,
        generation.material_prim_paths,
    )

    return DynamicGripper(
        stage=stage,
        root_prim=root,
        base_prim=links[generation.base_link],
        link_prims=links,
        joint_prims=joints,
        base_to_tcp=base_to_tcp,
        base_to_camera=base_to_camera,
        robot=robot,
    )
