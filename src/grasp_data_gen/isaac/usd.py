"""Shared USD prim, asset-reference, and transform operations."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

POSE_OP_SUFFIX = "graspDataGenPose"


def new_stage(world_path: str = "/World") -> Usd.Stage:
    """Create a metre-scaled Z-up stage with a default world prim."""

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Isaac Sim did not create a USD stage")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, world_path).GetPrim()
    stage.SetDefaultPrim(world)
    return stage


def asset_default_prim_path(asset_path: Path) -> Sdf.Path:
    """Return the validated default prim path of a USD asset."""

    asset_stage = Usd.Stage.Open(str(asset_path))
    if asset_stage is None:
        raise RuntimeError(f"could not open USD asset: {asset_path}")
    default_prim = asset_stage.GetDefaultPrim()
    if not default_prim:
        raise RuntimeError(f"USD asset has no valid default prim: {asset_path}")
    return default_prim.GetPath()


def reference_asset(
    stage: Usd.Stage,
    destination_path: str,
    asset_path: Path,
    source_path: str | Sdf.Path | None = None,
) -> Usd.Prim:
    """Reference one asset prim under a newly defined Xform prim."""

    source = (
        asset_default_prim_path(asset_path)
        if source_path is None
        else Sdf.Path(source_path)
    )
    prim = stage.DefinePrim(destination_path, "Xform")
    if not prim.GetReferences().AddReference(str(asset_path), source):
        raise RuntimeError(
            f"failed to reference {asset_path}[{source}] at {destination_path}"
        )
    return prim


def reference_materials(
    stage: Usd.Stage,
    asset_path: Path,
    source_root: Sdf.Path,
    relative_paths: Iterable[str],
) -> None:
    """Restore absolute material bindings retained by referenced robot links."""

    for relative_path in relative_paths:
        material_path = source_root.AppendPath(relative_path)
        material = stage.DefinePrim(material_path, "Scope")
        if not material.GetReferences().AddReference(str(asset_path), material_path):
            raise RuntimeError(f"failed to reference material: {material_path}")


def set_local_pose(
    prim: Usd.Prim,
    position: Gf.Vec3d = Gf.Vec3d(0.0),
    orientation: Gf.Quatd = Gf.Quatd(1.0),
) -> None:
    """Replace a prim's local transform with translate and orient operations."""

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp(
        UsdGeom.XformOp.PrecisionDouble,
        POSE_OP_SUFFIX,
    ).Set(position)
    xformable.AddOrientOp(
        UsdGeom.XformOp.PrecisionDouble,
        POSE_OP_SUFFIX,
    ).Set(orientation)


def configure_dynamic_body(prim: Usd.Prim) -> None:
    """Configure a gravity-disabled rigid body for grasp evaluation."""

    rigid_body = UsdPhysics.RigidBodyAPI(prim)
    if not rigid_body:
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)
    physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    physx_body.CreateDisableGravityAttr(True)
    physx_body.CreateEnableCCDAttr(True)
