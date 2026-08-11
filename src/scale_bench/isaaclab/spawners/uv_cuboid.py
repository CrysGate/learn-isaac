"""Isaac Lab cuboid spawner with face-varying texture coordinates."""

from __future__ import annotations

from collections.abc import Callable

import isaaclab.sim as sim_utils
from isaaclab.utils.configclass import configclass
from pxr import Gf, Sdf, Usd, UsdGeom


def spawn_uv_cuboid(
    prim_path: str,
    cfg: UvCuboidCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Spawn an Isaac Lab cuboid, then add one UV island to each face."""

    if len(cfg.uv_scale) != 2 or any(scale <= 0.0 for scale in cfg.uv_scale):
        raise ValueError(
            f"UV scale must contain two positive values, got {cfg.uv_scale}."
        )

    root_prim = sim_utils.spawn_cuboid(
        prim_path,
        cfg,
        translation=translation,
        orientation=orientation,
        **kwargs,
    )

    stage = sim_utils.get_current_stage()
    spawned_paths = sim_utils.find_matching_prim_paths(prim_path)
    if not spawned_paths:
        raise RuntimeError(f"No cuboids were spawned for path: '{prim_path}'.")

    for spawned_path in spawned_paths:
        cube_prim = stage.GetPrimAtPath(f"{spawned_path}/geometry/mesh")
        if not cube_prim.IsA(UsdGeom.Cube):
            raise RuntimeError(
                f"Expected a UsdGeom.Cube at '{cube_prim.GetPath()}', "
                f"got '{cube_prim.GetTypeName()}'."
            )
        _author_cube_uvs(cube_prim, cfg.uv_scale)

    return root_prim


def _author_cube_uvs(cube_prim: Usd.Prim, uv_scale: tuple[float, float]) -> None:
    """Author the 24 face-varying UV values required by ``UsdGeom.Cube``."""

    uv_u, uv_v = uv_scale
    face_uvs = [
        Gf.Vec2f(0.0, 0.0),
        Gf.Vec2f(uv_u, 0.0),
        Gf.Vec2f(uv_u, uv_v),
        Gf.Vec2f(0.0, uv_v),
    ]
    uv_primvar = UsdGeom.PrimvarsAPI(cube_prim).CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.faceVarying,
    )
    uv_primvar.Set(face_uvs * 6)


@configclass
class UvCuboidCfg(sim_utils.CuboidCfg):
    """Cuboid configuration extended with face-varying ``st`` UVs."""

    func: Callable = spawn_uv_cuboid
    uv_scale: tuple[float, float] = (1.0, 1.0)


__all__ = ["UvCuboidCfg", "spawn_uv_cuboid"]
