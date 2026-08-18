"""Read and write simulated prim poses through Isaac's grasping adapter."""

from __future__ import annotations

from isaacsim.replicator.grasping import transform_utils
from pxr import Gf, Usd

from grasp_data_gen.isaac.geometry import Pose


def world_pose(prim: Usd.Prim) -> Pose:
    """Read a simulated prim pose in world coordinates."""

    pose = transform_utils.get_prim_world_pose(prim)
    if pose is None:
        raise RuntimeError(f"could not read world pose for {prim.GetPath()}")
    return pose


def set_world_pose(prim: Usd.Prim, pose: Pose) -> None:
    """Set a simulated prim pose in world coordinates."""

    transform_utils.set_transform_attributes(
        prim,
        location=pose[0],
        orientation=pose[1],
    )


def relative_world_pose(parent: Usd.Prim, child: Usd.Prim) -> Pose:
    """Express a simulated child's world pose in its parent's frame."""

    parent_position, parent_orientation = world_pose(parent)
    child_position, child_orientation = world_pose(child)
    inverse = parent_orientation.GetInverse()
    return (
        Gf.Rotation(inverse).TransformDir(child_position - parent_position),
        (inverse * child_orientation).GetNormalized(),
    )
