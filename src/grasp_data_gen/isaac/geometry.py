"""Pose math and conversion at the PXR/model boundary."""

from __future__ import annotations

from pxr import Gf
from grasp_data_gen.models import PoseData


Pose = tuple[Gf.Vec3d, Gf.Quatd]


def compose_pose(parent_to_frame: Pose, frame_to_child: Pose) -> Pose:
    """Compose two rigid poses expressed as position/quaternion pairs."""

    parent_position, parent_orientation = parent_to_frame
    child_position, child_orientation = frame_to_child
    position = parent_position + Gf.Rotation(parent_orientation).TransformDir(child_position)
    orientation = (parent_orientation * child_orientation).GetNormalized()
    return position, orientation


def base_pose_from_tcp_pose(tcp_pose: Pose, base_to_tcp: Pose) -> Pose:
    """Recover a base pose from a TCP pose and the fixed base-to-TCP pose."""

    tcp_position, tcp_orientation = tcp_pose
    base_to_tcp_position, base_to_tcp_orientation = base_to_tcp
    base_orientation = (
        tcp_orientation * base_to_tcp_orientation.GetInverse()
    ).GetNormalized()
    base_position = tcp_position - Gf.Rotation(base_orientation).TransformDir(
        base_to_tcp_position
    )
    return base_position, base_orientation


def to_pose_data(pose: Pose) -> PoseData:
    """Convert a PXR pose to the serializable result schema."""

    position, orientation = pose
    imaginary = orientation.GetImaginary()
    return PoseData(
        position_m=tuple(float(value) for value in position),
        orientation_xyzw=(
            float(imaginary[0]),
            float(imaginary[1]),
            float(imaginary[2]),
            float(orientation.GetReal()),
        ),
    )


def from_pose_data(pose: PoseData) -> Pose:
    """Convert a serialized result pose to normalized PXR values."""

    x, y, z, w = pose.orientation_xyzw
    orientation = Gf.Quatd(w, Gf.Vec3d(x, y, z)).GetNormalized()
    return Gf.Vec3d(*pose.position_m), orientation
