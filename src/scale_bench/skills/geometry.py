"""Geometry utilities for skill-space poses and coordinate transforms."""

from __future__ import annotations

import math

from .models import Pose


def approach_start_pose(
    contact_tcp_pose_env: Pose,
    approach_axis_tcp: tuple[float, float, float],
    approach_distance_m: float,
) -> Pose:
    """Return the TCP pose before contact along its approach axis."""

    approach_axis_env = rotate_vector_xyzw(
        contact_tcp_pose_env.orientation_xyzw,
        approach_axis_tcp,
    )
    return Pose(
        tuple(
            contact_coordinate_env_m - approach_distance_m * approach_component_env
            for contact_coordinate_env_m, approach_component_env in zip(
                contact_tcp_pose_env.position_m,
                approach_axis_env,
                strict=True,
            )
        ),
        contact_tcp_pose_env.orientation_xyzw,
    )


def compose_pose(middle_pose_parent: Pose, child_pose_middle: Pose) -> Pose:
    """Return the child pose in parent from parent<-middle<-child."""

    child_position_offset_parent_m = rotate_vector_xyzw(
        middle_pose_parent.orientation_xyzw,
        child_pose_middle.position_m,
    )
    return Pose(
        tuple(
            middle_coordinate_parent_m + child_offset_parent_m
            for middle_coordinate_parent_m, child_offset_parent_m in zip(
                middle_pose_parent.position_m,
                child_position_offset_parent_m,
                strict=True,
            )
        ),
        normalize_quaternion_xyzw(
            multiply_quaternions_xyzw(
                middle_pose_parent.orientation_xyzw,
                child_pose_middle.orientation_xyzw,
            )
        ),
    )


def inverse_pose(frame_pose_parent: Pose) -> Pose:
    """Return the parent pose expressed in the frame coordinates."""

    parent_orientation_frame_xyzw = conjugate_quaternion_xyzw(
        frame_pose_parent.orientation_xyzw
    )
    parent_position_frame_m = rotate_vector_xyzw(
        parent_orientation_frame_xyzw,
        tuple(
            -coordinate_parent_m
            for coordinate_parent_m in frame_pose_parent.position_m
        ),
    )
    return Pose(parent_position_frame_m, parent_orientation_frame_xyzw)


def relative_pose(frame_pose_parent: Pose, child_pose_parent: Pose) -> Pose:
    """Return the child pose expressed in the frame coordinates."""

    return compose_pose(inverse_pose(frame_pose_parent), child_pose_parent)


def offset_z_env(
    position_env_m: tuple[float, float, float],
    offset_m: float,
) -> tuple[float, float, float]:
    """Offset an environment-frame position along the environment Z axis."""

    return (
        position_env_m[0],
        position_env_m[1],
        position_env_m[2] + offset_m,
    )


def multiply_quaternions_xyzw(
    left_orientation_xyzw: tuple[float, float, float, float],
    right_orientation_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Return the Hamilton product of two XYZW quaternions."""

    ax, ay, az, aw = left_orientation_xyzw
    bx, by, bz, bw = right_orientation_xyzw
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def conjugate_quaternion_xyzw(
    orientation_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Return the conjugate of an XYZW quaternion."""

    return (
        -orientation_xyzw[0],
        -orientation_xyzw[1],
        -orientation_xyzw[2],
        orientation_xyzw[3],
    )


def normalize_quaternion_xyzw(
    orientation_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Return a unit-length XYZW quaternion."""

    norm = math.sqrt(sum(component * component for component in orientation_xyzw))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("cannot normalize a non-finite or zero quaternion")
    return tuple(component / norm for component in orientation_xyzw)


def quaternion_xyzw_from_axis_angle(
    rotation_axis_frame: tuple[float, float, float],
    rotation_angle_rad: float,
) -> tuple[float, float, float, float]:
    """Return an XYZW quaternion for a rotation around a unit frame axis."""

    half_angle_rad = rotation_angle_rad / 2.0
    imaginary_scale = math.sin(half_angle_rad)
    return normalize_quaternion_xyzw(
        (
            rotation_axis_frame[0] * imaginary_scale,
            rotation_axis_frame[1] * imaginary_scale,
            rotation_axis_frame[2] * imaginary_scale,
            math.cos(half_angle_rad),
        )
    )


def quaternion_xyzw_from_rpy(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float, float]:
    """Return an XYZW quaternion for fixed-axis roll, pitch, then yaw."""

    half_roll_rad = roll_rad / 2.0
    half_pitch_rad = pitch_rad / 2.0
    half_yaw_rad = yaw_rad / 2.0
    sin_roll, cos_roll = math.sin(half_roll_rad), math.cos(half_roll_rad)
    sin_pitch, cos_pitch = math.sin(half_pitch_rad), math.cos(half_pitch_rad)
    sin_yaw, cos_yaw = math.sin(half_yaw_rad), math.cos(half_yaw_rad)
    return normalize_quaternion_xyzw(
        (
            sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
            cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
            cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
            cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
        )
    )


def quaternion_angular_distance_rad(
    left_orientation_xyzw: tuple[float, float, float, float],
    right_orientation_xyzw: tuple[float, float, float, float],
) -> float:
    """Return the shortest angular distance between two unit orientations."""

    dot = abs(
        sum(
            left_component * right_component
            for left_component, right_component in zip(
                left_orientation_xyzw,
                right_orientation_xyzw,
                strict=True,
            )
        )
    )
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def rotate_vector_xyzw(
    frame_orientation_parent_xyzw: tuple[float, float, float, float],
    vector_frame: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rotate a vector from frame coordinates into parent coordinates."""

    parent_orientation_frame_xyzw = conjugate_quaternion_xyzw(
        frame_orientation_parent_xyzw
    )
    vector_parent_xyzw = multiply_quaternions_xyzw(
        multiply_quaternions_xyzw(
            frame_orientation_parent_xyzw,
            (vector_frame[0], vector_frame[1], vector_frame[2], 0.0),
        ),
        parent_orientation_frame_xyzw,
    )
    return vector_parent_xyzw[:3]


__all__ = [
    "approach_start_pose",
    "compose_pose",
    "conjugate_quaternion_xyzw",
    "inverse_pose",
    "multiply_quaternions_xyzw",
    "normalize_quaternion_xyzw",
    "offset_z_env",
    "quaternion_angular_distance_rad",
    "quaternion_xyzw_from_axis_angle",
    "quaternion_xyzw_from_rpy",
    "relative_pose",
    "rotate_vector_xyzw",
]
