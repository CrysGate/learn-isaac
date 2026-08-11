"""Unit tests for simulator-independent configuration models."""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from scale_bench.config.models.camera import CameraConfig
from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.config.models.simulation import SimulationConfig


def _camera_data() -> dict:
    return {
        "model": "camera",
        "width": 640,
        "height": 480,
        "update_period_s": 1.0 / 30.0,
        "data_types": ["rgb", "distance_to_image_plane"],
        "focal_length_mm": 1.93,
        "intrinsic_source": "calibration",
        "intrinsic_matrix_px": [
            604.5,
            0.0,
            320.0,
            0.0,
            604.5,
            240.0,
            0.0,
            0.0,
            1.0,
        ],
        "distortion_model": "plumb_bob",
        "distortion_coefficients": [0.0] * 5,
        "clipping_range_m": [0.1, 10.0],
    }


def _robot_data() -> dict:
    return {
        "name": "robot",
        "usd_path": "robot.usd",
        "initial_joint_positions": {"arm": 0.0, "finger": 0.04},
        "kinematics": {
            "base_body": "base",
            "arm_joint_names": ["arm"],
            "ee_body": "tool",
            "tcp": {"parent_frame": "tool"},
        },
        "actuators": {
            "all": {"joint_names": ["arm", "finger"]},
        },
        "gripper": {
            "joint_names": ["finger"],
            "command_joint_names": ["finger"],
            "finger_body_names": ["left_finger", "right_finger"],
            "min_aperture_m": 0.0,
            "max_aperture_m": 0.08,
            "closed_positions": {"finger": 0.0},
            "open_positions": {"finger": 0.04},
        },
    }


def _scene_data() -> dict:
    surface = {
        "position_m": [0.0, 0.0, 0.0],
        "size_m": [1.0, 1.0, 0.1],
        "material_path": None,
        "static_friction": 0.8,
        "dynamic_friction": 0.8,
        "restitution": 0.0,
    }
    return {
        "room": {"usd_path": "room.usd"},
        "ground": surface,
        "table": surface,
        "task_object_placement_area": {
            "x_range_m": [-0.5, 0.5],
            "y_range_m": [-0.25, 0.25],
        },
        "robot_mounts": {
            "left": {
                "position_xy_m": [-0.3, 0.0],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "right": {
                "position_xy_m": [0.3, 0.0],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        },
        "camera": {
            "profile_path": "camera.yml",
            "stand_usd_path": "stand.usd",
            "stand_position_xy_m": [0.0, 0.0],
            "stand_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "sensor_local_position_m": [0.0, 0.0, 1.0],
            "sensor_local_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "lighting": {"texture_path": "light.hdr", "intensity": 1000.0},
    }


@pytest.mark.parametrize(
    ("model_type", "data"),
    [
        (CameraConfig, _camera_data()),
        (RobotConfig, _robot_data()),
        (SceneConfig, _scene_data()),
        (SimulationConfig, {}),
        (EnvironmentConfig, {}),
    ],
)
def test_models_are_frozen_and_forbid_extra_fields(model_type, data) -> None:
    model = model_type.model_validate(data)

    with pytest.raises(ValidationError, match="frozen"):
        model.__setattr__(next(iter(model_type.model_fields)), None)

    with pytest.raises(ValidationError, match="extra_forbidden"):
        model_type.model_validate({**data, "unexpected": True})


@pytest.mark.parametrize(
    "model_type",
    [
        CameraConfig,
        RobotConfig,
        SceneConfig,
        SimulationConfig,
        EnvironmentConfig,
    ],
)
def test_pure_models_do_not_load_files_or_build_native_configs(model_type) -> None:
    assert not hasattr(model_type, "load")
    assert not hasattr(model_type, "save")
    assert not any(name.startswith("build_") for name in vars(model_type))


def test_camera_rejects_non_finite_values() -> None:
    data = _camera_data()
    data["focal_length_mm"] = math.nan

    with pytest.raises(ValidationError, match="focal_length_mm"):
        CameraConfig.model_validate(data)


def test_camera_rejects_invalid_intrinsics_and_clipping_range() -> None:
    invalid_intrinsics = _camera_data()
    invalid_intrinsics["intrinsic_matrix_px"][3] = 1.0
    with pytest.raises(ValidationError, match="pinhole calibration matrix"):
        CameraConfig.model_validate(invalid_intrinsics)

    invalid_clipping = _camera_data()
    invalid_clipping["clipping_range_m"] = [2.0, 1.0]
    with pytest.raises(ValidationError, match="far plane"):
        CameraConfig.model_validate(invalid_clipping)


def test_robot_rejects_inconsistent_joint_contract() -> None:
    data = _robot_data()
    data["initial_joint_positions"].pop("finger")

    with pytest.raises(ValidationError, match="exactly cover"):
        RobotConfig.model_validate(data)


def test_robot_uses_authored_asset_scale() -> None:
    data = _robot_data()
    data["scale"] = [0.5, 0.5, 0.5]

    with pytest.raises(ValidationError, match="scale"):
        RobotConfig.model_validate(data)


def test_scene_excludes_environment_lifecycle_settings() -> None:
    data = _scene_data()
    data["runtime"] = {"num_envs": 2}

    with pytest.raises(ValidationError, match="runtime"):
        SceneConfig.model_validate(data)


def test_environment_owns_scene_cloning_settings() -> None:
    config = EnvironmentConfig(
        num_envs=4,
        env_spacing_m=3.0,
        replicate_physics=False,
        clone_in_fabric=True,
    )

    assert config.num_envs == 4
    assert config.env_spacing_m == 3.0
    assert config.replicate_physics is False
    assert config.clone_in_fabric is True


def test_scene_validates_ranges_and_quaternions() -> None:
    invalid_range = _scene_data()
    invalid_range["task_object_placement_area"]["x_range_m"] = [0.5, -0.5]
    with pytest.raises(ValidationError, match="lower bound"):
        SceneConfig.model_validate(invalid_range)

    invalid_quaternion = _scene_data()
    invalid_quaternion["robot_mounts"]["left"]["orientation_xyzw"] = [
        0.0,
        0.0,
        0.0,
        2.0,
    ]
    with pytest.raises(ValidationError, match="unit quaternion"):
        SceneConfig.model_validate(invalid_quaternion)
