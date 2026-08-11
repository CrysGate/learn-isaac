"""Runtime IO contract tests that do not launch a simulation."""

from __future__ import annotations

from copy import deepcopy
from types import SimpleNamespace

import pytest

from scale_bench.isaaclab.runtime.io_descriptors import validate_io_descriptors


def _runtime_fixture():
    env = SimpleNamespace(
        action_manager=SimpleNamespace(
            active_terms=[
                "left_arm",
                "left_gripper",
                "right_arm",
                "right_gripper",
            ],
            action_term_dim=[6, 1, 6, 1],
        ),
        observation_manager=SimpleNamespace(
            group_obs_concatenate={"policy": False},
            group_obs_term_dim={"policy": [(6,)]},
        ),
    )
    descriptors = {
        "actions": [
            {"shape": [6]},
            {"shape": [1]},
            {"shape": [6]},
            {"shape": [1]},
        ],
        "observations": {
            "policy": [
                {
                    "name": "left_arm_joint_pos",
                    "shape": [6],
                    "dtype": "torch.float32",
                    "extras": {},
                }
            ]
        },
        "runtime": {
            "step_dt": 1.0 / 30.0,
            "render_dt": 1.0 / 30.0,
            "camera_update_periods": {"overhead_camera": 1.0 / 30.0},
        },
    }
    return env, descriptors


def test_runtime_descriptor_validation_accepts_synchronized_contract() -> None:
    env, descriptors = _runtime_fixture()

    validate_io_descriptors(env, descriptors)


def test_runtime_descriptor_validation_rejects_actual_timing_mismatch() -> None:
    env, descriptors = _runtime_fixture()
    mismatched = deepcopy(descriptors)
    mismatched["runtime"]["camera_update_periods"]["overhead_camera"] = 0.1

    with pytest.raises(RuntimeError, match="camera update periods"):
        validate_io_descriptors(env, mismatched)
