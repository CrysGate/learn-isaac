"""Runtime IO contract tests that do not launch a simulation."""

from __future__ import annotations

from copy import deepcopy
from types import SimpleNamespace

import pytest
import torch
from isaaclab.envs import ManagerBasedEnv

from scale_bench.isaaclab.mdp.events import resolve_env_ids
from scale_bench.isaaclab.runtime.environment import ScaleBenchEnv
from scale_bench.isaaclab.runtime.io_descriptors import (
    build_io_descriptors,
    validate_io_descriptors,
)


def _runtime_fixture():
    env = SimpleNamespace(
        action_manager=SimpleNamespace(
            active_terms=[
                "left_arm",
                "left_gripper",
                "right_arm",
                "right_gripper",
            ],
        ),
        observation_manager=SimpleNamespace(
            group_obs_concatenate={"policy": False},
            group_obs_term_dim={"policy": [(6,), (480, 640, 3)]},
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
                },
                {
                    "name": "overhead_camera_rgb",
                    "shape": [480, 640, 3],
                    "dtype": "torch.uint8",
                    "extras": {"layout": "HWC"},
                },
            ]
        },
    }
    return env, descriptors


def test_runtime_descriptor_validation_accepts_runtime_contract() -> None:
    env, descriptors = _runtime_fixture()

    validate_io_descriptors(env, descriptors)


def test_runtime_descriptor_validation_rejects_camera_dtype_mismatch() -> None:
    env, descriptors = _runtime_fixture()
    mismatched = deepcopy(descriptors)
    mismatched["observations"]["policy"][1]["dtype"] = "torch.float32"

    with pytest.raises(RuntimeError, match="must use torch.uint8"):
        validate_io_descriptors(env, mismatched)


def test_runtime_descriptors_derive_slices_from_heterogeneous_term_dims() -> None:
    action_terms = [
        "left_arm",
        "left_gripper",
        "right_arm",
        "right_gripper",
    ]
    env = SimpleNamespace(
        action_manager=SimpleNamespace(
            active_terms=action_terms,
            action_term_dim=[3, 2, 7, 1],
            get_term=lambda name: object(),
        ),
        observation_manager=SimpleNamespace(
            active_terms={"policy": ["left_arm_joint_pos"]},
        ),
        scene=SimpleNamespace(sensors={}),
        physics_dt=1.0 / 120.0,
        step_dt=1.0 / 30.0,
        cfg=SimpleNamespace(
            decimation=4,
            arm_action_mode="joint_position",
            sim=SimpleNamespace(render_interval=4),
        ),
    )
    native_descriptors = {
        "actions": [{"shape": [0]} for _ in action_terms],
        "observations": {
            "policy": [{"name": "native_left_arm", "shape": [3]}],
        },
    }

    descriptors = build_io_descriptors(env, native_descriptors)

    assert [item["shape"] for item in descriptors["actions"]] == [
        [3],
        [2],
        [7],
        [1],
    ]
    assert [item["slice"] for item in descriptors["actions"]] == [
        [0, 3],
        [3, 5],
        [5, 12],
        [12, 13],
    ]
    assert [item["name"] for item in descriptors["actions"]] == action_terms
    assert descriptors["observations"]["policy"][0]["name"] == (
        "left_arm_joint_pos"
    )


def _environment_for_action_validation() -> ScaleBenchEnv:
    env = object.__new__(ScaleBenchEnv)
    env.scene = SimpleNamespace(num_envs=2)
    env.sim = SimpleNamespace(device="cpu")
    env.action_manager = SimpleNamespace(total_action_dim=4)
    env._is_closed = True
    return env


def test_environment_accepts_contract_compliant_action(monkeypatch) -> None:
    env = _environment_for_action_validation()
    expected = ("observation", {"accepted": True})
    monkeypatch.setattr(ManagerBasedEnv, "step", lambda self, action: expected)

    result = env.step(torch.zeros((2, 4), dtype=torch.float32))

    assert result == expected


@pytest.mark.parametrize(
    ("env_ids", "expected"),
    [
        (None, (0, 1, 2)),
        (slice(0, None, 2), (0, 2)),
        ([2, 0], (2, 0)),
        (torch.tensor([1, 2], dtype=torch.int32), (1, 2)),
    ],
)
def test_resolve_env_ids_accepts_integer_indices(env_ids, expected) -> None:
    assert resolve_env_ids(env_ids, num_envs=3) == expected


@pytest.mark.parametrize(
    "env_ids",
    [
        [0.5],
        [True],
        ["1"],
        torch.tensor([0.0]),
    ],
)
def test_resolve_env_ids_rejects_non_integer_indices(env_ids) -> None:
    with pytest.raises(TypeError, match="only integers"):
        resolve_env_ids(env_ids, num_envs=2)


def test_resolve_env_ids_rejects_invalid_shape_range_and_duplicates() -> None:
    with pytest.raises(TypeError, match="one-dimensional"):
        resolve_env_ids(1, num_envs=2)
    with pytest.raises(ValueError, match="one-dimensional"):
        resolve_env_ids(torch.tensor([[0]]), num_envs=2)
    with pytest.raises(IndexError, match=r"\[0, 2\)"):
        resolve_env_ids([-1], num_envs=2)
    with pytest.raises(ValueError, match="duplicates"):
        resolve_env_ids([1, 1], num_envs=2)
