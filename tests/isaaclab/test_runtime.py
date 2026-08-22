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
from scale_bench.tasks.common.evaluation import EvaluationResult


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


def test_environment_evaluates_selected_latest_observations() -> None:
    env = _environment_for_action_validation()

    class Task:
        def evaluate(self, observation):
            value = float(observation["score"].item())
            return EvaluationResult(success=value >= 0.5, progress=value)

    env._task = Task()
    env.obs_buf = {"evaluator": {"score": torch.tensor([[0.25], [0.75]])}}

    results = env.evaluate(env_ids=(1,))

    assert tuple(results) == (1,)
    assert results[1].success is True
    assert results[1].progress == pytest.approx(0.75)


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


class _FakeRecorderManager:
    active_terms = ["actions"]

    def __init__(self) -> None:
        self.success_call = None
        self.export_call = None

    def set_success_to_episodes(self, env_ids, success) -> None:
        self.success_call = (env_ids.clone(), success.clone())

    def export_episodes(self, env_ids, demo_ids=None) -> None:
        self.export_call = (env_ids.clone(), demo_ids)


def test_complete_episodes_marks_success_before_export() -> None:
    env = _environment_for_action_validation()
    recorder = _FakeRecorderManager()
    env.recorder_manager = recorder

    env.complete_episodes(
        env_ids=(1, 0),
        success=(True, False),
        demo_ids=(7, 8),
    )

    success_env_ids, success_values = recorder.success_call
    export_env_ids, demo_ids = recorder.export_call
    torch.testing.assert_close(
        success_env_ids,
        torch.tensor([1, 0], dtype=torch.int32),
    )
    torch.testing.assert_close(success_values, torch.tensor([True, False]))
    torch.testing.assert_close(
        export_env_ids,
        torch.tensor([1, 0], dtype=torch.int32),
    )
    assert demo_ids == (7, 8)


def test_complete_episodes_validates_recording_and_success_contract() -> None:
    env = _environment_for_action_validation()
    env.recorder_manager = SimpleNamespace(active_terms=[])
    with pytest.raises(RuntimeError, match="not enabled"):
        env.complete_episodes(success=(True, True))

    env.recorder_manager = _FakeRecorderManager()
    with pytest.raises(TypeError, match="boolean"):
        env.complete_episodes(success=(1, 0))
    with pytest.raises(ValueError, match="one value per environment"):
        env.complete_episodes(success=(True,))


def test_reset_attaches_layout_seed_to_recorded_episode(monkeypatch) -> None:
    env = _environment_for_action_validation()
    episodes = {0: SimpleNamespace(seed=None), 1: SimpleNamespace(seed=None)}
    env.recorder_manager = SimpleNamespace(
        active_terms=["initial_state"],
        get_episode=lambda env_id: episodes[env_id],
    )
    env._task_layout_reset = SimpleNamespace(
        episode_info=lambda env_ids: {
            "env_ids": (1,),
            "task_id": "fixture",
            "instruction": "Move the object.",
            "layout_seeds": (42,),
        }
    )
    monkeypatch.setattr(
        ManagerBasedEnv,
        "reset",
        lambda self, **kwargs: ({"policy": {}}, {}),
    )

    _, info = env.reset(env_ids=(1,))

    assert info["episode"]["layout_seeds"] == (42,)
    assert episodes[0].seed is None
    assert episodes[1].seed == 42


def test_close_releases_parent_resources_when_recorder_close_fails(
    monkeypatch,
) -> None:
    env = _environment_for_action_validation()
    env._is_closed = False

    def fail_close() -> None:
        raise OSError("dataset close failed")

    env.recorder_manager = SimpleNamespace(close=fail_close)
    parent_closed = False

    def close_parent(self) -> None:
        nonlocal parent_closed
        parent_closed = True
        self._is_closed = True

    monkeypatch.setattr(ManagerBasedEnv, "close", close_parent)

    with pytest.raises(OSError, match="dataset close failed"):
        env.close()

    assert parent_closed is True
