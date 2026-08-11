"""Public API assembly tests that do not import Isaac Lab."""

from __future__ import annotations

import sys
from types import ModuleType

from scale_bench.api import create_env


def test_create_env_delays_adapter_imports_and_forwards_inputs(monkeypatch) -> None:
    calls = {}
    cfg = object()

    builder_module = ModuleType("scale_bench.isaaclab.builders.environment")

    def build_environment_cfg(**kwargs):
        calls["builder"] = kwargs
        return cfg

    builder_module.build_environment_cfg = build_environment_cfg

    runtime_module = ModuleType("scale_bench.isaaclab.runtime.environment")

    class FakeEnv:
        def __init__(self, received_cfg) -> None:
            calls["runtime"] = received_cfg

    runtime_module.ScaleBenchEnv = FakeEnv
    monkeypatch.setitem(
        sys.modules,
        "scale_bench.isaaclab.builders.environment",
        builder_module,
    )
    monkeypatch.setitem(
        sys.modules,
        "scale_bench.isaaclab.runtime.environment",
        runtime_module,
    )

    inputs = {
        "left_robot_config": object(),
        "right_robot_config": object(),
        "scene_config": object(),
        "simulation_config": object(),
        "environment_config": object(),
        "task": object(),
        "base_seed": 17,
        "layouts": None,
        "task_builder": object(),
        "device": "cpu",
        "num_envs": 3,
        "env_spacing_m": 2.5,
    }
    env = create_env(**inputs)

    assert isinstance(env, FakeEnv)
    assert calls["runtime"] is cfg
    assert calls["builder"] == {
        "left_robot_config": inputs["left_robot_config"],
        "right_robot_config": inputs["right_robot_config"],
        "scene_config": inputs["scene_config"],
        "simulation_config": inputs["simulation_config"],
        "environment_config": inputs["environment_config"],
        "task": inputs["task"],
        "task_layout_seed": 17,
        "task_layouts": None,
        "task_builder": inputs["task_builder"],
        "device": "cpu",
        "num_envs": 3,
        "env_spacing_m": 2.5,
    }
