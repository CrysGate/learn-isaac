"""Public environment assembly API with delayed Isaac Lab imports."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.recording import RecordingConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.config.models.simulation import SimulationConfig
from scale_bench.tasks.common.layout import TaskLayout
from scale_bench.tasks.common.task import Task


def create_env(
    *,
    left_robot_config: RobotConfig,
    right_robot_config: RobotConfig,
    scene_config: SceneConfig,
    simulation_config: SimulationConfig,
    environment_config: EnvironmentConfig,
    recording_config: RecordingConfig | None = None,
    task: Task | None = None,
    base_seed: int | None = None,
    layouts: Sequence[TaskLayout] | None = None,
    task_builder: Any = None,
    device: str | None = None,
    num_envs: int | None = None,
    env_spacing_m: float | None = None,
) -> Any:
    """Build and return an environment after the caller has launched Isaac Sim."""

    from scale_bench.isaaclab.builders.environment import build_environment_cfg
    from scale_bench.isaaclab.runtime.environment import ScaleBenchEnv

    cfg = build_environment_cfg(
        left_robot_config=left_robot_config,
        right_robot_config=right_robot_config,
        scene_config=scene_config,
        simulation_config=simulation_config,
        environment_config=environment_config,
        recording_config=recording_config,
        task=task,
        task_layout_seed=base_seed,
        task_layouts=layouts,
        task_builder=task_builder,
        device=device,
        num_envs=num_envs,
        env_spacing_m=env_spacing_m,
    )
    return ScaleBenchEnv(cfg)


__all__ = ["create_env"]
