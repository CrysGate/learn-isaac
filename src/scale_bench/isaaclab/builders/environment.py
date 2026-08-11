"""Compose pure configs and task data into a complete Isaac Lab EnvCfg."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import MISSING

from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils.configclass import configclass

from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.config.models.simulation import SimulationConfig
from scale_bench.isaaclab.builders.scene import build_scene_cfg
from scale_bench.isaaclab.builders.simulation import build_simulation_cfg
from scale_bench.isaaclab.builders.task import TaskBuilder, resolve_task_builder
from scale_bench.isaaclab.managers.actions import (
    ActionsCfg,
    ArmActionMode,
    build_actions_cfg,
)
from scale_bench.isaaclab.managers.events import EventsCfg
from scale_bench.isaaclab.managers.observations import (
    ObservationsCfg,
    build_observations_cfg,
)
from scale_bench.isaaclab.mdp.events import ResetTaskLayout
from scale_bench.tasks.common.layout import TaskLayout
from scale_bench.tasks.common.placement import PlacementContext
from scale_bench.tasks.common.task import Task


@configclass
class ScaleBenchEnvCfg(ManagerBasedEnvCfg):
    """Native Isaac Lab configuration used by the ScaleBench runtime."""

    scene: InteractiveSceneCfg = MISSING
    sim: SimulationCfg = MISSING
    decimation: int = MISSING
    arm_action_mode: ArmActionMode = MISSING
    actions: ActionsCfg = MISSING
    observations: ObservationsCfg = MISSING
    events: EventsCfg = EventsCfg()


def build_environment_cfg(
    *,
    left_robot_config: RobotConfig,
    right_robot_config: RobotConfig,
    scene_config: SceneConfig,
    simulation_config: SimulationConfig,
    environment_config: EnvironmentConfig,
    task: Task | None = None,
    task_layout_seed: int | None = None,
    task_layouts: Sequence[TaskLayout] | None = None,
    task_builder: TaskBuilder | None = None,
    device: str | None = None,
    num_envs: int | None = None,
    env_spacing_m: float | None = None,
) -> ScaleBenchEnvCfg:
    """Build a complete native environment cfg from resolved inputs."""

    if task is None and (
        task_layout_seed is not None
        or task_layouts is not None
        or task_builder is not None
    ):
        raise ValueError("task layout inputs and task_builder require a task")
    if task is not None and (task_layout_seed is None) == (task_layouts is None):
        raise ValueError(
            "task requires exactly one of task_layout_seed or task_layouts"
        )
    scene_cfg = build_scene_cfg(
        left_robot_config=left_robot_config,
        right_robot_config=right_robot_config,
        scene_config=scene_config,
        environment_config=environment_config,
        num_envs=num_envs,
        env_spacing_m=env_spacing_m,
    )
    events = EventsCfg()
    if task is not None:
        placement_context = PlacementContext.from_scene_config(scene_config)
        layouts = _prepare_task_layouts(
            task=task,
            context=placement_context,
            num_envs=scene_cfg.num_envs,
            base_seed=task_layout_seed,
            task_layouts=task_layouts,
        )
        builder = resolve_task_builder(task, task_builder)
        asset_cfgs = builder.build_assets(task, layouts[0])
        _validate_task_asset_names(layouts[0], asset_cfgs)
        _add_task_assets(scene_cfg, asset_cfgs)
        events.task_layout = EventTerm(
            func=ResetTaskLayout,
            mode="reset",
            params={
                "task": task,
                "layouts": layouts,
            },
        )

    cfg = ScaleBenchEnvCfg(
        scene=scene_cfg,
        sim=build_simulation_cfg(simulation_config, device=device),
        decimation=environment_config.control_decimation,
        arm_action_mode=environment_config.arm_action_mode,
        actions=build_actions_cfg(
            left_robot_config=left_robot_config,
            right_robot_config=right_robot_config,
            arm_action_mode=environment_config.arm_action_mode,
        ),
        observations=build_observations_cfg(
            left_robot_config=left_robot_config,
            right_robot_config=right_robot_config,
            scene_cfg=scene_cfg,
        ),
        seed=environment_config.seed,
        num_rerenders_on_reset=environment_config.num_rerenders_on_reset,
        wait_for_textures=environment_config.wait_for_textures,
        events=events,
    )
    cfg.validate()
    _validate_runtime_timing(cfg)
    return cfg


def _add_task_assets(
    scene_cfg: InteractiveSceneCfg,
    asset_cfgs: Mapping[str, RigidObjectCfg],
) -> None:
    conflicts = [name for name in asset_cfgs if hasattr(scene_cfg, name)]
    if conflicts:
        raise ValueError(
            "scene_cfg already contains task asset fields: " + ", ".join(conflicts)
        )
    for name, asset_cfg in asset_cfgs.items():
        setattr(scene_cfg, name, asset_cfg)


def _validate_task_asset_names(
    layout: TaskLayout,
    asset_cfgs: Mapping[str, RigidObjectCfg],
) -> None:
    expected_names = set(layout.assets)
    actual_names = set(asset_cfgs)
    if actual_names != expected_names:
        missing = sorted(expected_names - actual_names)
        unexpected = sorted(actual_names - expected_names)
        raise ValueError(
            "TaskBuilder assets do not match the task layout; "
            f"missing={missing}, unexpected={unexpected}"
        )


def _prepare_task_layouts(
    *,
    task: Task,
    context: PlacementContext,
    num_envs: int,
    base_seed: int | None,
    task_layouts: Sequence[TaskLayout] | None,
) -> tuple[TaskLayout, ...]:
    if task_layouts is None:
        if base_seed is None:
            raise ValueError("base_seed is required when task_layouts is not provided")
        layouts = tuple(
            task.generate_layout(context, base_seed + env_id)
            for env_id in range(num_envs)
        )
    else:
        layouts = tuple(task_layouts)
        if len(layouts) not in {1, num_envs}:
            raise ValueError(
                "task_layouts must contain either one layout or exactly "
                f"num_envs ({num_envs}) layouts; got {len(layouts)}"
            )

    for layout in layouts:
        task.validate_layout(context, layout)
    return layouts * num_envs if len(layouts) == 1 else layouts


def _camera_update_periods(
    scene_cfg: InteractiveSceneCfg,
) -> tuple[tuple[str, float], ...]:
    return tuple(
        (name, value.update_period)
        for name, value in vars(scene_cfg).items()
        if isinstance(value, CameraCfg)
    )


def _validate_runtime_timing(cfg: ScaleBenchEnvCfg) -> None:
    step_dt = cfg.sim.dt * cfg.decimation
    if cfg.sim.render_interval != cfg.decimation:
        raise ValueError(
            "render_interval must equal control_decimation for the synchronous "
            "environment contract; "
            f"got {cfg.sim.render_interval} and {cfg.decimation}"
        )

    mismatched_cameras = [
        f"{name}={period:g}s"
        for name, period in _camera_update_periods(cfg.scene)
        if not math.isclose(period, step_dt, rel_tol=0.0, abs_tol=1.0e-9)
    ]
    if mismatched_cameras:
        raise ValueError(
            "camera update periods must equal the environment step_dt "
            f"({step_dt:g}s); mismatched: {', '.join(mismatched_cameras)}"
        )


__all__ = ["ScaleBenchEnvCfg", "build_environment_cfg"]
