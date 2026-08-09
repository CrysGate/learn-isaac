"""Compose project profiles into an Isaac Lab manager-based environment cfg."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import MISSING

import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.envs.manager_based_env_cfg import DefaultEventManagerCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils.configclass import configclass

from scale_bench.robots import RobotProfile
from scale_bench.scenes import SceneConfig, create_dual_arm_tabletop_scene_cfg
from scale_bench.sim import SimConfig
from scale_bench.tasks import TaskDefinition, TaskLayout

from .action_cfg import ActionsCfg, ArmActionMode, create_actions_cfg
from .events import ResetTaskLayout
from .observation_cfg import ObservationsCfg, create_observations_cfg
from .runtime_config import EnvRuntimeConfig


@configclass
class EventsCfg(DefaultEventManagerCfg):
    """Reset the complete scene, including stale articulation command targets."""

    reset_scene_to_default = EventTerm(
        func=mdp.reset_scene_to_default,
        mode="reset",
        params={"reset_joint_targets": True},
    )
    task_layout: EventTerm | None = None


@configclass
class ScaleBenchEnvCfg(ManagerBasedEnvCfg):
    """Native Isaac Lab configuration used by the ScaleBench runtime entry."""

    scene: InteractiveSceneCfg = MISSING
    sim: SimulationCfg = MISSING
    decimation: int = MISSING
    arm_action_mode: ArmActionMode = MISSING
    actions: ActionsCfg = MISSING
    observations: ObservationsCfg = MISSING
    events: EventsCfg = EventsCfg()


def create_env_cfg(
    *,
    left_robot_profile: RobotProfile,
    right_robot_profile: RobotProfile,
    scene_config: SceneConfig,
    sim_config: SimConfig,
    runtime_config: EnvRuntimeConfig,
    task: TaskDefinition | None = None,
    task_layout_seed: int | None = None,
    task_layouts: Sequence[TaskLayout] | None = None,
    device: str | None = None,
    num_envs: int | None = None,
    env_spacing_m: float | None = None,
) -> ScaleBenchEnvCfg:
    """Compile profiles and one unambiguous task-layout source into an EnvCfg.

    One explicit task layout is broadcast; ``num_envs`` layouts map by index.
    """

    if task is None and (task_layout_seed is not None or task_layouts is not None):
        raise ValueError("task_layout_seed and task_layouts require a task")
    if task is not None and (task_layout_seed is None) == (task_layouts is None):
        raise ValueError(
            "task requires exactly one of task_layout_seed or task_layouts"
        )
    if task is not None and task.scene_config != scene_config:
        raise ValueError("task and environment must use the same SceneConfig")

    scene_cfg = create_dual_arm_tabletop_scene_cfg(
        left_robot_profile=left_robot_profile,
        right_robot_profile=right_robot_profile,
        scene_config=scene_config,
        num_envs=num_envs,
        env_spacing_m=env_spacing_m,
    )
    events = EventsCfg()
    if task is not None:
        layouts = _prepare_task_layouts(
            task=task,
            num_envs=scene_cfg.num_envs,
            base_seed=task_layout_seed,
            task_layouts=task_layouts,
        )
        task.add_assets_to_scene(scene_cfg, layouts[0])
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
        sim=sim_config.build_simulation_cfg(device=device),
        decimation=runtime_config.control_decimation,
        arm_action_mode=runtime_config.arm_action_mode,
        actions=create_actions_cfg(
            left_robot_profile=left_robot_profile,
            right_robot_profile=right_robot_profile,
            arm_action_mode=runtime_config.arm_action_mode,
        ),
        observations=create_observations_cfg(
            left_robot_profile=left_robot_profile,
            right_robot_profile=right_robot_profile,
            scene_cfg=scene_cfg,
        ),
        seed=runtime_config.seed,
        num_rerenders_on_reset=runtime_config.num_rerenders_on_reset,
        wait_for_textures=runtime_config.wait_for_textures,
        events=events,
    )
    cfg.validate()
    _validate_runtime_timing(cfg)
    return cfg


def _prepare_task_layouts(
    *,
    task: TaskDefinition,
    num_envs: int,
    base_seed: int | None,
    task_layouts: Sequence[TaskLayout] | None,
) -> tuple[TaskLayout, ...]:
    """Resolve the immutable initial layout assigned to each environment."""

    if task_layouts is not None:
        layouts = tuple(task_layouts)
        if len(layouts) == 1:
            task.validate_layout(layouts[0])
            return layouts * num_envs
        if len(layouts) != num_envs:
            raise ValueError(
                "task_layouts must contain either one layout or exactly "
                f"num_envs ({num_envs}) layouts; got {len(layouts)}"
            )
        for layout in layouts:
            task.validate_layout(layout)
        return layouts

    if base_seed is None:
        raise ValueError("base_seed is required when task_layouts is not provided")

    first_layout = task.generate_layout(base_seed)
    return (first_layout,) + tuple(
        task.generate_layout(base_seed + env_id) for env_id in range(1, num_envs)
    )


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


__all__ = [
    "EventsCfg",
    "ScaleBenchEnvCfg",
    "create_env_cfg",
]
