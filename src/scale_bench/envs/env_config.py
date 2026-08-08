"""Compose project profiles into an Isaac Lab manager-based environment config."""

from __future__ import annotations

import math
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

from .events import ResetTaskLayout
from .runtime_config import EnvRuntimeConfig


@configclass
class ActionsCfg:
    """Action Manager extension point; populated in the Action Manager stage."""

    pass


@configclass
class ObservationsCfg:
    """Observation Manager extension point; populated in the Observation Manager stage."""

    pass


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
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventsCfg = EventsCfg()


def create_env_cfg(
    *,
    left_robot_profile: RobotProfile,
    right_robot_profile: RobotProfile,
    scene_config: SceneConfig,
    sim_config: SimConfig,
    runtime_config: EnvRuntimeConfig,
    task: TaskDefinition | None = None,
    layout: TaskLayout | None = None,
    resample_task_layouts: bool = False,
    device: str | None = None,
    num_envs: int | None = None,
    env_spacing_m: float | None = None,
) -> ScaleBenchEnvCfg:
    """Compile project profiles into one native manager-based environment config."""

    if task is None and (layout is not None or resample_task_layouts):
        raise ValueError("layout and resample_task_layouts require a task")
    if task is not None and layout is None:
        raise ValueError("task requires an already resolved layout")
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
    if task is not None and layout is not None:
        task.add_assets_to_scene(scene_cfg, layout)
        events.task_layout = EventTerm(
            func=ResetTaskLayout,
            mode="reset",
            params={
                "task": task,
                "initial_layout": layout,
                "resample_on_reset": resample_task_layouts,
            },
        )

    cfg = ScaleBenchEnvCfg(
        scene=scene_cfg,
        sim=sim_config.build_simulation_cfg(device=device),
        decimation=runtime_config.control_decimation,
        seed=runtime_config.seed,
        num_rerenders_on_reset=runtime_config.num_rerenders_on_reset,
        wait_for_textures=runtime_config.wait_for_textures,
        events=events,
    )
    cfg.validate()
    _validate_runtime_timing(cfg)
    return cfg


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
    "ActionsCfg",
    "EventsCfg",
    "ObservationsCfg",
    "ScaleBenchEnvCfg",
    "create_env_cfg",
]
