"""Exercise atomic Piper skills on deterministic existing task assets."""

from __future__ import annotations

import argparse
import math
import sys
import traceback
from collections.abc import Callable
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from scale_bench.config.loader import load_config
from scale_bench.config.models.simulation import SimulationConfig

from isaaclab.app import AppLauncher


TASK_NAMES = ("move_home", "gripper", "pick", "pick_place", "reorient")

parser = argparse.ArgumentParser()
parser.add_argument(
    "--tasks",
    nargs="+",
    choices=TASK_NAMES,
    default=list(TASK_NAMES),
)
parser.add_argument("--robot", choices=("auto", "left", "right"), default="auto")
parser.add_argument("--object", default="doll_00002")
parser.add_argument("--seed", type=int, default=44)
parser.add_argument("--max-steps-per-skill", type=int, default=900)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(visualizer=["kit"], enable_cameras=True, device=None)
args = parser.parse_args()

if args.seed < 0:
    parser.error("--seed must be non-negative")
if args.max_steps_per_skill <= 0:
    parser.error("--max-steps-per-skill must be positive")

sim_config = load_config(
    PROJECT_ROOT / "configs/sim/default.yml",
    SimulationConfig,
)
if args.device is None:
    args.device = sim_config.device
if args.rendering_mode is None:
    args.rendering_mode = sim_config.render.rendering_mode

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch
from isaaclab.utils.math import quat_from_angle_axis, quat_mul

from manipulation_skills import (
    JointPositionGoal,
    LiftGoal,
    ObjectPoseGoal,
    PlaceConfig,
    SkillSequence,
    close_gripper,
    home,
    move_to_pose,
    open_gripper,
    pick,
    place,
    rotate,
)
from manipulation_skills.goals import GoalResult
from manipulation_skills.piper import PiperRuntime
from scale_bench.api import create_env
from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.tasks.sort_dolls_by_size.config import SortDollsBySizeConfig
from scale_bench.tasks.sort_dolls_by_size.task import SortDollsBySize


def run_skill(env: Any, skill: Any, name: str) -> None:
    previous_phase = None
    for step_index in range(args.max_steps_per_skill):
        result = skill.tick()
        if result.phase != previous_phase:
            print(
                f"ATOMIC_SKILL task={name} step={step_index} "
                f"phase={result.phase}",
                flush=True,
            )
            previous_phase = result.phase
        env.step(result.action)
        if result.done:
            if not result.succeeded:
                raise RuntimeError(
                    f"{name} skill failed in phase {result.phase}: {result.message}"
                )
            return
    raise RuntimeError(
        f"{name} skill did not finish after {args.max_steps_per_skill} steps"
    )


def choose_place_pose(
    env: Any,
    task: SortDollsBySize,
    scene_config: SceneConfig,
    object_name: str,
) -> tuple[torch.Tensor, torch.Tensor]:
    current = env.scene[object_name].data.root_pose_w.torch[0]
    target_radius = math.hypot(*task.metadata[object_name].size[:2]) / 2.0
    x_min, x_max = scene_config.task_object_placement_area.x_range_m
    y_min, y_max = scene_config.task_object_placement_area.y_range_m
    offsets = ((0.0, -0.15), (0.0, 0.15), (-0.15, 0.0), (0.15, 0.0))

    for dx, dy in offsets:
        candidate = current[:3].clone()
        candidate[0] += dx
        candidate[1] += dy
        if not (
            x_min + target_radius <= candidate[0].item() <= x_max - target_radius
            and y_min + target_radius <= candidate[1].item() <= y_max - target_radius
        ):
            continue
        collision = False
        for name, metadata in task.metadata.items():
            if name == object_name:
                continue
            other = env.scene[name].data.root_pose_w.torch[0, :3]
            other_radius = math.hypot(*metadata.size[:2]) / 2.0
            distance = torch.linalg.vector_norm(candidate[:2] - other[:2]).item()
            if distance < target_radius + other_radius + 0.03:
                collision = True
                break
        if not collision:
            return candidate, current[3:7].clone()
    raise RuntimeError("could not find a collision-free nearby place target")


def evaluate_or_raise(name: str, result: GoalResult) -> None:
    print(
        f"ATOMIC_TASK_RESULT task={name} success={result.succeeded} "
        f"message={result.message!r} metrics={result.metrics}",
        flush=True,
    )
    if not result.succeeded:
        raise RuntimeError(f"{name} task goal failed: {result.message}")


def select_robot(env: Any, object_name: str | None = None) -> str:
    """Choose the closest arm for object tasks, or left for arm-only tasks."""

    if args.robot != "auto":
        return args.robot
    if object_name is None:
        return "left"
    object_position = env.scene[object_name].data.root_pos_w.torch[0, :2]
    distances = {
        side: torch.linalg.vector_norm(
            object_position
            - env.scene[f"{side}_robot"].data.root_pos_w.torch[0, :2]
        ).item()
        for side in ("left", "right")
    }
    return min(distances, key=distances.__getitem__)


def task_move_home(env: Any, profile: RobotConfig, _: Any, __: Any) -> GoalResult:
    robot = select_robot(env)
    runtime = PiperRuntime(env, profile, robot, 0, 0.04)
    position, orientation = runtime.tcp_pose_w()
    target_position = position.clone()
    target_position[2] += 0.05
    sequence = SkillSequence(
        (
            lambda: move_to_pose(
                env,
                profile,
                robot=robot,
                target_pose_w=(target_position, orientation),
            ),
            lambda: home(env, profile, robot=robot),
        )
    )
    run_skill(env, sequence, "move_home")
    final_runtime = PiperRuntime(env, profile, robot, 0, 0.04)
    return JointPositionGoal(final_runtime.home_joint_positions()).evaluate(
        final_runtime.arm_joint_positions()
    )


def task_gripper(env: Any, profile: RobotConfig, _: Any, __: Any) -> GoalResult:
    robot = select_robot(env)
    sequence = SkillSequence(
        (
            lambda: close_gripper(env, profile, robot=robot),
            lambda: open_gripper(env, profile, robot=robot),
        )
    )
    run_skill(env, sequence, "gripper")
    runtime = PiperRuntime(env, profile, robot, 0, 0.04)
    aperture = runtime.gripper_aperture_m()
    error = abs(aperture - runtime.max_gripper_aperture_m)
    return GoalResult(
        succeeded=error <= 0.003,
        message=(
            f"gripper aperture is {aperture:.4f} m; "
            f"expected {runtime.max_gripper_aperture_m:.4f} m"
        ),
        metrics={"aperture_error_m": error},
    )


def task_pick(env: Any, profile: RobotConfig, _: Any, __: Any) -> GoalResult:
    robot = select_robot(env, args.object)
    initial_height = float(
        env.scene[args.object].data.root_pose_w.torch[0, 2].item()
    )
    run_skill(
        env,
        pick(env, profile, robot=robot, object_name=args.object),
        "pick",
    )
    final_pose = env.scene[args.object].data.root_pose_w.torch[0]
    return LiftGoal(initial_height, 0.08).evaluate(
        (final_pose[:3], final_pose[3:7])
    )


def task_pick_place(
    env: Any,
    profile: RobotConfig,
    task: SortDollsBySize,
    scene_config: SceneConfig,
) -> GoalResult:
    robot = select_robot(env, args.object)
    target_pose = choose_place_pose(env, task, scene_config, args.object)
    sequence = SkillSequence(
        (
            lambda: pick(
                env,
                profile,
                robot=robot,
                object_name=args.object,
            ),
            lambda: place(
                env,
                profile,
                robot=robot,
                object_name=args.object,
                target_object_pose_w=target_pose,
                config=PlaceConfig(release_clearance_m=0.04),
            ),
        )
    )
    run_skill(env, sequence, "pick_place")
    final_pose = env.scene[args.object].data.root_pose_w.torch[0]
    return ObjectPoseGoal(target_pose).evaluate(
        (final_pose[:3], final_pose[3:7])
    )


def task_reorient(env: Any, profile: RobotConfig, _: Any, __: Any) -> GoalResult:
    robot = select_robot(env, args.object)
    sequence_target: dict[str, torch.Tensor] = {}

    def make_rotate():
        current = env.scene[args.object].data.root_pose_w.torch[0]
        delta = quat_from_angle_axis(
            current.new_tensor(math.pi / 4.0),
            current.new_tensor((0.0, 0.0, 1.0)),
        )
        target = quat_mul(delta, current[3:7])
        sequence_target["orientation"] = target.clone()
        return rotate(
            env,
            profile,
            robot=robot,
            object_name=args.object,
            target_object_orientation_wxyz=target,
        )

    sequence = SkillSequence(
        (
            lambda: pick(
                env,
                profile,
                robot=robot,
                object_name=args.object,
            ),
            make_rotate,
        )
    )
    run_skill(env, sequence, "reorient")
    current = env.scene[args.object].data.root_pose_w.torch[0]
    goal = ObjectPoseGoal(
        (current[:3].clone(), sequence_target["orientation"]),
        position_tolerance_m=0.05,
        orientation_tolerance_rad=0.15,
    )
    return goal.evaluate((current[:3], current[3:7]))


TASK_RUNNERS: dict[str, Callable[..., GoalResult]] = {
    "move_home": task_move_home,
    "gripper": task_gripper,
    "pick": task_pick,
    "pick_place": task_pick_place,
    "reorient": task_reorient,
}


def main() -> None:
    scene_config = load_config(
        PROJECT_ROOT / "configs/scene/default.yml",
        SceneConfig,
        asset_root=PROJECT_ROOT,
    )
    profile = load_config(
        PROJECT_ROOT / "configs/robots/piper.yml",
        RobotConfig,
        asset_root=PROJECT_ROOT,
    )
    environment_config = load_config(
        PROJECT_ROOT / "configs/envs/default.yml",
        EnvironmentConfig,
    )
    task = SortDollsBySize(
        load_config(
            PROJECT_ROOT / "configs/tasks/sort_dolls_by_size.yml",
            SortDollsBySizeConfig,
            asset_root=PROJECT_ROOT,
        )
    )
    if args.object not in task.assets:
        raise ValueError(
            f"object {args.object!r} is not in the task; "
            f"choose one of {sorted(task.assets)}"
        )
    env = create_env(
        left_robot_config=profile,
        right_robot_config=profile,
        scene_config=scene_config,
        simulation_config=sim_config,
        environment_config=environment_config,
        task=task,
        base_seed=args.seed,
        device=args.device,
        num_envs=1,
    )

    try:
        for task_name in args.tasks:
            env.reset()
            selected = select_robot(
                env,
                args.object if task_name in {"pick", "pick_place", "reorient"} else None,
            )
            print(f"ATOMIC_TASK task={task_name} robot={selected}", flush=True)
            result = TASK_RUNNERS[task_name](env, profile, task, scene_config)
            evaluate_or_raise(task_name, result)
        print("ATOMIC_TASKS_OK", flush=True)
    finally:
        env.close()


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except BaseException as error:
        traceback.print_exc()
        print(
            f"ATOMIC_TASK_ERROR type={type(error).__name__} message={error}",
            flush=True,
        )
        exit_code = 1
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
