"""Run one Piper pick in the current ScaleBench task environment."""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from scale_bench.sim import SimConfig

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument("--robot", choices=("left", "right"), default="left")
parser.add_argument("--object", default="doll_00002")
parser.add_argument("--seed", type=int, default=44)
parser.add_argument("--lift-distance", type=float, default=0.10)
parser.add_argument("--max-steps", type=int, default=900)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(visualizer=["kit"], enable_cameras=True, device=None)
args = parser.parse_args()

if args.seed < 0:
    parser.error("--seed must be non-negative")
if args.lift_distance <= 0.0:
    parser.error("--lift-distance must be positive")
if args.max_steps <= 0:
    parser.error("--max-steps must be positive")

sim_config = SimConfig.load()
if args.device is None:
    args.device = sim_config.device
if args.rendering_mode is None:
    args.rendering_mode = sim_config.render.rendering_mode

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

from manipulation_skills import PickConfig, pick
from scale_bench.envs import EnvRuntimeConfig, ScaleBenchEnv, create_env_cfg
from scale_bench.robots import RobotProfile
from scale_bench.scenes import SceneConfig
from scale_bench.tasks import SortDollsBySize


def main() -> None:
    scene_config = SceneConfig.load()
    profile = RobotProfile.load("configs/robots/piper.yml")
    task = SortDollsBySize(scene_config=scene_config)
    env_cfg = create_env_cfg(
        left_robot_profile=profile,
        right_robot_profile=profile,
        scene_config=scene_config,
        sim_config=sim_config,
        runtime_config=EnvRuntimeConfig.load(),
        task=task,
        task_layout_seed=args.seed,
        device=args.device,
        num_envs=1,
    )

    env = ScaleBenchEnv(env_cfg)
    try:
        env.reset()
        print("PICK_SETUP stage=environment_reset", flush=True)
        skill = pick(
            env,
            profile,
            robot=args.robot,
            object_name=args.object,
            config=PickConfig(lift_distance_m=args.lift_distance),
        )
        print("PICK_SETUP stage=skill_ready", flush=True)
        previous_phase = None
        for step_index in range(args.max_steps):
            result = skill.tick()
            if result.phase is not previous_phase:
                print(
                    f"PICK_PHASE step={step_index} phase={result.phase.value}",
                    flush=True,
                )
                previous_phase = result.phase
            env.step(result.action)
            if result.done:
                print(
                    f"PICK_RESULT success={result.succeeded} "
                    f"message={result.message!r}",
                    flush=True,
                )
                if not result.succeeded:
                    raise RuntimeError(result.message)
                return
        raise RuntimeError(f"pick did not finish after {args.max_steps} steps")
    except BaseException as error:
        traceback.print_exc()
        print(
            f"PICK_ERROR type={type(error).__name__} message={error}",
            flush=True,
        )
        raise
    finally:
        env.close()


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except BaseException:
        exit_code = 1
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
