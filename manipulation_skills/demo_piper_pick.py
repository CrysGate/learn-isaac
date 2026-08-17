"""Run one Piper pick in the current ScaleBench task environment."""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from scale_bench.config.loader import load_config
from scale_bench.config.models.recording import RecordingConfig
from scale_bench.config.models.simulation import SimulationConfig

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument("--robot", choices=("auto", "left", "right"), default="auto")
parser.add_argument("--object", default="doll_00002")
parser.add_argument("--seed", type=int, default=44)
parser.add_argument("--lift-distance", type=float, default=0.10)
parser.add_argument("--max-steps", type=int, default=900)
parser.add_argument(
    "--recording-output-dir",
    type=Path,
    default=PROJECT_ROOT / "outputs/datasets",
)
parser.add_argument("--dataset-name", default="piper_pick")
parser.add_argument("--record-cameras", action="store_true")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(visualizer=["kit"], enable_cameras=True, device=None)
args = parser.parse_args()

if args.seed < 0:
    parser.error("--seed must be non-negative")
if args.lift_distance <= 0.0:
    parser.error("--lift-distance must be positive")
if args.max_steps <= 0:
    parser.error("--max-steps must be positive")

recording_config = RecordingConfig(
    output_dir=args.recording_output_dir,
    dataset_name=args.dataset_name,
    record_camera_observations=args.record_cameras,
)
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

from manipulation_skills import PickConfig, pick
from manipulation_skills.piper import PiperRuntime
from scale_bench.api import create_env
from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.tasks.sort_dolls_by_size.config import SortDollsBySizeConfig
from scale_bench.tasks.sort_dolls_by_size.task import SortDollsBySize


def select_robot(env) -> str:
    """Choose the closest arm unless the caller explicitly selected one."""

    if args.robot != "auto":
        return args.robot
    object_position = env.scene[args.object].data.root_pos_w.torch[0, :2]
    distances = {
        side: torch.linalg.vector_norm(
            object_position
            - env.scene[f"{side}_robot"].data.root_pos_w.torch[0, :2]
        ).item()
        for side in ("left", "right")
    }
    return min(distances, key=distances.__getitem__)


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
    env = create_env(
        left_robot_config=profile,
        right_robot_config=profile,
        scene_config=scene_config,
        simulation_config=sim_config,
        environment_config=environment_config,
        recording_config=recording_config,
        task=task,
        base_seed=args.seed,
        device=args.device,
        num_envs=1,
    )

    try:
        env.reset()
        print("PICK_SETUP stage=environment_reset", flush=True)
        robot = select_robot(env)
        print(f"PICK_SETUP robot={robot}", flush=True)
        runtime = PiperRuntime(env, profile, robot, 0, 0.04)
        tcp_position, tcp_orientation = runtime.tcp_pose_w()
        object_position, object_orientation = runtime.object_pose_w(args.object)
        grasp = runtime.default_grasp(args.object, 0.005)
        pregrasp_position = grasp.pose_w[0] - 0.10 * grasp.approach_direction_w
        print(
            "PICK_DIAGNOSTIC "
            f"base={runtime.robot.data.root_pos_w.torch[0].tolist()} "
            f"object_pose={object_position.tolist() + object_orientation.tolist()} "
            f"tcp_pose={tcp_position.tolist() + tcp_orientation.tolist()} "
            f"pregrasp_pose={pregrasp_position.tolist() + grasp.pose_w[1].tolist()} "
            f"approach={grasp.approach_direction_w.tolist()}",
            flush=True,
        )
        skill = pick(
            env,
            profile,
            robot=robot,
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
                env.complete_episodes(success=(result.succeeded,))
                print(
                    f"PICK_RESULT success={result.succeeded} "
                    f"message={result.message!r}",
                    flush=True,
                )
                if not result.succeeded:
                    final_runtime = PiperRuntime(env, profile, robot, 0, 0.04)
                    final_position, final_orientation = final_runtime.tcp_pose_w()
                    print(
                        "PICK_DIAGNOSTIC "
                        f"final_tcp_pose={final_position.tolist() + final_orientation.tolist()} "
                        f"final_arm_joints={final_runtime.arm_joint_positions().tolist()}",
                        flush=True,
                    )
                    raise RuntimeError(result.message)
                return
        env.complete_episodes(success=(False,))
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
