"""Replay one ScaleBench HDF5 episode and recompute its task evaluation."""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

SUPPORTED_TASK_IDS = (
    "sort_dolls_by_size",
    "single_object_pick_and_place",
)

from scale_bench.config.loader import load_config
from scale_bench.config.models.simulation import SimulationConfig

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument(
    "--task",
    choices=SUPPORTED_TASK_IDS,
    default="sort_dolls_by_size",
    help="Task used to rebuild the episode layout and recompute success.",
)
parser.add_argument(
    "--episode-name",
    default=None,
    help="HDF5 group name; omitted only when the dataset has one episode.",
)
parser.add_argument(
    "--scene-config",
    type=Path,
    default=Path("configs/scene/default.yml"),
)
parser.add_argument(
    "--camera-config",
    type=Path,
    default=Path("configs/cameras/d435_smoke.yml"),
)
parser.add_argument(
    "--sim-config",
    type=Path,
    default=Path("configs/sim/default.yml"),
)
parser.add_argument(
    "--env-config",
    type=Path,
    default=Path("configs/envs/default.yml"),
)
parser.add_argument(
    "--wait-for-close",
    action="store_true",
    help=(
        "After replay, pause on the final episode state until the Kit window "
        "is closed. Used by run_demo_generation.py --replay for inspection."
    ),
)
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("dataset", type=Path)
parser.set_defaults(enable_cameras=True, device=None)
args = parser.parse_args()
if not args.dataset.is_file():
    parser.error(f"dataset does not exist: {args.dataset}")

sim_config = load_config(args.sim_config, SimulationConfig)
if args.device is None:
    args.device = sim_config.device
if args.rendering_mode is None:
    args.rendering_mode = sim_config.render.rendering_mode

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import h5py
from isaaclab.utils.datasets import HDF5DatasetFileHandler

from scale_bench.api import create_env
from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.runtime import EpisodeReplayRunner, RecordedEpisode
from scale_bench.tasks.common.placement import PlacementContext
from scale_bench.tasks.single_object_pick_and_place.config import (
    SingleObjectPickAndPlaceConfig,
)
from scale_bench.tasks.single_object_pick_and_place.task import (
    SingleObjectPickAndPlace,
)
from scale_bench.tasks.sort_dolls_by_size.config import SortDollsBySizeConfig
from scale_bench.tasks.sort_dolls_by_size.task import SortDollsBySize


def main() -> int:
    dataset_path = args.dataset.resolve()
    with h5py.File(dataset_path, "r") as dataset:
        episode_names = tuple(dataset["data"])
        if not episode_names:
            raise ValueError("dataset has no episodes")
        episode_name = _resolve_episode_name(episode_names, args.episode_name)
        episode_group = dataset["data"][episode_name]
        if "seed" not in episode_group.attrs:
            raise ValueError("recorded episode has no seed")
        if "success" not in episode_group.attrs:
            raise ValueError("recorded episode has no success result")
        seed = int(episode_group.attrs["seed"])
        recorded_success = bool(episode_group.attrs["success"])
        recorded_steps = int(episode_group.attrs["num_samples"])

    scene_config = load_config(
        PROJECT_ROOT / args.scene_config,
        SceneConfig,
        asset_root=PROJECT_ROOT,
    )
    robot_config = load_config(
        PROJECT_ROOT / "configs/robots/piper.yml",
        RobotConfig,
        asset_root=PROJECT_ROOT,
    )
    camera_profile_path = str((PROJECT_ROOT / args.camera_config).resolve())
    scene_config = scene_config.model_copy(
        update={
            "camera": scene_config.camera.model_copy(
                update={"profile_path": camera_profile_path}
            )
        }
    )
    if robot_config.camera is None:
        raise RuntimeError("replay requires mounted robot cameras")
    robot_config = robot_config.model_copy(
        update={
            "camera": robot_config.camera.model_copy(
                update={"profile_path": camera_profile_path}
            )
        }
    )
    environment_config = load_config(args.env_config, EnvironmentConfig)
    if args.task == "single_object_pick_and_place":
        task = SingleObjectPickAndPlace(
            load_config(
                PROJECT_ROOT
                / "configs/tasks/single_object_pick_and_place.yml",
                SingleObjectPickAndPlaceConfig,
                asset_root=PROJECT_ROOT,
            )
        )
    else:
        task = SortDollsBySize(
            load_config(
                PROJECT_ROOT / "configs/tasks/sort_dolls_by_size.yml",
                SortDollsBySizeConfig,
                asset_root=PROJECT_ROOT,
            )
        )
    layout = task.generate_layout(
        PlacementContext.from_scene_config(scene_config),
        seed,
    )

    print(
        f"[replay] creating ScaleBenchEnv for {episode_name} seed={seed}",
        flush=True,
    )
    env = create_env(
        left_robot_config=robot_config,
        right_robot_config=robot_config,
        scene_config=scene_config,
        simulation_config=sim_config,
        environment_config=environment_config,
        recording_config=None,
        task=task,
        layouts=(layout,),
        device=args.device,
        num_envs=1,
    )
    handler = HDF5DatasetFileHandler()
    handler.open(str(dataset_path))
    try:
        episode_data = handler.load_episode(episode_name, env.device)
        if episode_data is None:
            raise RuntimeError(f"episode disappeared from dataset: {episode_name}")
        initial_state = episode_data.get_initial_state()
        actions = episode_data.data.get("actions")
        if initial_state is None:
            raise ValueError("episode must contain initial_state")
        if actions is None:
            if recorded_steps != 0:
                raise ValueError("episode has no actions")
            hold = env.hold_action()
            actions = hold.new_empty((0, hold.shape[1]))
        result = EpisodeReplayRunner(env).run(
            RecordedEpisode(
                episode_name=episode_name,
                seed=seed,
                layout=layout,
                initial_state=initial_state,
                actions=actions,
                recorded_steps=recorded_steps,
                recorded_success=recorded_success,
            )
        )
        print(
            f"episode={result.episode_name} seed={result.seed} "
            f"layout_matches={result.layout_matches} "
            f"steps={result.replayed_steps}/{result.recorded_steps} "
            f"recorded_success={result.recorded_success} "
            f"replayed_success={result.evaluation.success} "
            f"progress={result.evaluation.progress:.3f} "
            f"consistent={result.consistent}",
            flush=True,
        )
        for name, value in result.evaluation.metrics.items():
            print(f"metric {name}={value:.6f}", flush=True)
        for status in getattr(result.evaluation, "statuses", ()):
            print(
                f"object={status.object_name} placed={status.placed} "
                f"position={status.position_m} "
                f"target={status.target_position_m} "
                f"xy_error={status.position_error_m:.6f} "
                f"z_error={status.height_error_m:.6f} "
                f"upright_error={status.upright_error_rad:.6f}",
                flush=True,
            )
        if args.wait_for_close:
            env.sim.pause()
            print(
                "[replay] paused at the final episode state; "
                "close the Kit window to exit",
                flush=True,
            )
            while simulation_app.is_running():
                simulation_app.update()
        return 0 if result.consistent else 1
    finally:
        handler.close()
        env.close()


def _resolve_episode_name(
    episode_names: tuple[str, ...],
    requested: str | None,
) -> str:
    if requested is None:
        if len(episode_names) != 1:
            raise ValueError(
                "--episode-name is required when the dataset has multiple episodes"
            )
        return episode_names[0]
    candidates = (requested, f"demo_{requested}")
    for candidate in candidates:
        if candidate in episode_names:
            return candidate
    raise ValueError(f"unknown episode name: {requested!r}")


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    except BaseException:
        traceback.print_exc()
        exit_code = 1
    finally:
        try:
            simulation_app.close(exit_code=exit_code)
        except SystemExit:
            # Older Isaac Sim builds may use SystemExit during shutdown.
            pass
    raise SystemExit(exit_code)
