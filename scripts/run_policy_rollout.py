"""Run the policy rollout runtime against a real ScaleBenchEnv."""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from scale_bench.config.loader import load_config
from scale_bench.config.models.simulation import SimulationConfig

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument("--num-envs", type=int, default=1)
parser.add_argument("--episodes", type=int, default=1)
parser.add_argument("--base-seed", type=int, default=100)
parser.add_argument("--max-steps", type=int, default=8)
parser.add_argument(
    "--left-joint4-offset-rad",
    type=float,
    default=None,
    help="Execute one real MoveToJoints command instead of the timed hold policy.",
)
parser.add_argument(
    "--record-output",
    type=Path,
    default=None,
    help="Enable HDF5 recording in this directory.",
)
parser.add_argument("--dataset-name", default="policy_rollout_smoke")
parser.add_argument(
    "--camera-config",
    type=Path,
    default=Path("configs/cameras/d435_smoke.yml"),
    help="Low-memory RGB-D profile used by the real multi-env smoke run.",
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
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(enable_cameras=True, device=None)
args = parser.parse_args()
if args.num_envs <= 0:
    parser.error("--num-envs must be positive")
if args.episodes <= 0:
    parser.error("--episodes must be positive")
if args.base_seed < 0:
    parser.error("--base-seed must be non-negative")
if args.max_steps <= 0:
    parser.error("--max-steps must be positive")

sim_config = load_config(args.sim_config, SimulationConfig)
if args.device is None:
    args.device = sim_config.device
if args.rendering_mode is None:
    args.rendering_mode = sim_config.render.rendering_mode

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch
from torch import Tensor

from scale_bench.api import create_env
from scale_bench.config.models.environment import EnvironmentConfig
from scale_bench.config.models.recording import RecordingConfig
from scale_bench.config.models.robot import RobotConfig
from scale_bench.config.models.scene import SceneConfig
from scale_bench.runtime import (
    BenchmarkScheduler,
    EpisodeContext,
    EpisodeSpec,
    PolicyOutput,
    PolicyRolloutRunner,
    TerminationReason,
)
from scale_bench.tasks.common.placement import PlacementContext
from scale_bench.tasks.sort_dolls_by_size.config import SortDollsBySizeConfig
from scale_bench.tasks.sort_dolls_by_size.task import SortDollsBySize
from scale_bench.isaaclab.runtime.command_adapter import (
    build_command_action_layout,
)
from scale_bench.skills import (
    CommandExecutor,
    JointState,
    JointTrajectory,
    MoveToJoints,
)


MAX_JOINT_STEP_RAD = 0.02


class SeedTimedHoldPolicy:
    """Hold the reset joint pose and finish seeds at different step counts."""

    def __init__(self, action: Tensor) -> None:
        self._action = action
        self._steps = torch.zeros(
            action.shape[0],
            dtype=torch.long,
            device=action.device,
        )
        self._finish_after = torch.ones_like(self._steps)

    def reset(
        self,
        env_ids: Tensor,
        contexts: tuple[EpisodeContext, ...],
    ) -> None:
        self._steps[env_ids] = 0
        self._finish_after[env_ids] = torch.tensor(
            tuple(1 + context.spec.seed % 4 for context in contexts),
            dtype=torch.long,
            device=self._action.device,
        )

    def act(
        self,
        observation: object,
        active_mask: Tensor,
    ) -> PolicyOutput:
        del observation
        done_mask = active_mask & (self._steps >= self._finish_after)
        self._steps += (active_mask & ~done_mask).to(dtype=torch.long)
        return PolicyOutput(action=self._action, done_mask=done_mask)


class SingleJointCommandPolicy:
    """Adapt one CommandExecutor program to the policy rollout test harness."""

    def __init__(
        self,
        env: object,
        executor: CommandExecutor,
        *,
        joint_offset_rad: float,
    ) -> None:
        self._env = env
        self._executor = executor
        self._joint_offset_rad = joint_offset_rad
        self._completed = torch.zeros(
            env.num_envs,
            dtype=torch.bool,
            device=env.device,
        )

    def reset(
        self,
        env_ids: Tensor,
        contexts: tuple[EpisodeContext, ...],
    ) -> None:
        del contexts
        self._completed[env_ids] = False
        hold = self._env.hold_action()
        start, stop = self._executor.layout.left_arm
        for env_id in env_ids.detach().cpu().tolist():
            target = hold[env_id, start:stop].clone()
            target[3] += self._joint_offset_rad
            steps = max(
                1,
                int(
                    torch.ceil(
                        torch.amax(torch.abs(target - hold[env_id, start:stop]))
                        / MAX_JOINT_STEP_RAD
                    ).item()
                ),
            )
            weights = torch.linspace(
                1.0 / steps,
                1.0,
                steps,
                dtype=target.dtype,
                device=target.device,
            ).unsqueeze(-1)
            trajectory = hold[env_id, start:stop].unsqueeze(0) + weights * (
                target - hold[env_id, start:stop]
            )
            self._executor.begin(
                env_id,
                MoveToJoints(
                    arm="left",
                    target_joint_state=JointState(target),
                    trajectory=JointTrajectory(trajectory),
                    label="left_joint4_smoke",
                ),
            )

    def act(
        self,
        observation: object,
        active_mask: Tensor,
    ) -> PolicyOutput:
        del observation
        done_mask = active_mask & self._completed
        command_mask = active_mask & ~done_mask
        if command_mask.any().item():
            batch = self._executor.next_actions(command_mask)
            self._completed |= batch.completed_after_step
            action = batch.action
        else:
            action = self._env.hold_action()
        return PolicyOutput(action=action, done_mask=done_mask)


def main() -> int:
    asset_root = PROJECT_ROOT
    scene_config = load_config(
        PROJECT_ROOT / "configs/scene/default.yml",
        SceneConfig,
        asset_root=asset_root,
    )
    robot_config = load_config(
        PROJECT_ROOT / "configs/robots/piper.yml",
        RobotConfig,
        asset_root=asset_root,
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
        raise RuntimeError("policy smoke requires mounted robot cameras")
    robot_config = robot_config.model_copy(
        update={
            "camera": robot_config.camera.model_copy(
                update={"profile_path": camera_profile_path}
            )
        }
    )
    environment_config = load_config(args.env_config, EnvironmentConfig)
    task_config = load_config(
        PROJECT_ROOT / "configs/tasks/sort_dolls_by_size.yml",
        SortDollsBySizeConfig,
        asset_root=asset_root,
    )
    task = SortDollsBySize(task_config)
    placement_context = PlacementContext.from_scene_config(scene_config)
    specs = tuple(
        EpisodeSpec(
            episode_id=f"policy-seed-{seed}",
            task_id=task.task_id,
            seed=seed,
            layout=task.generate_layout(placement_context, seed),
            max_steps=args.max_steps,
        )
        for seed in range(args.base_seed, args.base_seed + args.episodes)
    )
    initial_layouts = (
        tuple(spec.layout for spec in specs[: args.num_envs])
        if len(specs) >= args.num_envs
        else (specs[0].layout,)
    )
    recording_config = (
        None
        if args.record_output is None
        else RecordingConfig(
            output_dir=args.record_output,
            dataset_name=args.dataset_name,
        )
    )
    print("[policy-smoke] creating ScaleBenchEnv", flush=True)
    env = create_env(
        left_robot_config=robot_config,
        right_robot_config=robot_config,
        scene_config=scene_config,
        simulation_config=sim_config,
        environment_config=environment_config,
        recording_config=recording_config,
        task=task,
        layouts=initial_layouts,
        device=args.device,
        num_envs=args.num_envs,
    )
    print("[policy-smoke] environment ready", flush=True)
    try:
        command_expected_steps = None
        if args.left_joint4_offset_rad is None:
            policy = SeedTimedHoldPolicy(env.hold_action().clone())
        else:
            command_expected_steps = max(
                1,
                int(
                    torch.ceil(
                        torch.tensor(
                            abs(args.left_joint4_offset_rad) / MAX_JOINT_STEP_RAD
                        )
                    ).item()
                ),
            )
            policy = SingleJointCommandPolicy(
                env,
                CommandExecutor(
                    env,
                    build_command_action_layout(
                        env,
                        left_robot_config=robot_config,
                        right_robot_config=robot_config,
                    ),
                ),
                joint_offset_rad=args.left_joint4_offset_rad,
            )
        result = BenchmarkScheduler(specs).run(PolicyRolloutRunner(env, policy))
        expected_batches = (args.episodes + args.num_envs - 1) // args.num_envs
        valid = result.batch_count == expected_batches
        for index, episode in enumerate(result.episodes.values()):
            expected_steps = (
                1 + episode.spec.seed % 4
                if command_expected_steps is None
                else command_expected_steps
            )
            expected_reason = TerminationReason.CONTROLLER_FINISHED
            episode_valid = (
                episode.termination.reason == expected_reason
                and episode.steps == expected_steps
            )
            valid &= episode_valid
            print(
                f"episode={episode.spec.episode_id} "
                f"slot={index % args.num_envs} seed={episode.spec.seed} "
                f"steps={episode.steps} success={episode.success} "
                f"progress={episode.evaluation.progress:.3f} "
                f"termination={episode.termination.reason.value}",
                flush=True,
            )
        print(
            f"batches={result.batch_count} expected_batches={expected_batches} "
            f"recording={env.recording_enabled}",
            flush=True,
        )
        return 0 if valid else 1
    finally:
        env.close()


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
