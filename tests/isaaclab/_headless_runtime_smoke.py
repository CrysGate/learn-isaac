"""Run the initialized runtime contract in an isolated Isaac Sim process."""

from __future__ import annotations

import math
import sys
import traceback
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))


def main(asset_root: Path) -> None:
    from scale_bench.api import create_env
    from scale_bench.config.loader import load_config
    from scale_bench.config.models.environment import EnvironmentConfig
    from scale_bench.config.models.robot import RobotConfig
    from scale_bench.config.models.scene import SceneConfig
    from scale_bench.config.models.simulation import SimulationConfig
    from scale_bench.tasks.common.placement import PlacementContext
    from scale_bench.tasks.sort_dolls_by_size.config import (
        SortDollsBySizeConfig,
    )
    from scale_bench.tasks.sort_dolls_by_size.task import SortDollsBySize

    robot = load_config(
        PROJECT_ROOT / "configs/robots/piper.yml",
        RobotConfig,
        asset_root=asset_root,
    )
    scene = load_config(
        PROJECT_ROOT / "configs/scene/default.yml",
        SceneConfig,
        asset_root=asset_root,
    )
    simulation = load_config(
        PROJECT_ROOT / "configs/sim/default.yml",
        SimulationConfig,
    )
    environment = load_config(
        PROJECT_ROOT / "configs/envs/default.yml",
        EnvironmentConfig,
    )
    task = SortDollsBySize(
        load_config(
            PROJECT_ROOT / "configs/tasks/sort_dolls_by_size.yml",
            SortDollsBySizeConfig,
            asset_root=asset_root,
        )
    )

    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(headless=True, enable_cameras=True)
    simulation_app = app_launcher.app
    env = None
    succeeded = False
    try:
        import torch

        env = create_env(
            left_robot_config=robot,
            right_robot_config=robot,
            scene_config=scene,
            simulation_config=simulation,
            environment_config=environment,
            task=task,
            base_seed=41,
            num_envs=2,
        )

        observation, info = env.reset()
        assert info["episode"] == {
            "env_ids": (0, 1),
            "task_id": "sort_dolls_by_size",
            "instruction": task.instruction,
            "layout_seeds": (41, 42),
        }
        _assert_policy_observation(observation["policy"], torch)
        _assert_descriptors(env.get_IO_descriptors)
        layouts = tuple(
            task.generate_layout(PlacementContext.from_scene_config(scene), seed)
            for seed in (41, 42)
        )
        _assert_task_layouts(env, layouts, torch)

        asset_name = next(iter(task.assets))
        asset = env.scene[asset_name]
        initial_poses = asset.data.root_pose_w.torch.clone()
        perturbed_poses = initial_poses.clone()
        perturbed_poses[0, 0] += 0.25
        perturbed_poses[1, 0] -= 0.25
        asset.write_root_pose_to_sim_index(root_pose=perturbed_poses)

        partial_observation, partial_info = env.reset(env_ids=(1,))
        assert partial_info["episode"] == {
            "env_ids": (1,),
            "task_id": "sort_dolls_by_size",
            "instruction": task.instruction,
            "layout_seeds": (42,),
        }
        _assert_policy_observation(partial_observation["policy"], torch)
        reset_poses = asset.data.root_pose_w.torch
        torch.testing.assert_close(reset_poses[0], perturbed_poses[0])
        torch.testing.assert_close(reset_poses[1], initial_poses[1])

        action = torch.zeros(
            (env.num_envs, env.action_manager.total_action_dim),
            dtype=torch.float32,
            device=env.device,
        )
        stepped_observation, _ = env.step(action)
        _assert_policy_observation(stepped_observation["policy"], torch)
        succeeded = True
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        if env is not None:
            env.close()
        if succeeded:
            print("SCALE_BENCH_HEADLESS_RUNTIME_OK", flush=True)
        simulation_app.close()


def _assert_policy_observation(policy: dict, torch) -> None:
    expected_terms = {
        "left_arm_joint_pos",
        "left_gripper_joint_pos",
        "right_arm_joint_pos",
        "right_gripper_joint_pos",
        "left_robot_camera_rgb",
        "left_robot_camera_depth",
        "right_robot_camera_rgb",
        "right_robot_camera_depth",
        "overhead_camera_rgb",
        "overhead_camera_depth",
    }
    assert set(policy) == expected_terms
    for name, value in policy.items():
        assert value.shape[0] == 2
        if name.endswith("_camera_rgb"):
            assert value.shape[-1] == 3
            assert value.dtype == torch.uint8
        elif name.endswith("_camera_depth"):
            assert value.shape[-1] == 1
            assert value.dtype == torch.float32


def _assert_descriptors(descriptors: dict) -> None:
    actions = descriptors["actions"]
    assert [item["name"] for item in actions] == [
        "left_arm",
        "left_gripper",
        "right_arm",
        "right_gripper",
    ]
    assert [item["slice"] for item in actions] == [
        [0, 6],
        [6, 7],
        [7, 13],
        [13, 14],
    ]
    policy = descriptors["observations"]["policy"]
    camera_descriptors = [
        item for item in policy if "_camera_" in item["name"]
    ]
    assert len(camera_descriptors) == 6
    for descriptor in camera_descriptors:
        assert descriptor["extras"]["layout"] == "HWC"
        assert len(descriptor["extras"]["intrinsic_matrix_px"]) == 9

    runtime = descriptors["runtime"]
    assert math.isclose(runtime["physics_frequency_hz"], 120.0)
    assert math.isclose(runtime["step_frequency_hz"], 30.0)
    assert math.isclose(runtime["render_frequency_hz"], 30.0)
    assert runtime["control_decimation"] == 4
    assert runtime["arm_action_mode"] == "joint_position"


def _assert_task_layouts(env, layouts: tuple, torch) -> None:
    for asset_name in layouts[0].assets:
        expected = torch.tensor(
            [
                (
                    *layout.assets[asset_name].position_m,
                    *layout.assets[asset_name].orientation_xyzw,
                )
                for layout in layouts
            ],
            device=env.device,
            dtype=env.scene.env_origins.dtype,
        )
        expected[:, :3] += env.scene.env_origins
        actual = env.scene[asset_name].data.root_pose_w.torch
        torch.testing.assert_close(actual, expected)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: _headless_runtime_smoke.py ASSET_ROOT")
    main(Path(sys.argv[1]).resolve())
