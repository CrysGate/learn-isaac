"""Run the initialized runtime contract in an isolated Isaac Sim process."""

from __future__ import annotations

import math
import sys
import tempfile
import traceback
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))


def main(asset_root: Path) -> None:
    from scale_bench.api import create_env
    from scale_bench.config.loader import load_config
    from scale_bench.config.models.environment import EnvironmentConfig
    from scale_bench.config.models.recording import RecordingConfig
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
    recording_tmp = tempfile.TemporaryDirectory(prefix="scale_bench_recording_")
    dataset_path = Path(recording_tmp.name) / "runtime_smoke.hdf5"
    succeeded = False
    try:
        import torch

        env = create_env(
            left_robot_config=robot,
            right_robot_config=robot,
            scene_config=scene,
            simulation_config=simulation,
            environment_config=environment,
            recording_config=RecordingConfig(
                output_dir=recording_tmp.name,
                dataset_name="runtime_smoke",
            ),
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
        descriptors = env.get_IO_descriptors
        _assert_policy_observation(
            observation["policy"],
            descriptors,
            torch,
            check_image_content=True,
        )
        _assert_camera_frames_updated(env, (0, 1), torch)
        _assert_descriptors(descriptors)
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
        _assert_policy_observation(
            partial_observation["policy"], descriptors, torch
        )
        _assert_camera_frames_updated(env, (1,), torch)
        reset_poses = asset.data.root_pose_w.torch
        torch.testing.assert_close(reset_poses[0], perturbed_poses[0])
        torch.testing.assert_close(reset_poses[1], initial_poses[1])

        action = torch.zeros(
            (env.num_envs, env.action_manager.total_action_dim),
            dtype=torch.float32,
            device=env.device,
        )
        _fill_action_from_observation(
            action,
            partial_observation["policy"],
            descriptors["actions"],
        )
        left_arm_slice = slice(*descriptors["actions"][0]["slice"])
        left_gripper_slice = slice(*descriptors["actions"][1]["slice"])
        initial_left_joint = partial_observation["policy"][
            "left_arm_joint_pos"
        ][0, 0].clone()
        action[0, left_arm_slice.start] = initial_left_joint + 0.15
        action[:, left_gripper_slice] = 1.0

        for _ in range(4):
            stepped_observation, _ = env.step(action)

        _assert_policy_observation(
            stepped_observation["policy"], descriptors, torch
        )
        torch.testing.assert_close(env.action_manager.action, action)
        left_arm_term = env.action_manager.get_term("left_arm")
        left_gripper_term = env.action_manager.get_term("left_gripper")
        torch.testing.assert_close(
            left_arm_term.processed_actions[0, 0],
            action[0, left_arm_slice.start],
        )
        torch.testing.assert_close(
            left_gripper_term.processed_actions,
            torch.full_like(left_gripper_term.processed_actions, 0.05),
        )
        assert not torch.isclose(
            stepped_observation["policy"]["left_arm_joint_pos"][0, 0],
            initial_left_joint,
            atol=1.0e-4,
            rtol=0.0,
        )
        env.complete_episodes(
            env_ids=(0, 1),
            success=(True, False),
            demo_ids=(101, 102),
        )
        env.close()
        env = None
        _assert_recorded_dataset(dataset_path)
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
        recording_tmp.cleanup()


def _assert_policy_observation(
    policy: dict,
    descriptors: dict,
    torch,
    *,
    check_image_content: bool = False,
) -> None:
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
    policy_descriptors = {
        descriptor["name"]: descriptor
        for descriptor in descriptors["observations"]["policy"]
    }
    for name, value in policy.items():
        assert value.shape[0] == 2
        assert tuple(value.shape[1:]) == tuple(policy_descriptors[name]["shape"])
        if name.endswith("_camera_rgb"):
            assert tuple(value.shape[1:]) == (480, 640, 3)
            assert value.dtype == torch.uint8
            if check_image_content:
                assert torch.count_nonzero(value).item() > 0
        elif name.endswith("_camera_depth"):
            assert tuple(value.shape[1:]) == (480, 640, 1)
            assert value.dtype == torch.float32
            if check_image_content:
                valid_depth = torch.isfinite(value) & (value > 0.0)
                assert valid_depth.any().item()


def _assert_camera_frames_updated(env, env_ids: tuple[int, ...], torch) -> None:
    for sensor in env.scene.sensors.values():
        frames = sensor.frame.torch[list(env_ids)]
        assert torch.all(frames > 0).item()


def _fill_action_from_observation(
    action,
    policy: dict,
    action_descriptors: list[dict],
) -> None:
    observation_names = {
        "left_arm": "left_arm_joint_pos",
        "left_gripper": "left_gripper_joint_pos",
        "right_arm": "right_arm_joint_pos",
        "right_gripper": "right_gripper_joint_pos",
    }
    for descriptor in action_descriptors:
        action_slice = slice(*descriptor["slice"])
        source = policy[observation_names[descriptor["name"]]]
        action[:, action_slice] = source[:, : action_slice.stop - action_slice.start]


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


def _assert_recorded_dataset(dataset_path: Path) -> None:
    import h5py

    assert dataset_path.is_file()
    with h5py.File(dataset_path, "r") as dataset:
        assert set(dataset["data"]) == {"demo_101", "demo_102"}
        for demo_id, seed, success in (
            (101, 41, True),
            (102, 42, False),
        ):
            episode = dataset[f"data/demo_{demo_id}"]
            assert episode.attrs["seed"] == seed
            assert bool(episode.attrs["success"]) is success
            assert episode.attrs["num_samples"] == 4
            assert episode["actions"].shape == (4, 14)
            assert episode["processed_actions"].shape == (4, 14)
            assert set(episode["obs"]) == {
                "left_arm_joint_pos",
                "left_gripper_joint_pos",
                "right_arm_joint_pos",
                "right_gripper_joint_pos",
            }
            assert episode["obs/left_arm_joint_pos"].shape == (4, 6)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: _headless_runtime_smoke.py ASSET_ROOT")
    main(Path(sys.argv[1]).resolve())
