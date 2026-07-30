#!/usr/bin/env python3
"""Single test entry point for the dual-Piper simulation."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import dual_piper_sort as subject


class StaticAssetAndConstantTests(unittest.TestCase):
    def test_fixed_table_and_ground_constants(self) -> None:
        self.assertEqual(subject.TABLE_POSITION, (0.0, -0.05, 0.74))
        self.assertEqual(subject.TABLE_SIZE, (1.4, 1.1, 0.05))
        self.assertEqual(subject.TABLE_X_RANGE, (-0.70, 0.70))
        self.assertEqual(subject.TABLE_Y_RANGE, (-0.60, 0.50))
        self.assertEqual(subject.TABLE_TOP_Z, 0.765)
        self.assertEqual(subject.GROUND_POSITION, (0.0, 0.0, -0.05))
        self.assertEqual(subject.GROUND_SIZE, (6.0, 6.0, 0.1))

    def test_fixed_dual_piper_poses_and_wxyz_convention(self) -> None:
        self.assertEqual(subject.QUATERNION_ORDER, "wxyz")
        self.assertEqual(subject.LEFT_PIPER.base_pose.position, (-0.3, -0.45, 0.765))
        self.assertEqual(subject.RIGHT_PIPER.base_pose.position, (0.3, -0.45, 0.765))
        self.assertEqual(
            subject.LEFT_PIPER.base_pose.quaternion,
            subject.RIGHT_PIPER.base_pose.quaternion,
        )
        quaternion = subject.LEFT_PIPER.base_pose.quaternion
        self.assertTrue(math.isclose(sum(value * value for value in quaternion), 1.0))
        expected = 1.0 / math.sqrt(2.0)
        self.assertTrue(math.isclose(quaternion[0], expected, abs_tol=1.0e-12))
        self.assertTrue(math.isclose(quaternion[3], expected, abs_tol=1.0e-12))
        self.assertEqual(quaternion[1:3], (0.0, 0.0))

    def test_fixed_stand_overhead_camera_and_retracted_home(self) -> None:
        self.assertEqual(subject.CAMERA_STAND_POSITION, (0.0, -0.47, 0.765))
        self.assertEqual(
            subject.CAMERA_STAND_ORIENTATION,
            subject.normalize_quaternion((0.707, -0.707, 0.0, 0.0)),
        )
        self.assertEqual(subject.OVERHEAD_CAMERA_POSITION, (0.0, -0.41, 1.308))
        self.assertEqual(
            subject.OVERHEAD_CAMERA_USD_ORIENTATION,
            subject.normalize_quaternion((0.9659258, 0.2588190, 0.0, 0.0)),
        )
        self.assertEqual(
            subject.PIPER_HOME_JOINT_POSITION,
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def test_selected_doll_metadata(self) -> None:
        dolls = subject.get_doll_specs()
        self.assertEqual([doll.asset_id for doll in dolls], [f"{i:05d}" for i in range(5)])
        for actual, expected in zip(
            (doll.height for doll in dolls), (0.13, 0.11, 0.09, 0.07, 0.05)
        ):
            self.assertTrue(math.isclose(actual, expected, abs_tol=1.0e-9))
        self.assertEqual({doll.uuid for doll in dolls}, {subject.MATRYOSHKA_UUID})
        self.assertTrue(all(doll.mass == 0.05 for doll in dolls))
        self.assertTrue(all(doll.friction == 0.45 for doll in dolls))
        self.assertEqual(
            subject.MATRYOSHKA_SORT_ORDER,
            ("00004", "00003", "00002", "00001", "00000"),
        )

    def test_doll_targets_are_centred_size_aware_and_safe(self) -> None:
        specs = {spec.asset_id: spec for spec in subject.get_doll_specs()}
        targets = subject.compute_doll_target_layout()
        self.assertEqual(
            [target.asset_id for target in targets],
            list(subject.MATRYOSHKA_SORT_ORDER),
        )
        self.assertTrue(all(target.pose.position[0] == 0.0 for target in targets))
        first = targets[0]
        last = targets[-1]
        occupied_min_y = (
            first.pose.position[1] - specs[first.asset_id].footprint_radius
        )
        occupied_max_y = (
            last.pose.position[1] + specs[last.asset_id].footprint_radius
        )
        self.assertTrue(
            math.isclose(
                (occupied_min_y + occupied_max_y) / 2.0,
                subject.TABLE_POSITION[1],
                abs_tol=1.0e-12,
            )
        )
        for first, second in zip(targets, targets[1:]):
            required_distance = (
                specs[first.asset_id].footprint_radius
                + specs[second.asset_id].footprint_radius
                + subject.MATRYOSHKA_TARGET_GAP
            )
            self.assertTrue(
                math.isclose(
                    second.pose.position[1] - first.pose.position[1],
                    required_distance,
                    abs_tol=1.0e-12,
                )
            )

    def test_seeded_doll_layout_is_reproducible_and_non_overlapping(self) -> None:
        first = subject.sample_initial_doll_layout(20260730)
        second = subject.sample_initial_doll_layout(20260730)
        different = subject.sample_initial_doll_layout(20260731)
        self.assertEqual(first, second)
        self.assertNotEqual(first, different)
        report = subject.validate_initial_doll_layout(first)
        self.assertGreaterEqual(
            report["minimum_pair_surface_gap_m"],
            subject.MATRYOSHKA_INITIAL_GAP,
        )
        self.assertGreaterEqual(
            report["minimum_table_edge_clearance_m"],
            subject.MATRYOSHKA_TABLE_EDGE_CLEARANCE,
        )
        self.assertFalse(report["already_sorted"])

    def test_piper_sim_and_command_joint_mapping(self) -> None:
        self.assertEqual(
            subject.PIPER_DOF_NAMES,
            subject.PIPER_ARM_JOINT_NAMES + subject.PIPER_GRIPPER_JOINT_NAMES,
        )
        self.assertEqual(
            subject.PIPER_COMMAND_JOINT_NAMES,
            subject.PIPER_ARM_JOINT_NAMES + ("gripper_joint",),
        )
        self.assertEqual(len(subject.PIPER_HOME_DOF_POSITION), 8)

    def test_three_d435_pinhole_contracts(self) -> None:
        self.assertEqual(len(subject.CAMERA_SPECS), 3)
        self.assertEqual(
            {spec.name for spec in subject.CAMERA_SPECS},
            {
                subject.LEFT_WRIST_CAMERA_NAME,
                subject.RIGHT_WRIST_CAMERA_NAME,
                subject.OVERHEAD_CAMERA_NAME,
            },
        )
        for spec in subject.CAMERA_SPECS:
            intrinsics = subject.camera_intrinsics(spec)
            self.assertEqual(spec.resolution, (640, 480))
            self.assertEqual(spec.frequency_hz, 30)
            self.assertTrue(math.isclose(intrinsics[0][0], intrinsics[1][1]))
            self.assertEqual(intrinsics[0][2], 320.0)
            self.assertEqual(intrinsics[1][2], 240.0)
        self.assertTrue(
            subject.LEFT_WRIST_CAMERA_PRIM_PATH.startswith(
                subject.LEFT_PIPER_PRIM_PATH
            )
        )
        self.assertTrue(
            subject.RIGHT_WRIST_CAMERA_PRIM_PATH.startswith(
                subject.RIGHT_PIPER_PRIM_PATH
            )
        )
        self.assertTrue(
            subject.OVERHEAD_CAMERA_PRIM_PATH.startswith(
                subject.CAMERA_STAND_PRIM_PATH
            )
        )

    def test_static_asset_and_urdf_audit(self) -> None:
        report = subject.validate_static_assets()
        joints = report["urdf"]["joints"]
        self.assertEqual(tuple(joints)[:6], subject.PIPER_ARM_JOINT_NAMES)
        self.assertEqual(joints["joint8"]["mimic"], "gripper_joint")
        self.assertEqual(joints["gripper_center_fixed"]["child"], "gripper_center")
        self.assertEqual(joints["camera_fixed"]["child"], "camera")

    def test_simulation_rates_have_integer_substeps(self) -> None:
        self.assertEqual(
            subject.PHYSICS_FREQUENCY_HZ % subject.CONTROL_FREQUENCY_HZ, 0
        )
        self.assertEqual(
            subject.PHYSICS_FREQUENCY_HZ % subject.RENDER_FREQUENCY_HZ, 0
        )
        self.assertEqual(subject.CONTROL_FREQUENCY_HZ, subject.CAMERA_FREQUENCY_HZ)

    def test_curobo_config_uses_six_arm_joints_and_attachment_slots(self) -> None:
        config = subject.build_curobo_robot_config()["robot_cfg"]["kinematics"]
        self.assertEqual(
            config["cspace"]["joint_names"],
            list(subject.PIPER_ARM_JOINT_NAMES),
        )
        self.assertEqual(
            config["lock_joints"],
            {"gripper_joint": subject.PIPER_OPEN_GRIPPER_POSITION},
        )
        self.assertEqual(
            config["extra_collision_spheres"][
                subject.CUROBO_ATTACHED_OBJECT_LINK
            ],
            subject.CUROBO_ATTACHED_OBJECT_SPHERES,
        )
        self.assertIn(
            subject.CUROBO_ATTACHED_OBJECT_LINK,
            config["collision_link_names"],
        )

    def test_dual_arm_assignment_and_sorted_state_contract(self) -> None:
        layout = subject.sample_initial_doll_layout(20260730)
        poses = {placement.asset_id: placement.pose for placement in layout}
        assignments = subject.assign_dolls_to_robots(poses)
        self.assertEqual(set(assignments.values()), {"left", "right"})
        self.assertEqual(assignments["00000"], "right")
        self.assertEqual(assignments["00001"], "left")

        states = {}
        for placement in subject.compute_doll_target_layout():
            states[placement.asset_id] = {
                "position_m": list(placement.pose.position),
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                "linear_velocity_m_s": [0.0, 0.0, 0.0],
                "angular_velocity_rad_s": [0.0, 0.0, 0.0],
                "linear_speed_m_s": 0.0,
                "angular_speed_rad_s": 0.0,
                "upright_tilt_degrees": 0.0,
                "estimated_bottom_z_m": subject.TABLE_TOP_Z,
            }
        report = subject.validate_sorted_doll_states(states)
        self.assertEqual(report["maximum_target_error_m"], 0.0)
        self.assertEqual(
            report["order_small_to_large"],
            list(subject.MATRYOSHKA_SORT_ORDER),
        )

    def test_hdf5_schema_synchronizes_state_actions_and_three_rgbd_streams(
        self,
    ) -> None:
        import h5py
        import numpy as np

        layout = subject.sample_initial_doll_layout(20260730)
        metadata = subject.build_episode_metadata(
            episode_id="fast-schema-test",
            seed=20260730,
            planner_seed=1,
            sampled_layout=layout,
        )
        robot_position = np.asarray(
            [subject.PIPER_HOME_DOF_POSITION] * 2,
            dtype=np.float32,
        )
        robot_action = np.concatenate(
            (robot_position[:, :6], robot_position[:, 6:7]),
            axis=1,
        )
        initial_object_pose = np.asarray(
            [
                [*placement.pose.position, *placement.pose.quaternion]
                for placement in layout
            ],
            dtype=np.float32,
        )
        target_object_pose = np.asarray(
            [
                [*placement.pose.position, *placement.pose.quaternion]
                for placement in subject.compute_doll_target_layout()
            ],
            dtype=np.float32,
        )
        initial_state = {
            "robots": {
                "joint_position": robot_position,
                "joint_velocity": np.zeros((2, 8), dtype=np.float32),
                "joint_action": robot_action,
            },
            "objects": {
                "pose": initial_object_pose,
                "linear_velocity": np.zeros((5, 3), dtype=np.float32),
                "angular_velocity": np.zeros((5, 3), dtype=np.float32),
            },
            "targets": {"object_pose": target_object_pose},
        }
        cameras = {
            camera_name: {
                "rgb": np.zeros((480, 640, 3), dtype=np.uint8),
                "depth": np.ones((480, 640), dtype=np.float32),
                "rendering_time_s": subject.CONTROL_DT,
                "world_pose": np.asarray(
                    [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
                    dtype=np.float32,
                ),
                "world_to_camera": np.eye(4, dtype=np.float32),
            }
            for camera_name in subject.EPISODE_CAMERA_NAMES
        }
        frame = {
            "simulation_time_s": subject.CONTROL_DT,
            "world_time_s": 1.0,
            "robots": {
                "joint_position": robot_position,
                "joint_velocity": np.zeros((2, 8), dtype=np.float32),
                "joint_action": robot_action,
                "end_effector_world_pose": np.zeros(
                    (2, 7), dtype=np.float32
                ),
            },
            "objects": {
                "world_pose": initial_object_pose,
                "linear_velocity": np.zeros((5, 3), dtype=np.float32),
                "angular_velocity": np.zeros((5, 3), dtype=np.float32),
            },
            "task": {
                "phase": "initial_hold",
                "operator": "",
                "object_id": "",
            },
            "control": {
                "grasp_event_code": np.zeros(2, dtype=np.int8),
                "grasp_event_object_index": np.full(
                    2, -1, dtype=np.int8
                ),
                "grasp_event_relative_pose": np.full(
                    (2, 7), np.nan, dtype=np.float32
                ),
            },
            "cameras": cameras,
        }
        with tempfile.TemporaryDirectory() as temporary_directory:
            episode_path = Path(temporary_directory) / "episode.partial.h5"
            writer = subject.Hdf5EpisodeWriter(
                episode_path,
                metadata=metadata,
                initial_state=initial_state,
            )
            try:
                writer.append_frame(frame)
                writer.finish_expert(success=True, summary={"success": True})
            finally:
                writer.close()
            report = subject.validate_episode_hdf5(episode_path)
            self.assertEqual(report["schema_version"], "1.0.0")
            self.assertEqual(report["frame_count"], 1)
            self.assertTrue(report["expert_success"])
            self.assertFalse(report["replay_success"])
            self.assertFalse(report["accepted"])
            with self.assertRaisesRegex(
                ValueError, "has not passed replay acceptance"
            ):
                subject.validate_episode_hdf5(
                    episode_path,
                    require_accepted=True,
                )
            with h5py.File(episode_path, "r") as episode:
                for camera_name in subject.EPISODE_CAMERA_NAMES:
                    base = f"frames/cameras/{camera_name}"
                    self.assertEqual(
                        episode[f"{base}/rgb"].shape,
                        (1, 480, 640, 3),
                    )
                    self.assertEqual(
                        episode[f"{base}/depth"].shape,
                        (1, 480, 640),
                    )
                self.assertEqual(
                    episode["frames/robots/joint_action"].shape,
                    (1, 2, 7),
                )
                self.assertEqual(
                    episode["frames/objects/world_pose"].shape,
                    (1, 5, 7),
                )
            compatibility = subject.validate_episode_replay_compatibility(
                metadata
            )
            self.assertEqual(
                compatibility["object_ids"],
                list(subject.EPISODE_OBJECT_IDS),
            )
            subject.update_episode_replay_result(
                episode_path,
                success=True,
                summary={"success": True, "planner_invocations": 0},
            )
            accepted = subject.validate_episode_hdf5(
                episode_path,
                require_accepted=True,
            )
            self.assertTrue(accepted["accepted"])
            subject.update_episode_replay_result(
                episode_path,
                success=False,
                failure_reason="intentional rejection test",
            )
            rejected = subject.validate_episode_hdf5(episode_path)
            self.assertFalse(rejected["accepted"])
            self.assertFalse(rejected["replay_success"])
            self.assertEqual(
                rejected["failure_reason"],
                "intentional rejection test",
            )

    def test_task_uses_only_the_two_allowed_python_files(self) -> None:
        task_python_files = sorted(
            path.name
            for path in subject.REPOSITORY_ROOT.glob("dual_piper*.py")
        )
        self.assertEqual(
            task_python_files,
            ["dual_piper_sort.py"],
        )
        self.assertTrue(
            (subject.REPOSITORY_ROOT / "test_dual_piper_sort.py").is_file()
        )


class PublicDemoReplayIntegrationTest(unittest.TestCase):
    result: dict | None = None

    def test_public_demo_rebuilds_and_replays_accepted_episode(self) -> None:
        import h5py

        self.assertIsNotNone(self.__class__.result)
        result = self.__class__.result
        assert result is not None
        self.assertIn("DUAL_PIPER_DEMO_ACCEPTED", result["stdout"])
        schema = result["schema"]
        self.assertTrue(schema["expert_success"])
        self.assertTrue(schema["replay_success"])
        self.assertTrue(schema["accepted"])
        self.assertGreater(schema["frame_count"], 0)
        with h5py.File(result["episode_path"], "r") as episode:
            replay = json.loads(
                episode["results/replay_summary_json"][()].decode()
            )
        self.assertEqual(replay["planner_invocations"], 0)
        self.assertTrue(replay["success"])
        self.assertLessEqual(
            replay["final_validation"]["maximum_target_error_m"],
            subject.MATRYOSHKA_POSITION_TOLERANCE,
        )
        for robot in replay["final_robots"].values():
            self.assertLessEqual(
                robot["maximum_home_error_rad"],
                subject.PIPER_HOME_TOLERANCE_RAD,
            )


def _find_accepted_episode(
    output_dir: Path,
    explicit_episode: Path | None,
) -> Path | None:
    candidates = (
        [explicit_episode.resolve()]
        if explicit_episode is not None
        else sorted(
            (output_dir / "episodes").glob("*.h5"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
    )
    for candidate in candidates:
        try:
            subject.validate_episode_hdf5(
                candidate,
                require_accepted=True,
            )
        except (OSError, ValueError):
            if explicit_episode is not None:
                raise
            continue
        return candidate
    return None


def _prepare_public_demo_replay(
    *,
    headless: bool,
    episode: Path | None,
    output_dir: Path,
) -> dict:
    output_dir = output_dir.resolve()
    accepted_episode = _find_accepted_episode(output_dir, episode)
    common = [
        sys.executable,
        str(subject.REPOSITORY_ROOT / "dual_piper_sort.py"),
        "--output-dir",
        str(output_dir),
        "--seed",
        "20260730",
        "--planner-seed",
        "1",
    ]
    if headless:
        common.append("--headless")
    timeout_s = 2.0 * subject.EPISODE_MAX_RUNTIME_S + 120.0
    if accepted_episode is None:
        collection = subprocess.run(
            [*common, "--mode", "demo"],
            cwd=subject.REPOSITORY_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        if collection.returncode != 0:
            raise RuntimeError(
                "Public demo collection failed with "
                f"{collection.returncode}: {collection.stdout[-8000:]}"
            )
        accepted_episode = _find_accepted_episode(output_dir, None)
        if accepted_episode is None:
            raise RuntimeError(
                "Public demo returned success without an accepted HDF5"
            )
    replay = subprocess.run(
        [
            *common,
            "--mode",
            "demo",
            "--episode",
            str(accepted_episode),
        ],
        cwd=subject.REPOSITORY_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout_s,
        check=False,
    )
    if replay.returncode != 0:
        raise RuntimeError(
            "Public demo replay failed with "
            f"{replay.returncode}: {replay.stdout[-8000:]}"
        )
    return {
        "episode_path": str(accepted_episode),
        "stdout": replay.stdout,
        "schema": subject.validate_episode_hdf5(
            accepted_episode,
            require_accepted=True,
        ),
    }


class IsaacUsdAssetIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.world = subject.create_scene()
        cls.robots = None
        cls.cameras = None
        cls.dolls = None

    @classmethod
    def ensure_robots(cls):
        if cls.robots is None:
            cls.robots = subject.create_robots(cls.world)
        return cls.robots

    def test_camera_rgb_depth_calibration_and_coverage(self) -> None:
        self.ensure_robots()
        if self.__class__.cameras is None:
            self.__class__.cameras = subject.create_cameras(self.world)
        report = subject.validate_and_capture_cameras(
            self.world, self.__class__.cameras
        )
        self.assertEqual(
            set(report),
            {
                subject.LEFT_WRIST_CAMERA_NAME,
                subject.RIGHT_WRIST_CAMERA_NAME,
                subject.OVERHEAD_CAMERA_NAME,
            },
        )
        for camera_report in report.values():
            self.assertEqual(camera_report["rgb"]["shape"], [480, 640, 3])
            self.assertEqual(camera_report["rgb"]["dtype"], "uint8")
            self.assertEqual(camera_report["depth"]["shape"], [480, 640])
            self.assertEqual(camera_report["depth"]["dtype"], "float32")
            self.assertEqual(camera_report["depth"]["unit"], "m")
            self.assertAlmostEqual(
                camera_report["observed_period_s"],
                1.0 / 30.0,
                delta=5.0e-3,
            )

    def test_doll_rigid_bodies_mass_friction_and_stability(self) -> None:
        self.ensure_robots()
        layout = subject.sample_initial_doll_layout(20260730)
        if self.__class__.dolls is None:
            self.__class__.dolls = subject.create_dolls(self.world, layout)
        report = subject.settle_and_validate_dolls(
            self.world, self.__class__.dolls
        )
        self.assertEqual(
            set(report["stable_states"]),
            {f"{index:05d}" for index in range(5)},
        )
        self.assertGreaterEqual(
            report["settling"]["stable_consecutive_steps"],
            subject.MATRYOSHKA_STABLE_CONSECUTIVE_STEPS,
        )
        for asset_id, physics in report["physics"].items():
            self.assertAlmostEqual(physics["mass_kg"], 0.05, delta=1.0e-7)
            self.assertAlmostEqual(
                physics["dynamic_friction"], 0.45, delta=1.0e-7
            )
            self.assertTrue(physics["collision_prim_paths"], asset_id)
        for state in report["stable_states"].values():
            self.assertLessEqual(
                state["linear_speed_m_s"],
                subject.MATRYOSHKA_LINEAR_SPEED_TOLERANCE,
            )
            self.assertLessEqual(
                state["angular_speed_rad_s"],
                subject.MATRYOSHKA_ANGULAR_SPEED_TOLERANCE,
            )
            self.assertLessEqual(
                state["upright_tilt_degrees"],
                subject.MATRYOSHKA_UPRIGHT_TOLERANCE_DEGREES,
            )

    def test_robot_home_gripper_and_workspace(self) -> None:
        self.ensure_robots()
        report = subject.exercise_and_validate_robots(
            self.world, self.__class__.robots
        )
        for name in ("left", "right"):
            final = report["final_home"][name]
            self.assertLessEqual(
                final["maximum_home_error_rad"],
                subject.PIPER_HOME_TOLERANCE_RAD,
            )
            self.assertGreaterEqual(
                final["tool_forward_from_base_m"],
                subject.PIPER_HOME_TOOL_FORWARD_RANGE_M[0],
            )
            self.assertLessEqual(
                final["tool_forward_from_base_m"],
                subject.PIPER_HOME_TOOL_FORWARD_RANGE_M[1],
            )
        self.assertLess(
            max(report["gripper_cycle"]["closed_finger_separation_m"].values()),
            0.003,
        )

    def test_scene_smoke(self) -> None:
        report = subject.validate_scene(self.world)
        self.assertEqual(report["active_lights"], [subject.HDR_DOME_PRIM_PATH])
        self.assertEqual(report["table"]["bbox"]["max"][2], subject.TABLE_TOP_Z)
        self.assertGreater(report["camera_stand"]["collision_prim_count"], 0)

    def test_usd_asset_audit(self) -> None:
        report = subject.run_asset_audit()
        self.assertTrue(report["usd"]["doll_mass_and_friction_override_required"])
        self.assertTrue(report["usd"]["piper_camera_helper_is_not_sensor"])


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("fast", "integration"), default="fast")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--episode", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=subject.REPOSITORY_ROOT / "dual_piper_output",
    )
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    simulation_app = None
    if args.mode == "fast":
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(
            StaticAssetAndConstantTests
        )
    else:
        PublicDemoReplayIntegrationTest.result = (
            _prepare_public_demo_replay(
                headless=args.headless,
                episode=args.episode,
                output_dir=args.output_dir,
            )
        )
        # Isaac-supplied pxr bindings are available only after the application
        # starts.  Keep it alive until all assertions and runner output finish.
        from isaacsim import SimulationApp

        simulation_app = SimulationApp({"headless": args.headless})
        suite = unittest.TestSuite(
            (
                unittest.defaultTestLoader.loadTestsFromTestCase(
                    StaticAssetAndConstantTests
                ),
                unittest.defaultTestLoader.loadTestsFromTestCase(
                    PublicDemoReplayIntegrationTest
                ),
                unittest.defaultTestLoader.loadTestsFromTestCase(
                    IsaacUsdAssetIntegrationTest
                ),
            )
        )
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if result.wasSuccessful() and simulation_app is not None:
        sys.stdout.flush()
        sys.stderr.flush()
        simulation_app.close()
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
