#!/usr/bin/env python3
"""Single test entry point for the dual-Piper simulation."""

from __future__ import annotations

import argparse
import math
import sys
import unittest

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
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    simulation_app = None
    if args.mode == "fast":
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(
            StaticAssetAndConstantTests
        )
    else:
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
