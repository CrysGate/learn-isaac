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


class IsaacUsdAssetIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.world = subject.create_scene()
        cls.robots = None

    def test_robot_home_gripper_and_workspace(self) -> None:
        if self.__class__.robots is None:
            self.__class__.robots = subject.create_robots(self.world)
        report = subject.exercise_and_validate_robots(
            self.world, self.__class__.robots
        )
        for name in ("left", "right"):
            final = report["final_home"][name]
            self.assertLessEqual(
                final["maximum_home_error_rad"],
                subject.PIPER_HOME_TOLERANCE_RAD,
            )
            self.assertGreater(
                final["tool_forward_from_base_m"],
                subject.PIPER_WORKSPACE_FORWARD_MINIMUM_M,
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
