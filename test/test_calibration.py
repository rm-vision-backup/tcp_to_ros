#!/usr/bin/env python3
import math
import unittest
from types import SimpleNamespace

from tcp_to_ros.calibration import (GlobalPoint, angular_error, circular_mean,
                                    headings_within, median_global,
                                    task_point_to_local_enu, task_vector_to_enu,
                                    validate_calibration_config)


class CalibrationMathTest(unittest.TestCase):
    def test_global_coordinate_median_is_axis_wise(self):
        value = median_global([
            GlobalPoint(30.0, 110.0, 10.0),
            GlobalPoint(32.0, 108.0, 30.0),
            GlobalPoint(31.0, 109.0, 20.0),
        ])
        self.assertEqual(value, GlobalPoint(31.0, 109.0, 20.0))

    def test_circular_mean_wraps_across_pi(self):
        mean = circular_mean((math.radians(179), math.radians(-179)))
        self.assertLess(angular_error(mean, math.pi), math.radians(2))
        self.assertTrue(headings_within((math.radians(179), math.radians(-179)), mean, math.radians(30)))

    def test_right_forward_axes_rotate_with_heading(self):
        self.assertAlmostEqual(task_vector_to_enu(5.0, 10.0, 0.0)[0], 10.0)
        self.assertAlmostEqual(task_vector_to_enu(5.0, 10.0, 0.0)[1], -5.0)
        x, y, z = task_point_to_local_enu(15.0, 15.0, 3.0, 0.0)
        self.assertEqual((x, y, z), (15.0, -15.0, 3.0))

    def test_task_origin_aligns_with_ekf_origin(self):
        # mission_anchor=[0,0,0]：任务原点（右/前 0,0）与 EKF 原点（A01-A06
        # 中位数，set_gp_origin 冻结）对齐，任务点只按参考航向旋转。
        # A01 prepare 目标 (17.5 右, 50 前, 12) -> EKF ENU (50, -17.5, 12)。
        x, y, z = task_point_to_local_enu(17.5, 50.0, 12.0, 0.0, anchor=(0.0, 0.0, 0.0))
        self.assertAlmostEqual(x, 50.0)
        self.assertAlmostEqual(y, -17.5)
        self.assertAlmostEqual(z, 12.0)


class CalibrationConfigTest(unittest.TestCase):
    """implementation_plan_26091101 §3.1/§3.2：UAVn 校准集合启动校验。"""

    @staticmethod
    def _identities(targets, disabled=()):
        return [SimpleNamespace(exec_target=name, enabled=name not in disabled)
                for name in targets]

    def test_accepts_required_subset(self):
        calibration, known = validate_calibration_config(
            ["UAV1", "UAV2", "UAV3"],
            ["UAV1", "UAV2", "UAV3", "UAV4"],
            self._identities(["UAV1", "UAV2", "UAV3", "UAV4"]))
        self.assertEqual(calibration, ("UAV1", "UAV2", "UAV3"))
        self.assertEqual(known, ("UAV1", "UAV2", "UAV3", "UAV4"))

    def test_accepts_without_identity_file(self):
        calibration, known = validate_calibration_config(["UAV1"], ["UAV1", "UAV2"])
        self.assertEqual(calibration, ("UAV1",))
        self.assertEqual(known, ("UAV1", "UAV2"))

    def test_rejects_empty_sets(self):
        with self.assertRaises(ValueError):
            validate_calibration_config([], ["UAV1"])
        with self.assertRaises(ValueError):
            validate_calibration_config(["UAV1"], [])

    def test_rejects_duplicates(self):
        with self.assertRaises(ValueError):
            validate_calibration_config(["UAV1", "UAV1"], ["UAV1"])
        with self.assertRaises(ValueError):
            validate_calibration_config(["UAV1"], ["UAV1", "UAV1"])

    def test_rejects_calibration_outside_all(self):
        with self.assertRaises(ValueError):
            validate_calibration_config(["UAV2"], ["UAV1"])

    def test_rejects_missing_identity_member(self):
        with self.assertRaises(ValueError):
            validate_calibration_config(["UAV1"], ["UAV1", "UAV2"],
                                        self._identities(["UAV1"]))

    def test_rejects_disabled_identity_member(self):
        with self.assertRaises(ValueError):
            validate_calibration_config(
                ["UAV1"], ["UAV1", "UAV2"],
                self._identities(["UAV1", "UAV2"], disabled=("UAV2",)))

    def test_rejects_bad_or_duplicate_exec_target(self):
        with self.assertRaises(ValueError):
            validate_calibration_config(
                ["UAV1"], ["UAV1"],
                [SimpleNamespace(exec_target="", enabled=True)])
        with self.assertRaises(ValueError):
            validate_calibration_config(
                ["UAV1"], ["UAV1"],
                [SimpleNamespace(exec_target="UAV1", enabled=True),
                 SimpleNamespace(exec_target="UAV1", enabled=True)])


if __name__ == "__main__":
    unittest.main()
