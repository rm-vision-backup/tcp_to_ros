#!/usr/bin/env python3
import math
import unittest

from tcp_to_ros.calibration import (GlobalPoint, angular_error, circular_mean,
                                    headings_within, median_global,
                                    task_point_to_local_enu, task_vector_to_enu)


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
        self.assertEqual((x, y, z), (10.0, -5.0, 3.0))


if __name__ == "__main__":
    unittest.main()
