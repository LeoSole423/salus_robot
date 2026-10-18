"""Positive controls for route completion and explicit missing-heading failure."""
import unittest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tools.heading_route_metrics import route_observation
from tools.generate_heading_route_matrix import route_goals


class RouteMetricsTest(unittest.TestCase):
    def test_stationary_does_not_pass(self):
        result = route_observation([dict(x_m=0, y_m=0, yaw_rad=0,
                                         speed=0, stamp_s=1)], [], [])
        self.assertFalse(result['completed_curve'])
        self.assertFalse(result['heading_acquired_before_curve'])
        self.assertIsNone(result['curve_speed_median_mps'])

    def test_complete_quarter_requires_exit(self):
        poses = [dict(x_m=28, y_m=4, yaw_rad=1.57079632679, speed=1.6, stamp_s=3)]
        self.assertFalse(route_observation(poses, [], [])['completed_curve'])
        poses.append(dict(x_m=32, y_m=10, yaw_rad=1.57079632679,
                          speed=1.6, stamp_s=7))
        self.assertTrue(route_observation(poses, [], [])['completed_curve'])

    def test_generated_route_respects_mirror_and_manual_geometry(self):
        left = route_goals('quarter', 3., 1)
        right = route_goals('quarter', 3., -1)
        self.assertEqual(left[-1]['forward_m'], 31.)
        self.assertEqual(left[-1]['lateral_m'], -right[-1]['lateral_m'])
        self.assertEqual(left[0]['forward_m'], 20.)
        with self.assertRaises(ValueError):
            route_goals('quarter', 1., 1)
        s = route_goals('s', 3., 1)
        self.assertEqual(s[-1]['yaw_offset_rad'], 0.)


if __name__ == '__main__':
    unittest.main()
