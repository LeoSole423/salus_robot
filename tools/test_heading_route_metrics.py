"""Positive controls for route completion and explicit missing-heading failure."""
import unittest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tools.heading_route_metrics import route_observation


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


if __name__ == '__main__':
    unittest.main()
