"""Positive controls for route completion and explicit missing-heading failure."""
import unittest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tools.heading_route_metrics import route_observation, stream_coverage, minimum_model_radius
import math
from tools.generate_heading_route_matrix import route_goals
from tools.archive_route_campaign import archive
import tempfile
import tarfile


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

    def test_archive_preserves_capture_bytes_and_excludes_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root/'capture'; source.mkdir()
            raw = b'stamp,value\r\n1,2\r\n'
            (source/'poses.csv').write_bytes(raw)
            (source/'runtime').mkdir()
            (source/'runtime'/'cache').write_text('not evidence')
            hashes = archive(source, root/'evidence')
            self.assertEqual(set(hashes), {'poses.csv'})
            with tarfile.open(root/'evidence'/'bundle.tar.gz', 'r:gz') as tar:
                self.assertEqual(tar.extractfile('poses.csv').read(), raw)
            first = (root/'evidence'/'bundle.tar.gz').read_bytes()
            archive(source, root/'evidence')
            self.assertEqual((root/'evidence'/'bundle.tar.gz').read_bytes(), first)

    def test_s_requires_second_turn_and_straight_has_separate_completion(self):
        poses = [dict(x_m=31, y_m=3, yaw_rad=1.57079632679, speed=1.6, stamp_s=3)]
        self.assertFalse(route_observation(poses, [], [], radius=3, kind='s')['completed_route'])
        poses.append(dict(x_m=41, y_m=6, yaw_rad=0., speed=1.6, stamp_s=9))
        self.assertTrue(route_observation(poses, [], [], radius=3, kind='s')['completed_route'])
        straight = [dict(x_m=57, y_m=0, yaw_rad=0., speed=1.8, stamp_s=9)]
        result = route_observation(straight, [], [], kind='straight')
        self.assertTrue(result['completed_route'])
        self.assertIsNone(result['completed_curve'])

    def test_stream_gaps_and_inner_joint_bound_are_not_hidden(self):
        frozen = stream_coverage([1., 1., 1.], 1., 3.)
        self.assertEqual(frozen['max_gap_s'], 2.)
        self.assertEqual(frozen['covered_fraction'], 0.)
        manual = minimum_model_radius(.94, math.radians(30), math.radians(30), .75)
        self.assertGreater(manual['inner_joint_radius_m'], 2.)
        self.assertGreater(manual['minimum_model_radius_m'], manual['central_command_radius_m'])
        auto = minimum_model_radius(.94, math.radians(18), math.radians(30), .75)
        self.assertAlmostEqual(auto['minimum_model_radius_m'], .94/math.tan(math.radians(18)))

    def test_a_physical_loop_is_not_a_successful_quarter_turn(self):
        yaws = [0., math.pi/2, math.pi, -math.pi/2, 0., math.pi/2]
        poses = [dict(x_m=28., y_m=0., yaw_rad=yaw, speed=1.6, stamp_s=i)
                 for i, yaw in enumerate(yaws)]
        poses.append(dict(x_m=32., y_m=10., yaw_rad=math.pi/2, speed=1.6, stamp_s=7))
        result = route_observation(poses, [], [])
        self.assertTrue(result['unwanted_winding'])
        self.assertFalse(result['completed_curve'])

    def test_small_repeated_oscillations_are_not_a_full_physical_loop(self):
        poses = [dict(x_m=57., y_m=0., yaw_rad=math.radians(i % 2),
                      speed=1.6, stamp_s=i) for i in range(250)]
        result = route_observation(poses, [], [], kind='straight')
        self.assertFalse(result['unwanted_winding'])

    def test_wider_tracking_does_not_hide_a_completed_physical_turn(self):
        poses = [dict(x_m=27., y_m=0., yaw_rad=0., speed=1.6, stamp_s=1),
                 dict(x_m=31., y_m=4.5, yaw_rad=math.radians(70), speed=1.6, stamp_s=2),
                 dict(x_m=32., y_m=5.2, yaw_rad=math.pi/2, speed=1.6, stamp_s=3),
                 dict(x_m=32., y_m=10., yaw_rad=math.pi/2, speed=1.6, stamp_s=4)]
        self.assertTrue(route_observation(poses, [], [])['completed_curve'])


if __name__ == '__main__':
    unittest.main()
