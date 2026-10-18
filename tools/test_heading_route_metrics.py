"""Positive controls for route completion and explicit missing-heading failure."""
import unittest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tools.heading_route_metrics import route_observation
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


if __name__ == '__main__':
    unittest.main()
