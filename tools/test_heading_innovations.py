import math
from pathlib import Path
import tempfile
import unittest
from analyze_heading_innovations import parse_correction, parse_log, values
from prepare_heading_debug import prepare


def fixture(before=0.2, after=0.3):
    state = [0.0]*15
    state[5] = before
    corrected = list(state)
    corrected[5] = after
    gain = [0.0]*30
    gain[10:12] = [0.5,0.25]
    covariance = [0.0]*225
    covariance[80] = 0.03
    def vector(v): return '['+' '.join(map(str,v))+']'
    return ('------ FilterBase::processMeasurement (odom1_pose) ------\n'
            'Measurement time is 12340000000, last measurement time is 12300000000\n'
            '---------------------- Ekf::correct ----------------------\n'
            f'State is:\n{vector(state)}\n'
            'Update indices are:\n[0 1]\n'
            f'Kalman gain subset is:\n{vector(gain)}\n'
            'Innovation is:\n[0.1 0.2]\n'
            f'Corrected full state is:\n{vector(corrected)}\n'
            f'Corrected full estimate error covariance is:\n{vector(covariance)}\n'
            '------ /FilterBase::processMeasurement (odom1_pose) ------\n')


class NativeCorrections(unittest.TestCase):
    def test_position_update_changes_yaw_and_reconciles_gain_innovation(self):
        event = parse_correction(fixture())
        self.assertAlmostEqual(event['stamp_s'],12.34)
        self.assertAlmostEqual(event['yaw_delta_rad'],0.1)
        self.assertAlmostEqual(event['k_innovation_yaw_rad'],0.1)
        self.assertAlmostEqual(event['yaw_contributions_rad']['0'],0.05)
        self.assertAlmostEqual(event['yaw_contributions_rad']['1'],0.05)
        self.assertAlmostEqual(event['reconciliation_error_rad'],0)

    def test_angle_wrap_is_not_reported_as_full_revolution(self):
        event = parse_correction(fixture(math.pi-0.05,-math.pi+0.05))
        self.assertAlmostEqual(event['yaw_delta_rad'],0.1)

    def test_native_fixed_width_exponent_adjacent_zero(self):
        self.assertEqual(values('-2.2353e-1000           '),[-2.2353e-100,0])

    def test_incomplete_or_malformed_is_observable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'native.log'
            path.write_text(fixture().replace('Innovation is:', 'Missing innovation:')+fixture()[:150])
            events,stats = parse_log(path)
            self.assertEqual(events,[])
            self.assertEqual(stats['malformed'],1)
            self.assertEqual(stats['incomplete'],1)

    def test_debug_overlay_keeps_numeric_product_config(self):
        import yaml
        with tempfile.TemporaryDirectory() as directory:
            prepare(directory)
            for name in ('local','global'):
                source=yaml.safe_load(Path(f'src/salus_localization/config/localization_{name}_sim.yaml').read_text())
                actual=yaml.safe_load((Path(directory)/f'{name}-debug.yaml').read_text())
                params=actual[f'ekf_filter_node_{name}']['ros__parameters']
                self.assertTrue(params.pop('debug'))
                params.pop('debug_out_file')
                self.assertEqual(actual,source)
            for name in ('no_gps_position','compensated_gps','delayed_gps_reference'):
                actual=yaml.safe_load((Path(directory)/f'{name}-debug.yaml').read_text())
                self.assertFalse(actual[f'ekf_global_{name}']['ros__parameters']['publish_tf'])


class LeverGeometry(unittest.TestCase):
    def test_known_antenna_mount_is_removed_in_straight(self):
        from heading_gps_lever_diagnostic import compensate_position
        x,y=compensate_position(10.66,20.20,0)
        self.assertAlmostEqual(x,10)
        self.assertAlmostEqual(y,20)

    def test_known_mount_rotates_with_vehicle_in_left_turn(self):
        from heading_gps_lever_diagnostic import compensate_position
        x,y=compensate_position(9.80,20.66,math.pi/2)
        self.assertAlmostEqual(x,10)
        self.assertAlmostEqual(y,20)


if __name__ == '__main__':
    unittest.main()
