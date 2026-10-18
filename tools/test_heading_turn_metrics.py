import unittest
from tools.heading_turn_metrics import interpolate, continuity, recovery, angle
from tools.heading_turn_campaign import sequence, sensor_profile
import math

class MetricsTest(unittest.TestCase):
    def test_wrapped_interpolation_and_no_extrapolation(self):
        rows=[dict(stamp_s=1.,x=0.,y=0.,yaw_rad=math.radians(179)),dict(stamp_s=1.1,x=1.,y=0.,yaw_rad=math.radians(-179))]
        self.assertAlmostEqual(abs(interpolate(rows,1.05)['yaw_rad']),math.pi)
        self.assertIsNone(interpolate(rows,.9))
        rows[1]['stamp_s']=2.
        self.assertIsNone(interpolate(rows,1.5))
    def test_missing_stream_is_not_precision(self):
        self.assertEqual(continuity([],0,10)['max_gap_s'],10)
        rows=[dict(stamp_s=1.,receive_sim_s=5.)]*20
        c=continuity(rows,0,10)
        self.assertEqual(c['unique_stamps'],1)
        self.assertEqual(c['covered_fraction'],0)
        self.assertTrue(recovery([],0,10)['censored'])
    def test_recovery_requires_contiguous_data(self):
        self.assertTrue(recovery([(1,0,0),(3,0,0)],0,4)['censored'])
        good=[(i*.1,0,0) for i in range(25)]
        self.assertFalse(recovery(good,0,4)['censored'])
    def test_geometry_respects_product_radius(self):
        for speed in [.8,1.2,1.6]:
            for kind in ['right','left','s','half']:
                for _,_,v,w in sequence(kind,speed):
                    if w:self.assertGreaterEqual(abs(v/w),4.)
        with self.assertRaises(ValueError):sequence('right',1.2,3.)
    def test_individual_profiles_do_not_change_other_channels(self):
        base=sensor_profile('clean'); gyro=sensor_profile('gyro')
        self.assertEqual(base['sim_drive_sensor_adapter'],gyro['sim_drive_sensor_adapter'])
        self.assertEqual(base['sim_gps_normalizer'],gyro['sim_gps_normalizer'])
        self.assertEqual(sensor_profile('latency300')['sim_gps_normalizer']['ros__parameters']['gnss.latency_s'],.3)

if __name__=='__main__':unittest.main()
