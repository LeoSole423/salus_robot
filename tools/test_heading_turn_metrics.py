import unittest
from tools.heading_turn_metrics import interpolate, continuity, recovery, angle
from tools.heading_turn_campaign import sequence, sensor_profile
import math
import tempfile
import json
import gzip
from pathlib import Path
from tools.heading_turn_metrics import analyze

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
    def test_capture_replays_identically_with_diagonal_covariances(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            root.joinpath('sequence.json').write_text(json.dumps(dict(start_sim_s=1.,sequence=[['straight_before',.5,1.,0.],['curve',.5,1.,0.],['straight_after',1.,1.,0.]])))
            rows=[]
            for i in range(41):
                t=1.+i*.05
                for topic,frame,child in [('/odom_raw','salus_ackermann/odom','salus_ackermann/base_footprint'),('/odometry/local','odom','base_footprint'),('/odometry/global','map','base_footprint')]:
                    cov=[0.]*36;cov[0]=.1;cov[35]=.2
                    truth=topic=='/odom_raw'
                    rows.append(dict(topic=topic,stamp_s=t,receive_sim_s=t,frame_id=frame,child_frame_id=child,x=t-1+(0 if truth else .1),y=0.,yaw_rad=0. if truth else math.radians(1),yaw_rate=0.,speed=1.,pose_covariance=cov))
            path=root/'samples.jsonl';path.write_text('\n'.join(map(json.dumps,rows)))
            first=analyze(root)
            for row in rows:
                values=row.pop('pose_covariance');row['pose_covariance_diagonal']=[values[i] for i in (0,7,14,21,28,35)]
            path.unlink();root.joinpath('analysis-samples.jsonl.gz').write_bytes(gzip.compress('\n'.join(map(json.dumps,rows)).encode()))
            second=analyze(root)
            self.assertEqual(first,second)
            m=second['localization']['/odometry/global']['all']
            self.assertAlmostEqual(m['heading_deg']['rms'],1.)
            self.assertAlmostEqual(m['position_m']['rms'],.1)
            self.assertFalse(second['instrumentation_valid'])  # missing IMU/TF

    def test_geometry_respects_product_radius(self):
        for speed in [.8,1.2,1.6,1.8]:
            for kind in ['right','left','s','half']:
                for _,_,v,w in sequence(kind,speed):
                    if w:self.assertGreaterEqual(abs(v/w),4.)
        self.assertTrue(sequence('right',1.8,2.9))
        self.assertTrue(sequence('right',1.8,1.65,manual_envelope=True))
        with self.assertRaises(ValueError):sequence('right',1.8,1.6,manual_envelope=True)
        with self.assertRaises(ValueError):sequence('right',1.8,2.8)
    def test_individual_profiles_do_not_change_other_channels(self):
        base=sensor_profile('clean'); gyro=sensor_profile('gyro')
        self.assertEqual(base['sim_drive_sensor_adapter'],gyro['sim_drive_sensor_adapter'])
        self.assertEqual(base['sim_gps_normalizer'],gyro['sim_gps_normalizer'])
        self.assertEqual(sensor_profile('latency300')['sim_gps_normalizer']['ros__parameters']['gnss.latency_s'],.3)

if __name__=='__main__':unittest.main()
