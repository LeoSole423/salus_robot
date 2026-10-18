"""Exercise the actual Humble service capture without a simulator."""
import subprocess
import tempfile
import threading
from pathlib import Path
import unittest
import yaml
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
except ImportError:
    rclpy=None

@unittest.skipIf(rclpy is None,'ROS Humble required')
class ParameterCaptureTest(unittest.TestCase):
    def test_effective_array_and_scalar_parameters(self):
        rclpy.init();executor=SingleThreadedExecutor()
        names=['sim_drive_sensor_adapter','sim_imu_from_odom','sim_gps_normalizer',
               'gps_course_heading','ekf_filter_node_local','ekf_filter_node_global',
               'map_gps_absolute_measurement']
        nodes=[Node(name) for name in names]
        for node in nodes:
            node.declare_parameter('array_probe',[1.,2.])
            node.declare_parameter('scalar_probe',.3)
            node.declare_parameter('unset_array',rclpy.Parameter.Type.DOUBLE_ARRAY)
            executor.add_node(node)
        thread=threading.Thread(target=executor.spin);thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                script=Path(__file__).with_name('dump_heading_parameters.py')
                result=subprocess.run(['python3',str(script),directory],capture_output=True,text=True,timeout=100)
                self.assertEqual(result.returncode,0,result.stderr)
                files=list(Path(directory).glob('*.yaml'));self.assertEqual(len(files),7)
                for path in files:
                    params=next(iter(yaml.safe_load(path.read_text()).values()))['ros__parameters']
                    self.assertEqual(params['array_probe'],[1.,2.])
                    self.assertEqual(params['scalar_probe'],.3)
                    self.assertIsNone(params['unset_array'])
        finally:
            executor.shutdown();thread.join()
            for node in nodes:node.destroy_node()
            rclpy.shutdown()

if __name__=='__main__':unittest.main()
