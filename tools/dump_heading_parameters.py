"""Capture effective parameters through services, waiting for discovery once."""
import sys
import os
from array import array
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import ListParameters, GetParameters
import yaml

NODES={
 'salus_controller':'controller-parameters.yaml',
 'sim_drive_sensor_adapter':'sim_drive_sensor_adapter-parameters.yaml',
 'sim_imu_from_odom':'sim_imu_from_odom-parameters.yaml',
 'sim_gps_normalizer':'sim_gps_normalizer-parameters.yaml',
 'gps_course_heading':'gps-parameters.yaml',
 'ekf_filter_node_local':'local-ekf-parameters.yaml',
 'ekf_filter_node_global':'global-ekf-parameters.yaml',
 'map_gps_absolute_measurement':'gps-map-parameters.yaml'}

if os.environ.get('HEADING_NAVIGATION_PARAMETERS') == 'true':
 NODES.update(controller_server='nav-controller-parameters.yaml',
              planner_server='nav-planner-parameters.yaml')

rclpy.init();node=Node('heading_parameter_capture')
try:
 for name,filename in NODES.items():
    listing=node.create_client(ListParameters,'/'+name+'/list_parameters')
    getting=node.create_client(GetParameters,'/'+name+'/get_parameters')
    if not listing.wait_for_service(timeout_sec=20.) or not getting.wait_for_service(timeout_sec=20.):
        raise RuntimeError('parameter services unavailable: '+name)
    future=listing.call_async(ListParameters.Request(prefixes=[],depth=0))
    rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
    if not future.done():raise RuntimeError('list parameters timeout: '+name)
    names=future.result().result.names
    future=getting.call_async(GetParameters.Request(names=names))
    rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
    if not future.done():raise RuntimeError('get parameters timeout: '+name)
    values=future.result().values
    uninitialized=[]
    if len(values)!=len(names):
        # Humble returns an empty batch if any declared typed array is unset.
        values=[]
        for parameter_name in names:
            future=getting.call_async(GetParameters.Request(names=[parameter_name]))
            rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
            if not future.done():raise RuntimeError('get parameter timeout: '+parameter_name)
            response=future.result().values
            if response:
                values.append(parameter_value_to_python(response[0]))
            else:
                values.append(None);uninitialized.append(parameter_name)
    else:
        values=[parameter_value_to_python(v) for v in values]
    params=dict(zip(names,[list(v) if isinstance(v,array) else v for v in values]))
    Path(sys.argv[1],filename).write_text(yaml.safe_dump({name:{'ros__parameters':params,'uninitialized_parameters':uninitialized}}))
finally:
 node.destroy_node();rclpy.shutdown()
