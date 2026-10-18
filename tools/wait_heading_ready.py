"""Direct graph readiness for isolated heading captures; no ros2cli daemon."""
import time
import rclpy
from rclpy.node import Node

rclpy.init()
node=Node('heading_graph_readiness')
deadline=time.monotonic()+60
try:
    while time.monotonic()<deadline:
        if 'gps_course_heading' in node.get_node_names():
            break
        rclpy.spin_once(node,timeout_sec=.1)
    else:
        raise RuntimeError('gps_course_heading absent after 60 s')
finally:
    node.destroy_node()
    rclpy.shutdown()
