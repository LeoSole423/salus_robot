"""Conservative 3D ground removal before the cloud is projected to a LaserScan."""
from __future__ import annotations
import math
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from .scan_filters import obstacle_points

def rotate_translate(point: tuple[float,float,float], transform) -> tuple[float,float,float]:
    x,y,z=point; q=transform.rotation; t=transform.translation
    # Quaternion rotation, followed by translation.
    ix=q.w*x+q.y*z-q.z*y; iy=q.w*y+q.z*x-q.x*z; iz=q.w*z+q.x*y-q.y*x; iw=-q.x*x-q.y*y-q.z*z
    return (ix*q.w+iw*-q.x+iy*-q.z-iz*-q.y+t.x, iy*q.w+iw*-q.y+iz*-q.x-ix*-q.z+t.y, iz*q.w+iw*-q.z+ix*-q.y-iy*-q.x+t.z)

class ScanGroundFilter(Node):
    def __init__(self)->None:
        super().__init__("scan_ground_filter")
        for name,value in {"input_topic":"/scan_3d", "output_topic":"/obstacles_cloud", "target_frame":"base_footprint", "wheelbase_m":0.94, "profile":"urban", "ground_tolerance_m":0.20, "range_max":20.0}.items():self.declare_parameter(name,value)
        self.target=str(self.get_parameter("target_frame").value); profile=str(self.get_parameter("profile").value)
        self.tolerance=0.25 if profile == "rural" else float(self.get_parameter("ground_tolerance_m").value); self.range_max=float(self.get_parameter("range_max").value)
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self);self.pub=self.create_publisher(PointCloud2,str(self.get_parameter("output_topic").value),qos_profile_sensor_data);self.create_subscription(PointCloud2,str(self.get_parameter("input_topic").value),self.on_cloud,qos_profile_sensor_data)
        self.add_on_set_parameters_callback(self.on_parameters)
    def on_parameters(self,parameters):
        tolerance=self.tolerance
        for parameter in parameters:
            if parameter.name == "ground_tolerance_m":
                tolerance=float(parameter.value)
                if not 0.0 < tolerance <= 0.5:
                    return SetParametersResult(successful=False,reason="ground_tolerance_m must be in (0, 0.5]")
        self.tolerance=tolerance
        return SetParametersResult(successful=True)
    def on_cloud(self,msg:PointCloud2)->None:
        try: transform=self.buffer.lookup_transform(self.target,msg.header.frame_id,rclpy.time.Time(),timeout=Duration(seconds=0.05))
        except TransformException as error:
            self.get_logger().warn("LiDAR cloud rejected: missing transform to %s (%s)"%(self.target,error),throttle_duration_sec=2.0);return
        points=[]
        for row in point_cloud2.read_points(msg,field_names=("x","y","z"),skip_nans=True): points.append(rotate_translate((float(row[0]),float(row[1]),float(row[2])),transform.transform))
        header=Header();header.stamp=msg.header.stamp;header.frame_id=self.target
        self.pub.publish(point_cloud2.create_cloud_xyz32(header,obstacle_points(points,ground_tolerance_m=self.tolerance,max_range_m=self.range_max)))
def main(args=None)->None:
    rclpy.init(args=args);node=ScanGroundFilter()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
