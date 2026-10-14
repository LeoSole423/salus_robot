from copy import deepcopy
import math
from types import SimpleNamespace

import pytest
import rclpy
from nav_msgs.msg import Odometry
from robot_localization.srv import FromLL
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import LookupException
from salus_localization.map_gps_absolute_measurement import MapGpsAbsoluteMeasurement


class Capture:
    def __init__(self): self.messages=[]
    def publish(self,msg): self.messages.append(deepcopy(msg))


class Mount:
    def __init__(self,x=0.66,y=0.2): self.x,self.y=x,y
    def lookup_transform(self,base,sensor,stamp):
        assert base=='base_footprint' and sensor=='gps_link'
        return SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(x=self.x,y=self.y)))


@pytest.fixture
def node():
    rclpy.init()
    node=MapGpsAbsoluteMeasurement()
    node.now_s=lambda:8.1
    node.publisher=Capture()
    node.debug=Capture()
    node.tf_buffer=Mount()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def odometry(stamp,yaw=0,variance=0.1):
    msg=Odometry()
    msg.header.stamp.sec=int(stamp)
    msg.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
    msg.header.frame_id='map';msg.child_frame_id='base_footprint'
    msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
    msg.pose.covariance[35]=variance
    return msg


def orientation():
    msg=Imu();msg.header.stamp.sec=8;msg.header.frame_id='base_footprint'
    msg.orientation.w=1.0;msg.orientation_covariance[8]=0.05
    return msg


def fix():
    msg=NavSatFix();msg.header.stamp.sec=8;msg.header.frame_id='gps_link'
    msg.latitude=-31.4858037+0.2/111320
    msg.longitude=-64.2410570+0.66/(111320*math.cos(math.radians(-31.4858037)))
    msg.status.status=0
    msg.position_covariance_type=msg.COVARIANCE_TYPE_DIAGONAL_KNOWN
    msg.position_covariance=[0.0004,0.0,0.0,0.0,0.0004,0.0,0.0,0.0,0.0016]
    return msg


def ready(node,yaw=0):
    node.on_absolute_heading(orientation())
    node.on_map_odom(odometry(7.98,yaw))
    node.on_map_odom(odometry(8.04,yaw))


def test_rejects_uninitialized_heading_then_uses_tf_mount_and_original_stamp(node):
    node.on_fix(fix())
    assert not node.publisher.messages
    ready(node)
    msg=fix();msg.header.stamp.nanosec=10_000_000
    node.on_fix(msg)
    result=node.publisher.messages[-1]
    assert abs(result.pose.pose.position.x)<1e-6
    assert abs(result.pose.pose.position.y)<1e-6
    assert result.header.stamp.nanosec==10_000_000
    assert result.header.frame_id=='map' and result.child_frame_id=='base_footprint'
    assert msg.header.frame_id=='gps_link'  # Incoming fix is never mutated.
    assert result.pose.covariance[0]==pytest.approx(0.054)
    assert result.pose.covariance[1]==pytest.approx(-0.0132)
    assert result.pose.covariance[7]==pytest.approx(0.09356)


def test_pending_fix_waits_for_aligned_map_yaw_and_expires_without_tf(node):
    node.on_absolute_heading(orientation())
    node.on_map_odom(odometry(7.98))
    node.on_fix(fix())
    assert not node.publisher.messages
    node.on_map_odom(odometry(8.04))
    assert len(node.publisher.messages)==1
    class Missing:
        def lookup_transform(self,*args): raise LookupException('mount missing')
    node.tf_buffer=Missing()
    msg=fix();msg.header.stamp.nanosec=20_000_000
    node.on_fix(msg)
    assert node.pending
    node.now_s=lambda:9
    node.flush()
    assert not node.pending
    assert len(node.publisher.messages)==1


def test_no_new_absolute_heading_is_needed_while_map_estimate_turns(node):
    ready(node)
    node.on_fix(fix())
    node.now_s=lambda:8.3
    node.on_map_odom(odometry(8.2,math.pi/2))
    msg=fix();msg.header.stamp.nanosec=200_000_000
    node.on_fix(msg)
    result=node.publisher.messages[-1]
    assert result.pose.pose.position.x==pytest.approx(0.86,abs=1e-6)
    assert result.pose.pose.position.y==pytest.approx(-0.46,abs=1e-6)


def test_fromll_is_geographical_point_conversion_without_mount(node):
    request=FromLL.Request();request.ll_point.latitude=fix().latitude;request.ll_point.longitude=fix().longitude
    response=node.on_from_ll(request,FromLL.Response())
    assert response.map_point.x==pytest.approx(0.66,abs=1e-6)
    assert response.map_point.y==pytest.approx(0.2,abs=1e-6)


def test_invalid_covariance_future_and_duplicate_fixes_do_not_publish(node):
    ready(node)
    bad=fix();bad.position_covariance[0]=-1
    node.on_fix(bad)
    assert not node.publisher.messages
    future=fix();future.header.stamp.sec=9
    node.on_fix(future)
    assert not node.publisher.messages
    node.on_fix(fix())
    assert not node.publisher.messages


def test_clock_reset_discards_absolute_initialization_and_old_samples(node):
    ready(node)
    node.on_fix(fix())
    node.last_now=10.0
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000))
    assert MapGpsAbsoluteMeasurement.now_s(node)==1.0
    assert not node.headings.samples
    assert not node.pending
    assert node.absolute_heading_since is None
    assert node.last_fix_stamp is None


def test_unknown_or_wrong_frame_does_not_initialize_an_absolute_heading(node):
    msg=orientation();msg.header.frame_id='odom'
    node.on_absolute_heading(msg)
    node.on_map_odom(odometry(7.98))
    node.on_map_odom(odometry(8.04))
    node.on_fix(fix())
    assert not node.publisher.messages
