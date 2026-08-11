import math

from geometry_msgs.msg import Quaternion
from salus_localization.sim_imu_from_odom import planar_yaw_from_quaternion


def test_planar_yaw_from_quaternion() -> None:
    quaternion = Quaternion(w=math.cos(0.4), z=math.sin(0.4))
    assert math.isclose(planar_yaw_from_quaternion(quaternion), 0.8, abs_tol=1.0e-9)
