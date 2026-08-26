"""ROS adapter for odometry from calibrated kinematic measurements."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from salus_interfaces.msg import SteeringMeasurement, TractionMeasurement

from .ackermann_odometry import diag_covariance, quaternion_from_yaw, stamp_to_seconds
from .kinematic_ackermann_odometry_domain import (
    KinematicOdometryConfig, KinematicOdometryState, KinematicSample,
    accept_steering, accept_traction,
)


class KinematicAckermannOdometryNode(Node):
    """Publish derived wheel odometry; TF remains exclusively owned by the EKF."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("kinematic_ackermann_odometry", parameter_overrides=parameter_overrides)
        defaults = {
            "traction_input_topic": "/vehicle/kinematic_inputs/traction",
            "steering_input_topic": "/vehicle/kinematic_inputs/steering",
            "odom_topic": "/wheel/odometry", "twist_topic": "/vehicle/twist",
            "traction_source_id": "rear_drive_wheel_equivalent",
            "steering_source_id": "virtual_center_wheel",
            "odom_frame": "odom", "base_frame": "base_footprint",
            "wheelbase_m": 0.94, "max_pair_skew_s": 0.05, "max_dt_s": 0.2,
            "pose_covariance_xy": 0.05, "pose_covariance_yaw": 0.1,
            "twist_covariance_vx": 0.05, "twist_covariance_vy": 0.01,
            "twist_covariance_yaw_rate": 0.1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._config = KinematicOdometryConfig.create(
            traction_source_id=self.get_parameter("traction_source_id").value,
            steering_source_id=self.get_parameter("steering_source_id").value,
            wheelbase_m=self.get_parameter("wheelbase_m").value,
            max_pair_skew_s=self.get_parameter("max_pair_skew_s").value,
            max_dt_s=self.get_parameter("max_dt_s").value,
        )
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        if not self._odom_frame.strip() or not self._base_frame.strip():
            raise ValueError("odom_frame and base_frame must not be empty")
        self._pose_covariance_xy = _nonnegative(self, "pose_covariance_xy")
        self._pose_covariance_yaw = _nonnegative(self, "pose_covariance_yaw")
        self._twist_covariance_vx = _nonnegative(self, "twist_covariance_vx")
        self._twist_covariance_vy = _nonnegative(self, "twist_covariance_vy")
        self._twist_covariance_yaw_rate = _nonnegative(
            self, "twist_covariance_yaw_rate"
        )
        self._state = KinematicOdometryState()
        self._odom_publisher = self.create_publisher(Odometry, str(self.get_parameter("odom_topic").value), 10)
        self._twist_publisher = self.create_publisher(TwistWithCovarianceStamped, str(self.get_parameter("twist_topic").value), 10)
        self._traction_subscription = self.create_subscription(TractionMeasurement, str(self.get_parameter("traction_input_topic").value), self._on_traction, 10)
        self._steering_subscription = self.create_subscription(SteeringMeasurement, str(self.get_parameter("steering_input_topic").value), self._on_steering, 10)

    def _on_traction(self, message: TractionMeasurement) -> None:
        update = accept_traction(self._state, _traction_sample(message), self._config)
        self._state = update.state
        if update.emission is not None:
            self._publish(update.emission)

    def _on_steering(self, message: SteeringMeasurement) -> None:
        update = accept_steering(self._state, _steering_sample(message), self._config)
        self._state = update.state
        if update.emission is not None:
            self._publish(update.emission)

    def _publish(self, emission) -> None:
        odom = Odometry()
        seconds = math.floor(emission.stamp_s)
        nanoseconds = round((emission.stamp_s - seconds) * 1_000_000_000)
        if nanoseconds == 1_000_000_000:
            seconds, nanoseconds = seconds + 1, 0
        odom.header.stamp.sec, odom.header.stamp.nanosec = seconds, nanoseconds
        odom.header.frame_id, odom.child_frame_id = self._odom_frame, self._base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = emission.x_m, emission.y_m
        odom.pose.pose.orientation = quaternion_from_yaw(emission.yaw_rad)
        odom.pose.covariance = diag_covariance(self._pose_covariance_xy, self._pose_covariance_xy, self._pose_covariance_yaw)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = emission.speed_mps, emission.yaw_rate_rps
        odom.twist.covariance = diag_covariance(self._twist_covariance_vx, self._twist_covariance_vy, self._twist_covariance_yaw_rate)
        self._odom_publisher.publish(odom)
        twist = TwistWithCovarianceStamped()
        twist.header, twist.twist = odom.header, odom.twist
        self._twist_publisher.publish(twist)


def _sample(message, value: float) -> KinematicSample:
    return KinematicSample(
        str(message.metadata.source_id), stamp_to_seconds(message.metadata.header.stamp),
        int(message.source_type), int(message.metadata.status), int(message.available_fields),
        int(message.measured_fields), int(message.calculated_fields), int(message.inferred_fields), float(value),
    )


def _traction_sample(message: TractionMeasurement) -> KinematicSample:
    return _sample(message, message.linear_velocity_mps)


def _steering_sample(message: SteeringMeasurement) -> KinematicSample:
    return _sample(message, message.position_rad)


def _nonnegative(node: Node, name: str) -> float:
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicAckermannOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
