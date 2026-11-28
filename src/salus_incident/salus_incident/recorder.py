"""Independent ROS subscriber with a bounded, asynchronous SQLite writer."""
import fcntl
import json
import math
import os
import subprocess
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
from collections import Counter
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from diagnostic_msgs.msg import KeyValue
from salus_interfaces.msg import CmdVelFinal, NavEvent

from .archive import Archive
from .policy import NS, SteeringJump, navigation_reason


TOPICS = frozenset("""
/plan /route_executor/active_chunk_path /route_executor/mission_path
/lookahead_point /lookahead_collision_arc /path_health /tf /tf_static
/odometry/global /odometry/local /odometry/gps /imu/data /imu/data_global
/salus/gps/fix /salus/imu/data /gps/filtered /gps/rtk_status /gps/course_heading
/localization/orientation /scan /scan_clean /local_costmap/costmap
/local_costmap/costmap_updates /global_costmap/costmap /global_costmap/costmap_updates
/collision_monitor_state /nav_command_server/events /nav_command_server/telemetry
/cmd_vel /cmd_vel_safe /cmd_vel_final /controller/drive_telemetry /rosout
/navigate_to_pose/_action/status /navigate_through_poses/_action/status
/follow_path/_action/status /diagnostics
""".split())
BASELINES = frozenset({
    "/tf_static", "/plan", "/route_executor/active_chunk_path",
    "/local_costmap/costmap", "/global_costmap/costmap",
})
DURABLE_TOPICS = frozenset({"/tf_static", "/route_executor/mission_path", "/route_executor/active_chunk_path"})
FULL_COSTMAPS = frozenset({"/local_costmap/costmap", "/global_costmap/costmap"})
REQUIRED_TOPICS = frozenset({"/plan", "/tf", "/odometry/global", "/scan", "/cmd_vel_final"})


CONFIG_PARAMETERS = {
    "/planner_server": ("planner_plugins", "GridBased.plugin", "GridBased.minimum_turning_radius"),
    "/controller_server": (
        "controller_frequency", "controller_plugins", "FollowPath.plugin",
        "FollowPath.lookahead_dist", "FollowPath.min_lookahead_dist",
        "FollowPath.max_lookahead_dist", "FollowPath.lookahead_time",
        "FollowPath.use_velocity_scaled_lookahead_dist", "FollowPath.desired_linear_vel",
        "FollowPath.min_approach_linear_velocity", "FollowPath.use_collision_detection",
        "FollowPath.max_allowed_time_to_collision_up_to_carrot",
    ),
    "/local_costmap/local_costmap": ("resolution", "footprint", "robot_radius", "plugins"),
    "/global_costmap/global_costmap": ("resolution", "footprint", "robot_radius", "plugins"),
    "/navsat_transform": ("datum", "yaw_offset", "magnetic_declination_radians"),
    "/map_gps_absolute_measurement": ("datum_lat", "datum_lon", "datum_yaw_deg"),
    "/salus_from_ll": ("datum_lat", "datum_lon", "datum_yaw_deg"),
}


def source_version():
    if os.getenv("SOURCE_SHA"):
        return os.environ["SOURCE_SHA"]
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL, timeout=2,
        ).strip()
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def metadata(topic, type_name):
    qos = ""
    if topic in DURABLE_TOPICS:
        qos = ("- history: 1\n  depth: 100\n  reliability: 1\n  durability: 1\n"
               "  deadline: {sec: 9223372036, nsec: 854775807}\n"
               "  lifespan: {sec: 9223372036, nsec: 854775807}\n"
               "  liveliness: 1\n  liveliness_lease_duration: {sec: 9223372036, nsec: 854775807}\n"
               "  avoid_ros_namespace_conventions: false\n")
    return rosbag2_py.TopicMetadata(
        name=topic, type=type_name, serialization_format="cdr", offered_qos_profiles=qos,
    )


class Recorder(Node):
    def __init__(self):
        super().__init__("incident_recorder")
        self.root = Path(self.declare_parameter("root", "runtime/incidents").value)
        self.segment_ns = self.positive("segment_s", 30) * NS
        self.pre = self.positive("pre_s", 900)
        self.post = self.positive("post_s", 120)
        self.ring_bytes = self.positive("ring_bytes", 2 << 30)
        self.archive_bytes = self.positive("archive_bytes", 10 << 30)
        self.free_bytes = self.positive("minimum_free_bytes", 10 << 30)
        rate = self.declare_parameter("costmap_max_hz", 0.5).value
        if not math.isfinite(rate) or rate < 0:
            raise ValueError("costmap_max_hz must be finite and non-negative")
        self.costmap_period = NS / rate if rate else 0
        self.config_snapshot = self.declare_parameter("context_json", "").value
        threshold = self.declare_parameter("steering_jump_per_m", 0.18).value
        if not math.isfinite(threshold) or threshold <= 0:
            raise ValueError("steering_jump_per_m must be finite and positive")
        self.jump = SteeringJump(threshold)
        self.version = source_version()
        self.operator_context = {}
        if self.config_snapshot:
            raw_context = json.loads(Path(self.config_snapshot).read_text())
            for key in ("profile", "datum"):
                if key in raw_context and (
                    key == "profile" and raw_context[key] in {"sim", "real"}
                    or key == "datum" and isinstance(raw_context[key], list)
                    and len(raw_context[key]) == 3
                    and all(isinstance(value, (int, float)) and math.isfinite(value)
                            for value in raw_context[key])
                ):
                    self.operator_context[key] = raw_context[key]
        self.effective = {}
        self.parameter_clients = {
            name: self.create_client(GetParameters, name + "/get_parameters")
            for name in CONFIG_PARAMETERS
        }
        self.parameter_requests = {}
        self.create_timer(30., self.read_config)
        self.queue = Queue(maxsize=2048)
        self.trigger_queue = Queue(maxsize=32)
        self.byte_lock = Lock()
        self.queued_bytes = 0
        self.drops = Counter()
        self.stopping = Event()
        self.subscriptions_by_topic = {}
        self.publisher = self.create_publisher(
            NavEvent, "/salus/incident/events", QoSProfile(
                depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.mission = ""
        self.mission_context = {}
        self.create_timer(1., self.discover)
        self.read_config()
        self.worker = Thread(target=self.run_guarded, name="incident-sqlite-writer")
        self.worker.start()

    def positive(self, name, default):
        value = self.declare_parameter(name, default).value
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def discover(self):
        advertised = dict(self.get_topic_names_and_types())
        for topic in TOPICS - self.subscriptions_by_topic.keys():
            types = advertised.get(topic)
            if not types or len(types) != 1:
                continue
            type_name = types[0]
            message_type = get_message(type_name)
            durable = topic in DURABLE_TOPICS
            qos = QoSProfile(
                depth=100 if durable else 10,
                reliability=ReliabilityPolicy.RELIABLE if durable else ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE,
            )
            self.subscriptions_by_topic[topic] = self.create_subscription(
                message_type, topic,
                lambda raw, name=topic, kind=type_name, cls=message_type: self.receive(name, kind, cls, raw),
                qos, raw=True,
            )

    def read_config(self):
        for node, client in self.parameter_clients.items():
            pending = self.parameter_requests.get(node)
            if pending and pending.done():
                result = pending.result() if pending.exception() is None else None
                if result:
                    values = {}
                    for name, value in zip(CONFIG_PARAMETERS[node], result.values):
                        if value.type:
                            values[name] = parameter_value_to_python(value)
                    self.effective = {**self.effective, node: values}
            if client.service_is_ready() and (pending is None or pending.done()):
                self.parameter_requests[node] = client.call_async(
                    GetParameters.Request(names=list(CONFIG_PARAMETERS[node])),
                )

    def receive(self, topic, kind, cls, raw):
        stamp = self.now_ns()
        if stamp <= 0 or self.stopping.is_set():
            return
        reason = None
        baseline_key = topic
        if topic == "/tf_static":
            message = deserialize_message(raw, cls)
            baseline_key = (topic, tuple(sorted(tf.child_frame_id for tf in message.transforms)))
        elif topic == "/cmd_vel_final":
            message = deserialize_message(raw, CmdVelFinal)
            if self.jump.observe(stamp, message.source, message.twist.linear.x, message.twist.angular.z):
                reason = "STEERING_CURVATURE_JUMP"
        elif topic == "/nav_command_server/events":
            message = deserialize_message(raw, NavEvent)
            details = {item.key: item.value for item in message.details}
            if message.code in {"PATROL_MISSION_STARTED", "ROUTE_LOOP_STARTED"}:
                self.mission = details.get("mission_id", "")
                self.mission_context = {"mission_start": details}
            reason = navigation_reason(message.code, details)
        elif topic == "/path_health":
            message = deserialize_message(raw, cls)
            if message.state == message.STOP_AND_WAIT and message.reason.endswith(":PLANNER_U_TURN"):
                reason = "PLANNER_U_TURN"
        elif topic == "/rosout":
            message = deserialize_message(raw, cls)
            if message.name == self.get_name():
                return
            if "RegulatedPurePursuitController detected collision ahead" in message.msg:
                reason = "PREDICTED_COLLISION"
            elif message.level < 30:
                return
        item = (stamp, topic, kind, raw, baseline_key, reason, self.mission, self.mission_context)
        with self.byte_lock:
            try:
                if self.queued_bytes + len(raw) > 32 << 20:
                    raise Full
                self.queue.put_nowait(item)
                self.queued_bytes += len(raw)
            except Full:
                self.drops[topic] += 1
                if reason:
                    try:
                        self.trigger_queue.put_nowait((stamp, reason, self.mission, self.mission_context))
                    except Full:
                        self.drops["incident_trigger"] += 1

    def capture_context(self):
        return {"source_version": self.version, "config": self.effective,
                "operator_context": self.operator_context,
                "costmap_max_hz": NS / self.costmap_period if self.costmap_period else 0,
                "wall_time_ns": time.time_ns(), "ros_time_ns": self.now_ns()}

    def notify(self, info):
        message = NavEvent()
        message.stamp = self.get_clock().now().to_msg()
        message.component = "incident_recorder"
        message.code = "NAVIGATION_INCIDENT"
        message.severity = 1
        message.details = [KeyValue(key=key, value=str(info.get(key, ""))) for key in
                           ("id", "status", "mission_id", "trigger_ns")]
        self.publisher.publish(message)

    def run_guarded(self):
        try:
            self.root.mkdir(parents=True, exist_ok=True)
            with (self.root / ".writer.lock").open("a") as lease:
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
                self.run()
        except (OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(f"Incident recorder unavailable: {error}")
        finally:
            self.stopping.set()

    def run(self):
        archive = Archive(self.root, self.notify, pre=self.pre, post=self.post,
                          ring_bytes=self.ring_bytes, archive_bytes=self.archive_bytes, required_topics=REQUIRED_TOPICS,
                          free_bytes=self.free_bytes, context=self.capture_context)
        writer = folder = None
        end = start = None
        previous_stamp = archive.latest_stamp()
        segment_bytes = 0
        registered, topics, baselines = set(), {}, {}

        def close():
            nonlocal writer
            if writer is None:
                return
            writer = None  # Humble finalizes SQLite and metadata in the destructor.
            with self.byte_lock:
                drops = dict(self.drops)
                self.drops.clear()
            archive.add(folder, dict(start_ns=start, end_ns=end, topics=topics, drops=drops))

        def write(stamp, topic, kind, raw):
            if topic not in registered:
                writer.create_topic(metadata(topic, kind))
                registered.add(topic)
            writer.write(topic, raw, stamp)
            stats = topics.setdefault(topic, dict(first_ns=stamp, last_ns=stamp, count=0))
            stats["last_ns"] = stamp
            stats["count"] += 1

        try:
            while not self.stopping.is_set() or not self.queue.empty() or not self.trigger_queue.empty():
                if not self.trigger_queue.empty():
                    archive.trigger(*self.trigger_queue.get_nowait())
                try:
                    stamp, topic, kind, raw, key, reason, mission, context = self.queue.get(timeout=.25)
                except Empty:
                    if writer and (self.now_ns() >= start + self.segment_ns
                                   or (archive.pending and self.now_ns() >= archive.pending[1]["end_ns"])):
                        close()
                    archive.advance(self.now_ns())
                    continue
                with self.byte_lock:
                    self.queued_bytes -= len(raw)
                if stamp <= 0:
                    continue
                if previous_stamp is not None and stamp < previous_stamp:
                    close()
                    archive.reset_clock()
                    baselines.clear()
                previous_stamp = stamp
                if writer and (stamp >= start + self.segment_ns
                               or segment_bytes >= 64 << 20
                               or (archive.pending and stamp > archive.pending[1]["end_ns"])):
                    close()
                if archive.pending and stamp > archive.pending[1]["end_ns"]:
                    archive.advance(stamp)
                if reason:
                    archive.trigger(stamp, reason, mission, context)
                if writer is None:
                    archive.prune()
                    if not archive.has_space():
                        archive.trim_ring(stamp)
                    if not archive.has_space():
                        with self.byte_lock:
                            self.drops[topic] += 1
                        continue
                    start = stamp
                    folder = archive.ring / f"{stamp:020d}-{time.time_ns()}"
                    writer = rosbag2_py.SequentialWriter()
                    writer.open(rosbag2_py.StorageOptions(uri=str(folder), storage_id="sqlite3"),
                                rosbag2_py.ConverterOptions("", ""))
                    registered, topics = set(), {}
                    segment_bytes = 0
                    for name, type_name, payload in baselines.values():
                        write(stamp, name, type_name, payload)
                end = stamp
                if topic in BASELINES:
                    baselines[key] = (topic, kind, raw)
                if topic in FULL_COSTMAPS and topic in topics and (
                        stamp - topics[topic]["last_ns"] < self.costmap_period):
                    continue
                write(stamp, topic, kind, raw)
                segment_bytes += len(raw)
        except (RuntimeError, OSError) as error:
            self.get_logger().error(f"Incident recording interrupted: {error}")
            self.stopping.set()
        finally:
            try:
                close()
                archive.finish("recorder_stopped")
            except (RuntimeError, OSError) as error:
                self.get_logger().error(f"Could not finalize incident: {error}")


def main():
    os.nice(10)
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stopping.set()
        node.worker.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
