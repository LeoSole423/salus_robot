"""Thin ROS adapter for route policies.

It never publishes velocity or talks to Nav2.  Each selected waypoint is sent
through nav_command_server, preserving the single command authority.
"""
from __future__ import annotations
import math, uuid
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL
from salus_interfaces.msg import NavTelemetry
from salus_interfaces.srv import (SetRouteMissionLL, CancelRouteMission, GetRouteMissionState, SetNavGoalLL, CancelNavGoal, BrakeNav)
from .route_model import RouteMission, RoutePhase, RouteWaypoint
from .route_preparation import validate_inputs, prepare
from .route_anchor import select_anchor
from .route_chunker import build_chunk, next_start
from .route_progress import project
from .route_state_machine import transition


class RouteExecutorNode(Node):
    def __init__(self):
        super().__init__("route_executor")
        self.declare_parameter("waypoint_reached_tolerance_m", 1.2)
        self._mission = RouteMission(); self._pose = None; self._chunk = None; self._target_offset = 0
        self._set_goal = self.create_client(SetNavGoalLL, "/nav_command_server/set_goal_ll")
        self._cancel_goal = self.create_client(CancelNavGoal, "/nav_command_server/cancel_goal")
        self._brake = self.create_client(BrakeNav, "/nav_command_server/brake")
        self._fromll = [self.create_client(FromLL, "/fromLL"), self.create_client(FromLL, "/navsat_transform/fromLL")]
        self.create_subscription(Odometry, "/odometry/global", self._on_pose, 10)
        self.create_subscription(NavTelemetry, "/nav_command_server/telemetry", self._on_telemetry, 10)
        self._mission_path = self.create_publisher(Path, "/route_executor/mission_path", 10)
        self._chunk_path = self.create_publisher(Path, "/route_executor/active_chunk_path", 10)
        self.create_service(SetRouteMissionLL, "/route_executor/set_route_mission_ll", self._set)
        self.create_service(CancelRouteMission, "/route_executor/cancel_route_mission", self._cancel)
        self.create_service(GetRouteMissionState, "/route_executor/get_route_mission_state", self._state)

    def _on_pose(self, msg): self._pose = msg.pose.pose.position
    def _on_telemetry(self, msg):
        if self._mission.phase != RoutePhase.ACTIVE: return
        if msg.manual_enabled:
            self._pause("manual takeover"); return
        if self._chunk and self._pose: self._mission.progress = project(self._chunk, self._pose.x, self._pose.y)
        if self._chunk and not msg.goal_active and msg.nav_result_text == "succeeded": self._advance()
        elif msg.nav_result_text in ("aborted", "cancelled") and not msg.goal_active: self._pause(f"navigation {msg.nav_result_text}")

    def _convert(self, points):
        client = next((item for item in self._fromll if item.service_is_ready()), None)
        if client is None: return None, "fromLL service unavailable"
        converted=[]
        for point in points:
            request=FromLL.Request(); request.ll_point.latitude,request.ll_point.longitude=point.lat,point.lon
            future=client.call_async(request); rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if not future.done() or future.result() is None: return None,"fromLL conversion failed"
            map_point=future.result().map_point; converted.append(RouteWaypoint(**{**point.__dict__,"map_x":map_point.x,"map_y":map_point.y}))
        return converted,""

    def _set(self, request, response):
        error=validate_inputs(list(request.lats),list(request.lons),list(request.yaws_deg),list(request.waypoint_action_jsons),list(request.waypoint_roles))
        response.input_waypoint_count=len(request.lats)
        if error: response.ok,response.error=False,error; return response
        raw=[RouteWaypoint(float(a),float(b),float(c),i,True,(list(request.waypoint_action_jsons) or [""]*len(request.lats))[i],(list(request.waypoint_roles) or ["normal"]*len(request.lats))[i]) for i,(a,b,c) in enumerate(zip(request.lats,request.lons,request.yaws_deg))]
        converted,error=self._convert(raw)
        if error: response.ok,response.error=False,error; return response
        prepared=prepare(converted,loop=bool(request.loop),input_count=len(raw),spacing_m=float(request.leg_spacing_m),chunk_span_m=float(request.chunk_span_m),chunk_max_waypoints=int(request.chunk_max_waypoints))
        if self._pose: anchor=select_anchor(prepared,self._pose.x,self._pose.y,float(self.get_parameter("waypoint_reached_tolerance_m").value)); prepared=type(prepared)(**{**prepared.__dict__,"anchor_input_index":anchor})
        self._mission=RouteMission(prepared=prepared,phase=RoutePhase.PREPARING,mission_id=str(uuid.uuid4()),target_index=prepared.anchor_input_index)
        transition(self._mission,RoutePhase.ACTIVE); self._dispatch(); response.ok,response.error=True,""; response.expanded_waypoint_count=len(prepared.waypoints); return response

    def _dispatch(self):
        route=self._mission.prepared
        self._chunk=build_chunk(route,self._mission.target_index,self._mission.loop_iteration)
        if self._chunk is None: transition(self._mission,RoutePhase.COMPLETED); self._brake.call_async(BrakeNav.Request(duration_s=0.25,brake_pct=100)); return
        self._target_offset=0; self._publish_paths(); self._send_target()
    def _send_target(self):
        point=self._chunk.waypoints[self._target_offset]; req=SetNavGoalLL.Request(); req.lat,req.lon,req.yaw_deg=point.lat,point.lon,point.yaw_deg; req.suppress_success_brake=True
        self._set_goal.call_async(req)
    def _advance(self):
        self._mission.reached+=1; self._target_offset+=1
        if self._target_offset < len(self._chunk.waypoints): self._send_target(); return
        self._mission.target_index=next_start(self._mission.prepared,self._chunk); self._mission.chunk_id+=1
        if self._mission.prepared.loop and self._mission.target_index == 0: self._mission.loop_iteration+=1
        self._dispatch()
    def _pause(self, reason):
        if self._mission.phase == RoutePhase.ACTIVE: transition(self._mission,RoutePhase.PAUSED,reason)
        self._cancel_goal.call_async(CancelNavGoal.Request())
    def _cancel(self, _request,response):
        if self._mission.phase == RoutePhase.ACTIVE: transition(self._mission,RoutePhase.CANCELLED,"cancelled")
        self._cancel_goal.call_async(CancelNavGoal.Request()); self._brake.call_async(BrakeNav.Request(duration_s=0.25,brake_pct=100)); response.ok,response.error=True,""; return response
    def _publish_paths(self):
        for publisher,points in ((self._mission_path,self._mission.prepared.waypoints),(self._chunk_path,self._chunk.waypoints)):
            msg=Path();msg.header.frame_id="map"; msg.header.stamp=self.get_clock().now().to_msg()
            for point in points:
                pose=PoseStamped();pose.header=msg.header;pose.pose.position.x,pose.pose.position.y=point.map_x,point.map_y;pose.pose.orientation.w=1.0;msg.poses.append(pose)
            publisher.publish(msg)
    def _state(self,_request,response):
        m=self._mission;p=m.prepared;response.ok,response.error=True,"";response.active=m.phase==RoutePhase.ACTIVE;response.paused=m.phase==RoutePhase.PAUSED;response.status=m.phase.value;response.loop=False if p is None else p.loop;response.mission_id=m.mission_id;response.chunk_id=m.chunk_id;response.loop_iteration=m.loop_iteration;response.reached_checkpoint_count=m.reached;response.input_waypoint_count=0 if p is None else p.input_count;response.expanded_waypoint_count=0 if p is None else len(p.waypoints);response.current_target_index=m.target_index;response.current_progress_expanded_index=m.progress.expanded_index;response.current_checkpoint_index=m.progress.checkpoint_index;response.current_progress_ratio=m.progress.ratio;response.cross_track_error_m=m.progress.cross_track_error_m;response.distance_to_target_m=m.progress.distance_to_target_m;response.blocked_reason_text=m.pause_reason;return response

def main(args=None):
    rclpy.init(args=args); node=RouteExecutorNode()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
