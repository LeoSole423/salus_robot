#!/usr/bin/env python3
"""End-to-end bounded vector keepout validation over a Fortress long-range world."""
import json, math, os, subprocess, sys, time
from pathlib import Path as FilePath
import rclpy
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from smoke_navigation_zones_sim import ZonesSmoke, call, goal_request, lat_lon, local_to_map, yaw
from smoke_runtime import SmokeRuntime
from salus_interfaces.srv import SetZonesGeoJson

FAR_X, FAR_C_X = 355.0, 705.0

def square(identifier, x, y=0., half=.5):
    points=[(x-half,y-half),(x+half,y-half),(x+half,y+half),(x-half,y+half),(x-half,y-half)]
    return {"type":"Feature","properties":{"id":identifier,"enabled":True},"geometry":{"type":"Polygon","coordinates":[[[lon,lat] for lat,lon in (lat_lon(px,py) for px,py in points)]]}}

def has_core(grid, x, y):
    m=grid.metadata; i=math.floor((x-m.origin.position.x)/m.resolution); j=math.floor((y-m.origin.position.y)/m.resolution)
    return 0<=i<m.size_x and 0<=j<m.size_y and grid.data[j*m.size_x+i] == 254

def contains(grid,x,y):
    m=grid.metadata; return m.origin.position.x<=x<m.origin.position.x+m.size_x*m.resolution and m.origin.position.y<=y<m.origin.position.y+m.size_y*m.resolution

def map_point_to_odom(x_map, y_map, transform):
    tx, ty, theta = transform; c, s = math.cos(theta), math.sin(theta)
    return tx + c*x_map - s*y_map, ty + s*x_map + c*y_map

def cost_at(grid, x, y):
    m=grid.metadata; i=math.floor((x-m.origin.position.x)/m.resolution); j=math.floor((y-m.origin.position.y)/m.resolution)
    return grid.data[j*m.size_x+i] if 0<=i<m.size_x and 0<=j<m.size_y else None

def metadata(grid):
    m=grid.metadata; return {"size_x":int(m.size_x),"size_y":int(m.size_y),"resolution":float(m.resolution),"origin_x":float(m.origin.position.x),"origin_y":float(m.origin.position.y)}

def gps_distance_m(a, b):
    lat1, lat2 = math.radians(a.latitude), math.radians(b.latitude)
    dlat, dlon = lat2-lat1, math.radians(b.longitude-a.longitude)
    h=math.sin(dlat/2)**2+math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 6371000.0*2.0*math.asin(math.sqrt(h))

def polygon_center(polygon):
    points=polygon.outer.points; xs=[p.x for p in points]; ys=[p.y for p in points]
    return (min(xs)+max(xs))/2., (min(ys)+max(ys))/2.

class LongRange(ZonesSmoke):
    def __init__(self):
        super().__init__(); self.global_maps=[]; self.local_maps=[]; self.gps=[]
        from sensor_msgs.msg import NavSatFix
        self.create_subscription(Costmap,"/global_costmap/costmap_raw",self.global_maps.append,10)
        self.create_subscription(Costmap,"/local_costmap/costmap_raw",self.local_maps.append,10)
        self.create_subscription(NavSatFix,"/gps/fix_raw",self.gps.append,10)
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
    def map_odom(self):
        if not self.tf.can_transform("odom","map",Time()): raise RuntimeError("map->odom unavailable")
        t=self.tf.lookup_transform("odom","map",Time()).transform
        q=t.rotation; return (t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def teleport(self,x,y):
        request=f'name: "salus_ackermann" position {{ x: {x} y: {y} z: 0.30 }} orientation {{ w: 1 }}'
        result=subprocess.run(["ign","service","-s","/world/salus_empty/set_pose","--reqtype","ignition.msgs.Pose","--reptype","ignition.msgs.Boolean","--timeout","5000","--req",request],text=True,capture_output=True,timeout=8)
        if result.returncode or "data: true" not in result.stdout: raise RuntimeError(f"Fortress set_pose failed: {result.stdout} {result.stderr}")
    def audit_local(self, grid, zone):
        stamp=Time.from_msg(grid.header.stamp); tf=self.tf.lookup_transform("odom","map",stamp).transform
        q=tf.rotation; transform=(tf.translation.x,tf.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        point=map_point_to_odom(*zone,transform); m=metadata(grid); base=self.tf.lookup_transform("odom","base_footprint",stamp).transform.translation
        cores=[(m["origin_x"]+(i+.5)*m["resolution"],m["origin_y"]+(j+.5)*m["resolution"]) for j in range(grid.metadata.size_y) for i in range(grid.metadata.size_x) if grid.data[j*grid.metadata.size_x+i]==254]
        return {"stamp_ns":stamp.nanoseconds,"metadata":m,"t_odom_map":transform,"zone_map":zone,"zone_odom":point,"robot_odom":(base.x,base.y),"inside":contains(grid,*point),"cost":cost_at(grid,*point),"core_count":len(cores),"core_bbox":None if not cores else (min(x for x,y in cores),min(y for x,y in cores),max(x for x,y in cores),max(y for x,y in cores)),"nearest_core":None if not cores else min(math.hypot(x-point[0],y-point[1]) for x,y in cores)}

def wait(node,pred,msg,timeout=30.): node.runtime.wait(msg,pred,timeout)
def detour(node):
    current=node.odom[-1]; heading=yaw(current); x,y=local_to_map(current,22.,0.)
    request=ComputePathToPose.Goal(); request.use_start=False; request.planner_id="GridBased"
    request.goal=PoseStamped(); request.goal.header.frame_id="map"; request.goal.pose.position.x=x; request.goal.pose.position.y=y
    request.goal.pose.orientation.z=math.sin(heading/2.); request.goal.pose.orientation.w=math.cos(heading/2.)
    future=node.plan_action.send_goal_async(request); wait(node,lambda:future.done(),"planner goal was not accepted",12.)
    handle=future.result()
    if handle is None or not handle.accepted: raise RuntimeError("planner goal rejected")
    result=handle.get_result_async(); wait(node,lambda:result.done(),"planner result unavailable",12.)
    outcome=result.result(); path=outcome.result.path
    if len(path.poses)<=2: raise RuntimeError("planner did not return avoidance path")
    start=current.pose.pose.position
    deviations=[abs(-(p.pose.position.x-start.x)*math.sin(heading)+(p.pose.position.y-start.y)*math.cos(heading)) for p in path.poses]
    if max(deviations)<1.: raise RuntimeError("planner did not avoid the keepout zone")

def main():
    rclpy.init(); n=LongRange(); n.runtime=SmokeRuntime(n,"vector-keepout-runtime",FilePath(os.environ.get("SMOKE_ARTIFACT_DIR","."))/"vector_keepout_long_range.json"); ok=False; err=None; evidence={}
    try:
        n.runtime.wait("navigation startup unavailable", n.startup_ready, 45., stimulate=n.poll_bt_state, observe=n.startup_evidence)
        wait(n,lambda:n.odom and n.global_maps and n.local_maps and n.gps and n.tf.can_transform("odom","map",Time()),"initial odometry/GPS/TF/costmaps unavailable",30.)
        initial=n.odom[-1]; initial_tf=n.map_odom(); initial_gps=n.gps[-1]
        zones={"type":"FeatureCollection","features":[square("zone_a",9.),square("zone_b",FAR_X),square("zone_c",FAR_C_X)]}
        response=call(n,n.set_zones,SetZonesGeoJson.Request(geojson=json.dumps(zones)),"set zones unavailable")
        if not response.ok or response.polygon_count!=3: raise RuntimeError(response.error)
        wait(n,lambda:n.projected and len(n.projected[-1].polygons)==3,"projected state missing zones")
        source=[(p.zone_id,[(v.x,v.y) for v in p.outer.points]) for p in n.projected[-1].polygons]; zone_b=polygon_center(next(p for p in n.projected[-1].polygons if p.zone_id=="zone_b"))
        wait(n,lambda:n.global_maps and has_core(n.global_maps[-1],9.,0.),"zone A core not rasterized")
        if contains(n.global_maps[-1],FAR_X,0.): raise RuntimeError("far zone unexpectedly inside origin rolling window")
        if call(n,n.goal,goal_request(9.,0.,0.),"zone A goal unavailable").ok: raise RuntimeError("zone A goal accepted")
        detour(n)
        n.teleport(350.,0.)
        wait(n,lambda:n.gps and gps_distance_m(initial_gps,n.gps[-1])>300.,"GPS did not move >300 m")
        wait(n,lambda:n.odom and n.odom[-1].pose.pose.position.x>300.,"global odometry did not reach far region")
        wait(n,lambda:n.global_maps and contains(n.global_maps[-1],FAR_X,0.) and has_core(n.global_maps[-1],FAR_X,0.),"far zone not rasterized after rolling shift")
        if contains(n.global_maps[-1],9.,0.): raise RuntimeError("origin zone remains in far rolling window")
        if [(p.zone_id,[(v.x,v.y) for v in p.outer.points]) for p in n.projected[-1].polygons] != source: raise RuntimeError("map-fixed projected geometry changed")
        if call(n,n.goal,goal_request(FAR_X,0.,0.),"zone B goal unavailable").ok: raise RuntimeError("zone B goal accepted")
        detour(n)
        audits=[]
        def coherent():
            if not n.local_maps: return False
            try: audits.append(n.audit_local(n.local_maps[-1],zone_b))
            except Exception: return False
            return audits[-1]["inside"] and audits[-1]["nearest_core"] is not None and audits[-1]["nearest_core"] < 2.
        wait(n,coherent,"coherent local zone B sample unavailable")
        before_tf=audits[-1]["t_odom_map"]; before_local=n.local_maps[-1]; old_local=audits[-1]["zone_odom"]; evidence.update({"local_audit_before":audits[-1]})
        if not has_core(before_local,*old_local): raise RuntimeError("old local core missing before correction")
        # The 5 m physical move changes GPS/global EKF correction while local odom remains wheel-integrated.
        n.teleport(350.,5.)
        wait(n,lambda:math.hypot(n.map_odom()[0]-before_tf[0],n.map_odom()[1]-before_tf[1])>1.,"map->odom correction did not change")
        new_audits=[]
        def corrected_local():
            if not n.local_maps: return False
            try: new_audits.append(n.audit_local(n.local_maps[-1],zone_b))
            except Exception: return False
            evidence["local_audit_after"] = new_audits[-1]
            return new_audits[-1]["inside"] and new_audits[-1]["nearest_core"] is not None
        wait(n,corrected_local,"coherent local sample after correction unavailable")
        new_tf=new_audits[-1]["t_odom_map"]; new_local=new_audits[-1]["zone_odom"]
        if not has_core(n.local_maps[-1],*new_local): raise RuntimeError("new local core missing")
        evidence.update({"t_odom_map_after":new_tf,"new_local":new_local,"old_local_after_cost":cost_at(n.local_maps[-1],*old_local),"new_local_after_cost":cost_at(n.local_maps[-1],*new_local),"local_after":metadata(n.local_maps[-1])})
        wait(n,lambda:not has_core(n.local_maps[-1],*old_local),"former local core was not cleared")
        moved={"type":"FeatureCollection","features":[square("zone_a",9.),square("zone_b",FAR_X+4.),square("zone_c",FAR_C_X)]}; move_revision=n.projected[-1].revision; moved_response=call(n,n.set_zones,SetZonesGeoJson.Request(geojson=json.dumps(moved)),"move zone unavailable")
        if not moved_response.ok: raise RuntimeError(moved_response.error)
        wait(n,lambda:n.projected and n.projected[-1].revision>move_revision,"moved revision unavailable")
        wait(n,lambda:n.global_maps and has_core(n.global_maps[-1],FAR_X+4.,0.) and not has_core(n.global_maps[-1],FAR_X,0.),"moved zone stale global core")
        remove_revision=n.projected[-1].revision; removed=call(n,n.set_zones,SetZonesGeoJson.Request(geojson=json.dumps({"type":"FeatureCollection","features":[square("zone_a",9.),square("zone_c",FAR_C_X)]})),"remove zone unavailable")
        if not removed.ok: raise RuntimeError(removed.error)
        wait(n,lambda:n.projected and n.projected[-1].revision>remove_revision and all(p.zone_id!="zone_b" for p in n.projected[-1].polygons),"removed revision/geometry unavailable")
        wait(n,lambda:n.global_maps and not has_core(n.global_maps[-1],FAR_X+4.,0.),"removed zone stale global core")
        n.teleport(700.,0.); wait(n,lambda:n.gps and gps_distance_m(initial_gps,n.gps[-1])>650.,"GPS did not reach third region")
        wait(n,lambda:n.global_maps and contains(n.global_maps[-1],FAR_C_X,0.) and has_core(n.global_maps[-1],FAR_C_X,0.),"zone C core not rasterized")
        if contains(n.global_maps[-1],FAR_X,0.): raise RuntimeError("zone B remains in third rolling window")
        if call(n,n.goal,goal_request(FAR_C_X,0.,0.),"zone C goal unavailable").ok: raise RuntimeError("zone C goal accepted")
        detour(n)
        origin_meta=metadata(n.global_maps[0]); far_meta=metadata(n.global_maps[-1])
        if any(origin_meta[k]!=far_meta[k] for k in ("size_x","size_y","resolution")): raise RuntimeError("rolling costmap dimensions changed")
        if n.count_publishers("/keepout_filter_mask")!=0: raise RuntimeError("legacy keepout mask publisher present")
        evidence.update({"initial_global":origin_meta,"far_global":far_meta,"initial_local":metadata(before_local),"far_local":metadata(n.local_maps[-1]),"map_odom_before":before_tf,"map_odom_after":new_tf,"projected_polygons":len(source),"legacy_mask_publishers":0,"revisions":{"move":move_revision,"remove":remove_revision}}); ok=True; return 0
    except Exception as e: err=e; raise
    finally: n.runtime.finish(ok,error=err,evidence=evidence); n.destroy_node(); rclpy.shutdown()
if __name__=="__main__": sys.exit(main())
