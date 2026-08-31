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

def samples(grid, x, y):
    """Representative core and halo cells for one 1 m square keepout."""
    return {"core":cost_at(grid,x,y),"halo":cost_at(grid,x+1.25,y)}

def same_samples(grid, x, y, baseline):
    return samples(grid,x,y) == baseline

def perpendicular_distance(point, start, goal):
    dx, dy=goal[0]-start[0],goal[1]-start[1]
    length=math.hypot(dx,dy)
    if length == 0.: return 0.
    return abs(dx*(point[1]-start[1])-dy*(point[0]-start[0]))/length

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
def wait_latency(node,pred,msg,timeout=30.):
    started=time.monotonic(); wait(node,pred,msg,timeout); return time.monotonic()-started

def pose(x,y,heading=0.):
    result=PoseStamped(); result.header.frame_id="map"; result.pose.position.x=x; result.pose.position.y=y
    result.pose.orientation.z=math.sin(heading/2.); result.pose.orientation.w=math.cos(heading/2.)
    return result

def detour(node, zone):
    """Plan across a fixed map polygon, independent of the observed robot pose."""
    start=(zone[0]-14.,zone[1]); goal=(zone[0]+20.,zone[1])
    grid=node.global_maps[-1]
    if not (contains(grid,*start) and contains(grid,*goal)): raise RuntimeError("explicit planner endpoints outside rolling global costmap")
    if samples(grid,*start)["core"] == 254 or samples(grid,*goal)["core"] == 254: raise RuntimeError("explicit planner endpoint inside keepout")
    request=ComputePathToPose.Goal(); request.use_start=True; request.planner_id="GridBased"; request.start=pose(*start); request.goal=pose(*goal)
    future=node.plan_action.send_goal_async(request); wait(node,lambda:future.done(),"planner goal was not accepted",12.)
    handle=future.result()
    if handle is None or not handle.accepted: raise RuntimeError("planner goal rejected")
    result=handle.get_result_async(); wait(node,lambda:result.done(),"planner result unavailable",12.)
    outcome=result.result(); path=outcome.result.path
    if len(path.poses)<=2: raise RuntimeError("planner did not return avoidance path")
    deviations=[perpendicular_distance((p.pose.position.x,p.pose.position.y),start,goal) for p in path.poses]
    if max(deviations)<1.: raise RuntimeError("planner did not avoid the keepout zone")
    return {"start":start,"goal":goal,"poses":len(path.poses),"max_lateral_deviation_m":max(deviations)}

def process_resources():
    """Informational process snapshot; it deliberately is not a performance gate."""
    result=subprocess.run(["ps","-eo","pid=,rss=,pcpu=,comm=,args="],text=True,capture_output=True,check=False)
    selected=[]
    for line in result.stdout.splitlines():
        if any(token in line for token in ("nav2_","costmap","planner_server","controller_server")):
            fields=line.split(maxsplit=4)
            if len(fields)>=4: selected.append({"pid":int(fields[0]),"rss_kib":int(fields[1]),"cpu_pct":float(fields[2]),"command":fields[3]})
    return selected

def main():
    rclpy.init(); n=LongRange(); n.runtime=SmokeRuntime(n,"vector-keepout-runtime",FilePath(os.environ.get("SMOKE_ARTIFACT_DIR","."))/"vector_keepout_long_range.json"); ok=False; err=None; evidence={}
    try:
        n.runtime.wait("navigation startup unavailable", n.startup_ready, 45., stimulate=n.poll_bt_state, observe=n.startup_evidence)
        wait(n,lambda:n.odom and n.global_maps and n.local_maps and n.gps and n.tf.can_transform("odom","map",Time()),"initial odometry/GPS/TF/costmaps unavailable",30.)
        initial=n.odom[-1]; initial_tf=n.map_odom(); initial_gps=n.gps[-1]
        initial_global=metadata(n.global_maps[-1]); initial_local=metadata(n.local_maps[-1])
        zone_a=local_to_map(n.odom[-1],9.,0.)
        zones={"type":"FeatureCollection","features":[square("zone_a",*zone_a),square("zone_b",FAR_X),square("zone_c",FAR_C_X)]}
        set_started=time.monotonic(); response=call(n,n.set_zones,SetZonesGeoJson.Request(geojson=json.dumps(zones)),"set zones unavailable"); set_call_latency=time.monotonic()-set_started
        if not response.ok or response.polygon_count!=3: raise RuntimeError(response.error)
        initial_revision_latency=wait_latency(n,lambda:n.projected and len(n.projected[-1].polygons)==3,"projected state missing zones")
        source=[(p.zone_id,[(v.x,v.y) for v in p.outer.points]) for p in n.projected[-1].polygons]; zone_b=polygon_center(next(p for p in n.projected[-1].polygons if p.zone_id=="zone_b")); zone_c=polygon_center(next(p for p in n.projected[-1].polygons if p.zone_id=="zone_c"))
        initial_raster_latency=wait_latency(n,lambda:n.global_maps and has_core(n.global_maps[-1],*zone_a),"zone A core not rasterized")
        if contains(n.global_maps[-1],FAR_X,0.): raise RuntimeError("far zone unexpectedly inside origin rolling window")
        if call(n,n.goal,goal_request(*zone_a,0.),"zone A goal unavailable").ok: raise RuntimeError("zone A goal accepted")
        evidence["zone_a_detour"]=detour(n,zone_a)
        n.teleport(350.,0.)
        wait(n,lambda:n.gps and gps_distance_m(initial_gps,n.gps[-1])>300.,"GPS did not move >300 m")
        wait(n,lambda:n.odom and n.odom[-1].pose.pose.position.x>300.,"global odometry did not reach far region")
        wait(n,lambda:n.global_maps and contains(n.global_maps[-1],*zone_b) and has_core(n.global_maps[-1],*zone_b),"far zone not rasterized after rolling shift")
        b_global=metadata(n.global_maps[-1]); b_local_before=metadata(n.local_maps[-1])
        if contains(n.global_maps[-1],9.,0.): raise RuntimeError("origin zone remains in far rolling window")
        if [(p.zone_id,[(v.x,v.y) for v in p.outer.points]) for p in n.projected[-1].polygons] != source: raise RuntimeError("map-fixed projected geometry changed")
        if call(n,n.goal,goal_request(FAR_X,0.,0.),"zone B goal unavailable").ok: raise RuntimeError("zone B goal accepted")
        evidence["zone_b_detour"]=detour(n,zone_b)
        audits=[]
        def coherent():
            if not n.local_maps: return False
            try: audits.append(n.audit_local(n.local_maps[-1],zone_b))
            except Exception: return False
            return audits[-1]["inside"] and audits[-1]["nearest_core"] is not None and audits[-1]["nearest_core"] < 2.
        wait(n,coherent,"coherent local zone B sample unavailable")
        before_tf=audits[-1]["t_odom_map"]; before_local=n.local_maps[-1]; old_local=audits[-1]["zone_odom"]; evidence.update({"local_audit_before":audits[-1]})
        old_before_samples=samples(before_local,*old_local)
        if old_before_samples["core"] != 254 or old_before_samples["halo"] in (None,0): raise RuntimeError("old local core/halo missing before correction")
        # The 5 m physical move changes GPS/global EKF correction while local odom remains wheel-integrated.
        n.teleport(350.,5.)
        wait(n,lambda:math.hypot(n.map_odom()[0]-before_tf[0],n.map_odom()[1]-before_tf[1])>1.,"map->odom correction did not change")
        new_audits=[]
        def corrected_local():
            if not n.local_maps: return False
            try: new_audits.append(n.audit_local(n.local_maps[-1],zone_b))
            except Exception: return False
            evidence["local_audit_after"] = new_audits[-1]
            changed=math.hypot(new_audits[-1]["t_odom_map"][0]-before_tf[0],new_audits[-1]["t_odom_map"][1]-before_tf[1])>1.
            return changed and new_audits[-1]["inside"] and new_audits[-1]["nearest_core"] is not None
        wait(n,corrected_local,"coherent local sample after correction unavailable")
        new_tf=new_audits[-1]["t_odom_map"]; new_local=new_audits[-1]["zone_odom"]
        if not has_core(n.local_maps[-1],*new_local): raise RuntimeError("new local core missing")
        new_after_samples=samples(n.local_maps[-1],*new_local)
        if new_after_samples["core"] != 254 or new_after_samples["halo"] in (None,0): raise RuntimeError("new local core/halo missing after correction")
        wait(n,lambda:samples(n.local_maps[-1],*old_local)["core"] != 254 and samples(n.local_maps[-1],*old_local)["halo"] in (0,None),"former local core/halo was not cleared")
        old_after_samples=samples(n.local_maps[-1],*old_local)
        evidence.update({"t_odom_map_after":new_tf,"new_local":new_local,"local_before_samples":old_before_samples,"local_after_new_samples":new_after_samples,"local_after_old_baseline":old_after_samples,"local_after":metadata(n.local_maps[-1])})
        moved_x=FAR_X+6.; moved={"type":"FeatureCollection","features":[square("zone_a",*zone_a),square("zone_b",moved_x),square("zone_c",FAR_C_X)]}
        move_baseline=samples(n.global_maps[-1],moved_x,0.)
        if move_baseline["core"] == 254 or move_baseline["halo"] not in (0,None): raise RuntimeError("move destination lacks a keepout-free baseline")
        move_revision=n.projected[-1].revision; move_started=time.monotonic(); moved_response=call(n,n.set_zones,SetZonesGeoJson.Request(geojson=json.dumps(moved)),"move zone unavailable"); move_call_latency=time.monotonic()-move_started
        if not moved_response.ok: raise RuntimeError(moved_response.error)
        move_revision_latency=wait_latency(n,lambda:n.projected and n.projected[-1].revision>move_revision,"moved revision unavailable")
        moved_revision=n.projected[-1].revision
        move_raster_latency=wait_latency(n,lambda:n.global_maps and samples(n.global_maps[-1],moved_x,0.)["core"] == 254 and same_samples(n.global_maps[-1],*zone_b,move_baseline),"moved zone core/clearing unavailable")
        old_b_after_move=samples(n.global_maps[-1],*zone_b)
        remove_baseline=samples(n.global_maps[-1],moved_x,0.)
        if remove_baseline["core"] != 254 or remove_baseline["halo"] in (0,None): raise RuntimeError("moved zone core/halo missing")
        remove_revision=n.projected[-1].revision; remove_started=time.monotonic(); removed=call(n,n.set_zones,SetZonesGeoJson.Request(geojson=json.dumps({"type":"FeatureCollection","features":[square("zone_a",*zone_a),square("zone_c",FAR_C_X)]})),"remove zone unavailable"); remove_call_latency=time.monotonic()-remove_started
        if not removed.ok: raise RuntimeError(removed.error)
        remove_revision_latency=wait_latency(n,lambda:n.projected and n.projected[-1].revision>remove_revision and all(p.zone_id!="zone_b" for p in n.projected[-1].polygons),"removed revision/geometry unavailable")
        removed_revision=n.projected[-1].revision
        remove_raster_latency=wait_latency(n,lambda:n.global_maps and same_samples(n.global_maps[-1],moved_x,0.,move_baseline),"removed zone core/halo was not restored to baseline")
        removed_baseline=samples(n.global_maps[-1],moved_x,0.)
        n.teleport(700.,0.); wait(n,lambda:n.gps and gps_distance_m(initial_gps,n.gps[-1])>650. and n.odom and n.odom[-1].pose.pose.position.x>650.,"GPS/global odometry did not reach third region")
        if [(p.zone_id,[(v.x,v.y) for v in p.outer.points]) for p in n.projected[-1].polygons] != [item for item in source if item[0] != "zone_b"]: raise RuntimeError("map-fixed A/C geometry changed after rolling shifts")
        if abs(zone_c[0])<=150. and abs(zone_c[1])<=150.: raise RuntimeError("zone C is not beyond legacy extent")
        wait(n,lambda:n.global_maps and contains(n.global_maps[-1],*zone_c) and has_core(n.global_maps[-1],*zone_c),"zone C core not rasterized")
        c_global=metadata(n.global_maps[-1]); c_local=metadata(n.local_maps[-1])
        if contains(n.global_maps[-1],FAR_X,0.): raise RuntimeError("zone B remains in third rolling window")
        if call(n,n.goal,goal_request(*zone_c,0.),"zone C goal unavailable").ok: raise RuntimeError("zone C goal accepted")
        evidence["zone_c_detour"]=detour(n,zone_c)
        for name, value in (("~350 global",b_global),("~700 global",c_global)):
            if any(initial_global[k]!=value[k] for k in ("size_x","size_y","resolution")): raise RuntimeError(f"rolling {name} dimensions changed")
        for name, value in (("~350 local",b_local_before),("~700 local",c_local)):
            if any(initial_local[k]!=value[k] for k in ("size_x","size_y","resolution")): raise RuntimeError(f"rolling {name} dimensions changed")
        if n.count_publishers("/keepout_filter_mask")!=0: raise RuntimeError("legacy keepout mask publisher present")
        evidence.update({"global_costmaps":{"origin":initial_global,"~350m":b_global,"~700m":c_global},"local_costmaps":{"origin":initial_local,"~350m":b_local_before,"~700m":c_local},"map_odom_before":before_tf,"map_odom_after":new_tf,"map_fixed_coordinates":{"zone_a":zone_a,"zone_b":zone_b,"zone_c":zone_c},"projected_polygons":len(source),"legacy_mask_publishers":0,"revisions":{"initial":n.projected[0].revision,"move_from":move_revision,"move_to":moved_revision,"remove_from":remove_revision,"remove_to":removed_revision},"baseline_and_clearing":{"move_destination_before":move_baseline,"old_b_after_move":old_b_after_move,"moved_b_before_remove":remove_baseline,"moved_b_after_remove":removed_baseline},"latencies_s":{"set_call":set_call_latency,"set_response_to_initial_revision":initial_revision_latency,"set_response_to_initial_raster":initial_raster_latency,"move_call":move_call_latency,"move_revision":move_revision_latency,"move_costmap":move_raster_latency,"remove_call":remove_call_latency,"remove_revision":remove_revision_latency,"remove_costmap":remove_raster_latency},"process_resources":process_resources()}); ok=True; return 0
    except Exception as e: err=e; raise
    finally: n.runtime.finish(ok,error=err,evidence=evidence); n.destroy_node(); rclpy.shutdown()
if __name__=="__main__": sys.exit(main())
