#!/usr/bin/env python3
"""End-to-end bounded vector keepout validation over a Fortress long-range world."""
import json, math, os, subprocess, sys, time
from pathlib import Path as FilePath
import rclpy
from nav2_msgs.msg import Costmap
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from smoke_navigation_zones_sim import ZonesSmoke, call, goal_request, lat_lon, projected_contains, yaw
from smoke_runtime import SmokeRuntime
from salus_interfaces.srv import SetZonesGeoJson

FAR_X = 355.0

def square(identifier, x, y=0., half=.5):
    points=[(x-half,y-half),(x+half,y-half),(x+half,y+half),(x-half,y+half),(x-half,y-half)]
    return {"type":"Feature","properties":{"id":identifier,"enabled":True},"geometry":{"type":"Polygon","coordinates":[[[lon,lat] for lat,lon in (lat_lon(px,py) for px,py in points)]]}}

def has_core(grid, x, y):
    m=grid.metadata; i=math.floor((x-m.origin.position.x)/m.resolution); j=math.floor((y-m.origin.position.y)/m.resolution)
    return 0<=i<m.size_x and 0<=j<m.size_y and grid.data[j*m.size_x+i] == 254

def contains(grid,x,y):
    m=grid.metadata; return m.origin.position.x<=x<m.origin.position.x+m.size_x*m.resolution and m.origin.position.y<=y<m.origin.position.y+m.size_y*m.resolution

def map_point_to_odom(x_map, y_map, transform):
    tx, ty, theta = transform; dx, dy = x_map-tx, y_map-ty
    return math.cos(theta)*dx + math.sin(theta)*dy, -math.sin(theta)*dx + math.cos(theta)*dy

def metadata(grid):
    m=grid.metadata; return {"size_x":int(m.size_x),"size_y":int(m.size_y),"resolution":float(m.resolution),"origin_x":float(m.origin.position.x),"origin_y":float(m.origin.position.y)}

def gps_distance_m(a, b):
    lat1, lat2 = math.radians(a.latitude), math.radians(b.latitude)
    dlat, dlon = lat2-lat1, math.radians(b.longitude-a.longitude)
    h=math.sin(dlat/2)**2+math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 6371000.0*2.0*math.asin(math.sqrt(h))

class LongRange(ZonesSmoke):
    def __init__(self):
        super().__init__(); self.global_maps=[]; self.local_maps=[]; self.gps=[]
        from sensor_msgs.msg import NavSatFix
        self.create_subscription(Costmap,"/global_costmap/costmap_raw",self.global_maps.append,10)
        self.create_subscription(Costmap,"/local_costmap/costmap_raw",self.local_maps.append,10)
        self.create_subscription(NavSatFix,"/gps/fix_raw",self.gps.append,10)
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
    def map_odom(self):
        if not self.tf.can_transform("map","odom",Time()): raise RuntimeError("map->odom unavailable")
        t=self.tf.lookup_transform("map","odom",Time()).transform
        q=t.rotation; return (t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def teleport(self,x,y):
        request=f'name: "salus_ackermann" position {{ x: {x} y: {y} z: 0.30 }} orientation {{ w: 1 }}'
        result=subprocess.run(["ign","service","-s","/world/salus_empty/set_pose","--reqtype","ignition.msgs.Pose","--reptype","ignition.msgs.Boolean","--timeout","5000","--req",request],text=True,capture_output=True,timeout=8)
        if result.returncode or "data: true" not in result.stdout: raise RuntimeError(f"Fortress set_pose failed: {result.stdout} {result.stderr}")

def wait(node,pred,msg,timeout=30.): node.runtime.wait(msg,pred,timeout)
def detour(node,start_x,goal_x):
    node.plans.clear(); r=call(node,node.goal,goal_request(goal_x,0.,0.),"detour goal unavailable")
    if not r.ok: raise RuntimeError(r.error)
    wait(node,lambda:node.plans and len(node.plans[-1].poses)>2,"planner did not return path",12.)
    if max(abs(p.pose.position.y) for p in node.plans[-1].poses)<1.: raise RuntimeError("planner did not avoid the keepout zone")

def main():
    rclpy.init(); n=LongRange(); n.runtime=SmokeRuntime(n,"vector-keepout-runtime",FilePath(os.environ.get("SMOKE_ARTIFACT_DIR","."))/"vector_keepout_long_range.json"); ok=False; err=None; evidence={}
    try:
        n.runtime.wait("navigation startup unavailable", n.startup_ready, 45., stimulate=n.poll_bt_state, observe=n.startup_evidence)
        wait(n,lambda:n.odom and n.global_maps and n.local_maps and n.gps and n.tf.can_transform("map","odom",Time()),"initial odometry/GPS/TF/costmaps unavailable",30.)
        initial=n.odom[-1]; initial_tf=n.map_odom(); initial_gps=n.gps[-1]
        zones={"type":"FeatureCollection","features":[square("zone_a",9.),square("zone_b",FAR_X)]}
        response=call(n,n.set_zones,SetZonesGeoJson.Request(geojson=json.dumps(zones)),"set zones unavailable")
        if not response.ok or response.polygon_count!=2: raise RuntimeError(response.error)
        wait(n,lambda:n.projected and len(n.projected[-1].polygons)==2,"projected state missing both zones")
        source=[(p.zone_id,[(v.x,v.y) for v in p.outer.points]) for p in n.projected[-1].polygons]
        wait(n,lambda:n.global_maps and has_core(n.global_maps[-1],9.,0.),"zone A core not rasterized")
        if contains(n.global_maps[-1],FAR_X,0.): raise RuntimeError("far zone unexpectedly inside origin rolling window")
        if call(n,n.goal,goal_request(9.,0.,0.),"zone A goal unavailable").ok: raise RuntimeError("zone A goal accepted")
        detour(n,0.,22.)
        n.teleport(350.,0.)
        wait(n,lambda:n.gps and gps_distance_m(initial_gps,n.gps[-1])>300.,"GPS did not move >300 m")
        wait(n,lambda:n.odom and n.odom[-1].pose.pose.position.x>300.,"global odometry did not reach far region")
        wait(n,lambda:n.global_maps and contains(n.global_maps[-1],FAR_X,0.) and has_core(n.global_maps[-1],FAR_X,0.),"far zone not rasterized after rolling shift")
        if contains(n.global_maps[-1],9.,0.): raise RuntimeError("origin zone remains in far rolling window")
        if [(p.zone_id,[(v.x,v.y) for v in p.outer.points]) for p in n.projected[-1].polygons] != source: raise RuntimeError("map-fixed projected geometry changed")
        if call(n,n.goal,goal_request(FAR_X,0.,0.),"zone B goal unavailable").ok: raise RuntimeError("zone B goal accepted")
        detour(n,350.,375.)
        before_tf=n.map_odom(); before_local=n.local_maps[-1]; old_local=map_point_to_odom(FAR_X,0.,before_tf)
        # The 5 m physical move changes GPS/global EKF correction while local odom remains wheel-integrated.
        n.teleport(350.,5.)
        wait(n,lambda:math.hypot(n.map_odom()[0]-before_tf[0],n.map_odom()[1]-before_tf[1])>1.,"map->odom correction did not change")
        wait(n,lambda:n.tf.can_transform("map","odom",Time()) and math.hypot(n.map_odom()[0]-before_tf[0],n.map_odom()[1]-before_tf[1])>1.,"map->odom correction did not change")
        new_tf=n.map_odom(); new_local=map_point_to_odom(FAR_X,0.,new_tf)
        wait(n,lambda:n.local_maps and not has_core(n.local_maps[-1],*old_local),"former local core was not cleared")
        wait(n,lambda:n.local_maps and has_core(n.local_maps[-1],*new_local),"new local core missing")
        moved={"type":"FeatureCollection","features":[square("zone_a",9.),square("zone_b",FAR_X+4.)]}; call(n,n.set_zones,SetZonesGeoJson.Request(geojson=json.dumps(moved)),"move zone unavailable")
        wait(n,lambda:n.global_maps and has_core(n.global_maps[-1],FAR_X+4.,0.) and not has_core(n.global_maps[-1],FAR_X,0.),"moved zone stale global core")
        call(n,n.set_zones,SetZonesGeoJson.Request(geojson=json.dumps({"type":"FeatureCollection","features":[square("zone_a",9.)]})),"remove zone unavailable")
        wait(n,lambda:n.global_maps and not has_core(n.global_maps[-1],FAR_X+4.,0.),"removed zone stale global core")
        evidence={"initial_global":metadata(n.global_maps[0]),"far_global":metadata(n.global_maps[-1]),"initial_local":metadata(before_local),"far_local":metadata(n.local_maps[-1]),"map_odom_before":before_tf,"map_odom_after":new_tf,"projected_polygons":len(source),"legacy_mask_publishers":n.count_publishers("/keepout_filter_mask")}; ok=True; return 0
    except Exception as e: err=e; raise
    finally: n.runtime.finish(ok,error=err,evidence=evidence); n.destroy_node(); rclpy.shutdown()
if __name__=="__main__": sys.exit(main())
