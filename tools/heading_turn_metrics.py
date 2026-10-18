"""Pure timestamp/frame-aware metrics for #328; no ROS dependencies."""
import bisect
import json
import gzip
import math
from pathlib import Path


def angle(a,b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


class Interpolator:
    """Index one immutable time series once; preserve the exact pairing rule."""
    def __init__(self, rows):
        self.rows=rows
        self.times=[r['stamp_s'] for r in rows]

    def __call__(self,t):
        rows,times=self.rows,self.times
        i=bisect.bisect_left(times,t)
        if i<len(rows) and times[i]==t:
            return rows[i]
        if i==0 or i==len(rows) or times[i]-times[i-1]>.2:
            return None
        a,b=rows[i-1:i+1]; f=(t-times[i-1])/(times[i]-times[i-1])
        return dict(stamp_s=t,x=a['x']+f*(b['x']-a['x']),y=a['y']+f*(b['y']-a['y']),
                    yaw_rad=a['yaw_rad']+f*angle(b['yaw_rad'],a['yaw_rad']))


def interpolate(rows,t):
    return Interpolator(rows)(t)


def circle_radius(a,b,c):
    """Circumradius of three physical poses; None for coincident/straight data."""
    ab=math.hypot(b['x']-a['x'],b['y']-a['y'])
    bc=math.hypot(c['x']-b['x'],c['y']-b['y'])
    ac=math.hypot(c['x']-a['x'],c['y']-a['y'])
    cross=abs((b['x']-a['x'])*(c['y']-a['y'])-(b['y']-a['y'])*(c['x']-a['x']))
    return ab*bc*ac/(2*cross) if cross>1e-8 and min(ab,bc,ac)>1e-6 else None


def distribution(values):
    ordered=sorted(values)
    return dict(n=len(values),minimum=min(values) if values else None,
                p05=ordered[int((len(ordered)-1)*.05)] if values else None,
                median=ordered[len(ordered)//2] if values else None,
                p95=ordered[int((len(ordered)-1)*.95)] if values else None)


def stats(values):
    return dict(n=len(values),maximum=max(values) if values else None,
                rms=math.sqrt(sum(x*x for x in values)/len(values)) if values else None)


def continuity(rows,start,end):
    values=[r for r in rows if start<=r['stamp_s']<=end]
    stamps=sorted(set(r['stamp_s'] for r in values))
    gaps=[b-a for a,b in zip([start]+stamps,stamps+[end])]
    ages=[r['receive_sim_s']-r['stamp_s'] for r in values]
    return dict(samples=len(values),unique_stamps=len(stamps),max_gap_s=max(gaps),
                max_receive_age_s=max(ages) if ages else None,
                covered_fraction=(stamps[-1]-stamps[0])/(end-start) if len(stamps)>1 else 0)


def recovery(pairs,after,end):
    """First continuous one-second good interval; missing >.2 s resets it."""
    begin=None; previous=None
    for t,h,p in pairs:
        if t<after: continue
        if h>2 or p>.25 or (previous is not None and t-previous>.2):
            begin=None
        else:
            if begin is None: begin=t
            if t-begin>=1.: return dict(seconds=begin-after,censored=False)
        previous=t
    return dict(seconds=None,censored=True,observed_until_s=end-after)


def analyze(directory):
    root=Path(directory)
    data=(root/'samples.jsonl').read_text() if (root/'samples.jsonl').exists() else gzip.decompress((root/'analysis-samples.jsonl.gz').read_bytes()).decode()
    rows=[json.loads(line) for line in data.splitlines()]
    seq=json.loads((root/'sequence.json').read_text()); start=seq['start_sim_s']
    phases=[]; boundary=start
    for name,duration,*_ in seq['sequence']:
        phases.append((name,boundary,boundary+duration)); boundary+=duration
    end=boundary
    series={}
    for r in rows: series.setdefault(r['topic'],[]).append(r)
    for values in series.values(): values.sort(key=lambda r:r['stamp_s'])
    truth_topic=seq.get('truth_topic','/odom_raw')
    raw=series.get(truth_topic,[])
    if not raw: raise ValueError('no ground truth')
    truth=Interpolator(raw)
    if any(r['frame_id'] not in ('odom','salus_ackermann/odom','map') or r['child_frame_id'] not in ('base_footprint','salus_ackermann/base_footprint') for r in raw): raise ValueError('truth must be odom')
    result=dict(schema_version=2,truth_topic=truth_topic,reference_kind='physical_world_pose' if truth_topic=='/simulation/ground_truth' else 'kinematic_odometry_not_independent_truth',start_sim_s=start,duration_s=end-start,
                alignment='header interpolation <=0.2 s, no fitting; fixed sim map=odom datum/spawn',
                localization={},continuity={},instrumentation_valid=True)
    after=next(a for n,a,b in phases if n=='straight_after')
    paired_by_topic={}
    for topic,frame in [('/odometry/local','odom'),('/odometry/global','map')]:
        estimates=series.get(topic,[])
        if any(r['frame_id']!=frame or r['child_frame_id']!='base_footprint' for r in estimates):
            raise ValueError('unexpected estimator frame')
        paired=[]; rejected=0
        for r in estimates:
            if not start<=r['stamp_s']<=end: continue
            ref=truth(r['stamp_s'])
            if ref is None: rejected+=1; continue
            vals=(r['x'],r['y'],r['yaw_rad'],ref['x'],ref['y'],ref['yaw_rad'])
            if not all(math.isfinite(x) for x in vals): raise ValueError('nonfinite pose')
            paired.append((r['stamp_s'],abs(math.degrees(angle(r['yaw_rad'],ref['yaw_rad']))),math.hypot(r['x']-ref['x'],r['y']-ref['y'])))
        paired_by_topic[topic]=paired
        metrics=dict(all=dict(heading_deg=stats([h for t,h,p in paired]),position_m=stats([p for t,h,p in paired])),phases={},unpaired=rejected)
        for name,a,b in phases:
            population=[(h,p) for t,h,p in paired if a<=t<b]
            metrics['phases'][name]=dict(heading_deg=stats([h for h,p in population]),position_m=stats([p for h,p in population]))
        metrics['recovery']=recovery(paired,after,end)
        # Same estimate support for every offset. This diagnostic never changes
        # primary errors; t+offset< t implies a lagging estimate.
        offsets=[i*.02 for i in range(-15,16)]
        support=[r for r in estimates if start+.3<=r['stamp_s']<=end-.3 and all(truth(r['stamp_s']+o) is not None for o in offsets)]
        sweep=[dict(offset_s=o,heading_deg=stats([math.degrees(angle(r['yaw_rad'],truth(r['stamp_s']+o)['yaw_rad'])) for r in support])['rms']) for o in offsets]
        metrics['lag_diagnostic']=dict(common_samples=len(support),sweep=sweep,best=min(sweep,key=lambda x:x['heading_deg']) if support else None)
        if len(paired)<10 or rejected>max(2,len(paired)*.05):
            result['instrumentation_valid']=False
        metrics['observed_covariance']=dict(
            yaw_variance_rad2=stats([(r['pose_covariance'][35] if 'pose_covariance' in r else r['pose_covariance_diagonal'][5]) for r in estimates if start<=r['stamp_s']<=end]),
            x_variance_m2=stats([(r['pose_covariance'][0] if 'pose_covariance' in r else r['pose_covariance_diagonal'][0]) for r in estimates if start<=r['stamp_s']<=end]))
        result['localization'][topic]=metrics
    for topic in sorted(set([truth_topic,'/odom_raw','/imu/data','/odometry/local','/odometry/global','/tf/map/odom','/tf/odom/base_footprint','/gps/fix','/gps/odometry_map'])):
        c=continuity(series.get(topic,[]),start,end);result['continuity'][topic]=c
        if topic not in ('/gps/fix','/gps/odometry_map') and (c['samples']<10 or c['max_gap_s']>.5 or c['covered_fraction']<.95 or c['max_receive_age_s']>.5):
            result['instrumentation_valid']=False
    debug=[r for r in series.get('/gps/course_heading/debug',[]) if start<=r['stamp_s']<=end]
    result['heading_gate']=dict(strong_samples=sum(r.get('turn_state')=='STRONG_TURN' for r in debug),
        valid_in_strong=sum(r.get('turn_state')=='STRONG_TURN' and r.get('valid',False) for r in debug),
        held_samples=sum(r.get('reason','').startswith('hold_') for r in debug),
        window_generations=sorted(set(r.get('window_generation',0) for r in debug)),
        transitions=[dict(elapsed_s=r['stamp_s']-start,state=r.get('turn_state')) for i,r in enumerate(debug) if i==0 or r.get('turn_state')!=debug[i-1].get('turn_state')])
    transitions=result['heading_gate']['transitions']
    result['heading_gate']['reacquisitions']=[]
    for transition in transitions:
        if transition['state']!='REACQUIRE': continue
        t=start+transition['elapsed_s']
        first=next((r for r in debug if r['stamp_s']>=t and r.get('valid')),None)
        result['heading_gate']['reacquisitions'].append(dict(
            elapsed_s=transition['elapsed_s'],
            collection_s=first['stamp_s']-t if first else None,
            baseline_m=first.get('distance_m') if first else None))
    result['gps_compensation']=dict(reasons={reason:sum(r.get('reason')==reason for r in series.get('/gps/odometry_map/debug',[])) for reason in sorted(set(r.get('reason','') for r in series.get('/gps/odometry_map/debug',[])))})
    # Integrate requested unicycle path, independent from estimator. It is a
    # kinematic command reference, not an accepted Nav2 path.
    commands=series.get('/experiment/command',[]); model=[];x=y=h=0.;previous=start;v=w=0.
    for r in commands:
        if not start<=r['stamp_s']<=end: continue
        dt=r['stamp_s']-previous; previous=r['stamp_s']
        if abs(w)>1e-9:
            x+=v/w*(math.sin(h+w*dt)-math.sin(h));y+=v/w*(-math.cos(h+w*dt)+math.cos(h))
        else: x+=v*math.cos(h)*dt;y+=v*math.sin(h)*dt
        h+=w*dt;v=r['speed'];w=r['yaw_rate'];ref=truth(r['stamp_s'])
        if ref:model.append(math.hypot(x-ref['x'],y-ref['y']))
    result['tracking_command_model_m']=stats(model)
    turns=[r for r in raw if start<=r['stamp_s']<=end and abs(r['yaw_rate'])>=.18]
    result['actual_strong_turn']=dict(first_elapsed_s=turns[0]['stamp_s']-start if turns else None,
        last_elapsed_s=turns[-1]['stamp_s']-start if turns else None)
    result['actual_strong_turn']['errors']={}
    if turns:
        for topic,pairs in paired_by_topic.items():
            selected=[(h,p) for t,h,p in pairs if turns[0]['stamp_s']<=t<=turns[-1]['stamp_s']]
            result['actual_strong_turn']['errors'][topic]=dict(heading_deg=stats([h for h,p in selected]),position_m=stats([p for h,p in selected]))
    fixes=[r for r in series.get('/gps/fix',[]) if start<=r['stamp_s']<=end]
    outages=[]
    for a,b in zip(fixes,fixes[1:]):
        if b['stamp_s']-a['stamp_s']<=.5:continue
        resumed=b['receive_sim_s']
        outages.append(dict(last_measurement_elapsed_s=a['stamp_s']-start,
            next_measurement_elapsed_s=b['stamp_s']-start,resumed_receive_elapsed_s=resumed-start,
            measurement_gap_s=b['stamp_s']-a['stamp_s'],
            recovery={topic:recovery(pairs,resumed,end) for topic,pairs in paired_by_topic.items()}))
    result['gnss_outages']=outages

    result['motion']=dict(truth_distance_m=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(raw,raw[1:]) if start<=a['stamp_s']<b['stamp_s']<=end),
        measured_speed_mps=stats([abs(r['speed']) for r in raw if start<=r['stamp_s']<=end]),
        steering_deg=stats([abs(r['steer']) for r in series.get('/controller/drive_telemetry',[]) if start<=r['stamp_s']<=end]))
    radii=[abs(r['speed']/r['yaw_rate']) for r in raw if start<=r['stamp_s']<=end and abs(r['yaw_rate'])>=.1 and abs(r['speed'])>=.3]
    result['motion']['observed_radius_m']=dict(minimum=min(radii) if radii else None,median=sorted(radii)[len(radii)//2] if radii else None)
    geometric=[];steady=[]
    turn_start=next((a for name,a,b in phases if name=='entry'),None)
    for r in raw:
        t=r['stamp_s']
        if not start+.25<=t<=end-.25 or abs(r['yaw_rate'])<.1:continue
        a,b=truth(t-.25),truth(t+.25)
        if a is None or b is None:continue
        radius=circle_radius(a,r,b)
        if radius is None:continue
        geometric.append(radius)
        if turn_start is not None and t>=turn_start+3. and any(name.startswith('curve') and begin<=t<finish-.25 for name,begin,finish in phases):
            steady.append(radius)
    result['motion']['geometric_radius_m']=distribution(geometric)
    result['motion']['steady_geometric_radius_m']=distribution(steady)

    reference_errors=[]
    if truth_topic!='/odom_raw':
        for r in series.get('/odom_raw',[]):
            if not start<=r['stamp_s']<=end:continue
            ref=truth(r['stamp_s'])
            if ref:reference_errors.append((abs(math.degrees(angle(r['yaw_rad'],ref['yaw_rad']))),math.hypot(r['x']-ref['x'],r['y']-ref['y'])))
    result['kinematic_vs_physical']=dict(heading_deg=stats([h for h,p in reference_errors]),position_m=stats([p for h,p in reference_errors]))
    result['navigation']='not running: estimator-only, no planner decision inferred'
    (root/'turn-metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    return result


if __name__=='__main__':
    import sys
    print(json.dumps(analyze(sys.argv[1]),indent=2))
