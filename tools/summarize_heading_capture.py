"""Timestamp-paired metrics and standalone SVG for a manual heading capture."""
import bisect
import json
import math
import sys
from pathlib import Path

root = Path(sys.argv[1])
rows = [json.loads(x) for x in (root/'samples.jsonl').read_text().splitlines()]
seq = json.loads((root/'sequence.json').read_text())
start = seq['start_sim_s']
series = {}
for r in rows:
    series.setdefault(r['topic'], []).append(r)
for values in series.values():
    values.sort(key=lambda x:x['stamp_s'])
raw = series['/odom_raw']
times = [r['stamp_s'] for r in raw]
def diff(a,b):
    return math.atan2(math.sin(a-b), math.cos(a-b))
def truth(t):
    i = bisect.bisect_left(times,t)
    if i==0 or i==len(times):
        return None
    a,b=raw[i-1],raw[i]
    if b['stamp_s']-a['stamp_s']>0.2:
        return None
    return a['yaw_rad'] + diff(b['yaw_rad'],a['yaw_rad'])*(t-a['stamp_s'])/(b['stamp_s']-a['stamp_s'])
def percentile(values,p):
    return sorted(values)[min(len(values)-1,int((len(values)-1)*p))] if values else None
metrics = {'timestamp_pairing':'interpolation of /odom_raw at measurement header stamp; no extrapolation', 'phases':{}}
for phase,duration,speed,rate in seq['sequence']:
    phase_rows=[r for r in raw if r['phase']==phase]
    result={'raw_yaw_start_deg':math.degrees(phase_rows[0]['yaw_rad']),
            'raw_yaw_end_deg':math.degrees(phase_rows[-1]['yaw_rad']), 'errors':{}}
    for topic in ['/odometry/local','/odometry/global','/gps/course_heading']:
        errors=[]
        for r in series.get(topic,[]):
            if r['phase']!=phase: continue
            ref=truth(r['stamp_s'])
            if ref is not None: errors.append(abs(math.degrees(diff(r['yaw_rad'],ref))))
        result['errors'][topic]={'paired_samples':len(errors), 'abs_yaw_error_p95_deg':percentile(errors,.95),'abs_yaw_error_max_deg':max(errors) if errors else None}
    gps=[r for r in series.get('/gps/course_heading',[]) if r['phase']==phase]
    result['gps_heading_samples']=len(gps)
    result['gps_first_elapsed_s']=gps[0]['stamp_s']-start if gps else None
    metrics['phases'][phase]=result
debug = series.get('/gps/course_heading/debug', [])
active_debug = [r for r in debug if start <= r['stamp_s'] <= start+sum(x[1] for x in seq['sequence'])]
transitions = []
previous = None
for r in active_debug:
    state = r.get('turn_state', 'legacy')
    if state != previous:
        transitions.append(dict(elapsed_s=r['stamp_s']-start, state=state))
        previous = state
strong_debug = [r for r in active_debug if r.get('turn_state') == 'STRONG_TURN']
metrics['gate'] = dict(transitions=transitions,
    strong_turn_debug_samples=len(strong_debug),
    valid_during_strong_turn=sum(bool(r['valid']) for r in strong_debug),
    held_samples=sum(r['reason'].startswith('hold_') for r in active_debug),
    window_generations=sorted(set(r.get('window_generation',0) for r in active_debug)),
    reasons={reason:sum(r['reason']==reason for r in active_debug) for reason in sorted(set(r['reason'] for r in active_debug))})
reacquire = next((r for r in transitions if r['state']=='REACQUIRE'), None)
if reacquire:
    first = next((r for r in active_debug if r['valid'] and r['stamp_s']-start >= reacquire['elapsed_s']), None)
    metrics['gate']['reacquire_elapsed_s'] = reacquire['elapsed_s']
    metrics['gate']['first_clean_heading_elapsed_s'] = first['stamp_s']-start if first else None
    metrics['gate']['window_collection_s'] = first['stamp_s']-start-reacquire['elapsed_s'] if first else None
    metrics['gate']['first_clean_baseline_m'] = first['distance_m'] if first else None
actual_turn = [r for r in raw if start <= r['stamp_s'] and abs(r['yaw_rate']) >= 0.18]
metrics['measured_turn'] = dict(rate_threshold_rps=0.18,
    first_elapsed_s=actual_turn[0]['stamp_s']-start if actual_turn else None,
    last_elapsed_s=actual_turn[-1]['stamp_s']-start if actual_turn else None, errors={})
if actual_turn:
    for topic in ['/odometry/local','/odometry/global']:
        errors=[abs(math.degrees(diff(r['yaw_rad'], truth(r['stamp_s']))))
            for r in series.get(topic, [])
            if actual_turn[0]['stamp_s'] <= r['stamp_s'] <= actual_turn[-1]['stamp_s'] and truth(r['stamp_s']) is not None]
        metrics['measured_turn']['errors'][topic]=dict(max_deg=max(errors) if errors else None, p95_deg=percentile(errors,.95))
metrics['continuity'] = {}
for topic in ['/odometry/local','/odometry/global','/tf/map/odom','/tf/odom/base_footprint']:
    values=[r for r in series.get(topic,[]) if start<=r['stamp_s']<=start+sum(x[1] for x in seq['sequence'])]
    gaps=[b['stamp_s']-a['stamp_s'] for a,b in zip(values,values[1:])]
    steps=[abs(math.degrees(diff(b['yaw_rad'],a['yaw_rad']))) for a,b in zip(values,values[1:])]
    metrics['continuity'][topic]=dict(samples=len(values), max_gap_s=max(gaps) if gaps else None,
        max_step_deg=max(steps) if steps else None)
root.joinpath('metrics.json').write_text(json.dumps(metrics,indent=2))

# Dependency-free vector chart. GPS measurements are points, never interpolated
# across outages: missing absolute corrections must remain visible.
W,H=1140,740
left_turn = seq.get('scenario') == 'left'
yaw_low, yaw_high = (-10,90) if left_turn else (-90,10)
rate_low, rate_high = (-.1,.45) if left_turn else (-.45,.1)
left,right=85,1090
duration=sum(x[1] for x in seq['sequence'])
def X(t):return left+(right-left)*t/duration
def Y(v,top,low,high):return top+190*(high-v)/(high-low)
parts=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
 '<rect width="100%" height="100%" fill="white"/>',
 '<style>text{font-family:Arial,sans-serif;fill:#243247;font-size:13px}</style>',
 '<text x="85" y="28" font-size="21">SALUS — heading: recta → curva fuerte → recta</text>',
 '<text x="85" y="50">Simulación clean · GPS course-over-ground · seed 6400 · orientación medida, no validación física</text>']
phases=[('Recta inicial','#f1f5f9'),('Curva fuerte','#fff1d6'),('Recta de salida','#f1f5f9'),('Stop','#e9eef5')]
for top,low,high,label in [(110,yaw_low,yaw_high,'Heading [°]'),(390,rate_low,rate_high,'Velocidad de giro [rad/s]')]:
    begin=0
    for (name,color),(_,d,_,_) in zip(phases,seq['sequence']):
        parts.append(f'<rect x="{X(begin)}" y="{top}" width="{X(begin+d)-X(begin)}" height="190" fill="{color}"/>')
        if top==110:parts.append(f'<text x="{X(begin)+8}" y="98">{name}</text>')
        begin+=d
    for j in range(6):
        value=low+(high-low)*j/5
        yy=Y(value,top,low,high)
        parts.append(f'<line x1="{left}" y1="{yy}" x2="{right}" y2="{yy}" stroke="#cbd5e1" stroke-width=".6"/><text x="25" y="{yy+4}">{value:.2f}</text>')
    parts.append(f'<text x="85" y="{top-22}">{label}</text>')
    for t in range(0,24,2):
        parts.append(f'<text x="{X(t)-4}" y="{top+208}">{t}</text>')
    parts.append(f'<text x="840" y="{top+230}">Tiempo desde inicio [s simulados]</text>')
colors={'/odom_raw':'#1f2937','/odometry/local':'#2563eb','/odometry/global':'#dc2626','/gps/course_heading':'#a21caf','/imu/data':'#059669'}
for topic in ['/odom_raw','/odometry/local','/odometry/global','/gps/course_heading']:
    points=[(X(r['stamp_s']-start),Y(math.degrees(r['yaw_rad']),110,yaw_low,yaw_high)) for r in series.get(topic,[]) if 0<=r['stamp_s']-start<=duration]
    if topic=='/gps/course_heading':
        for x,y in points:parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2.7" fill="{colors[topic]}"/>')
    else:
        coords=' '.join(f'{x:.2f},{y:.2f}' for x,y in points)
        parts.append(f'<polyline points="{coords}" fill="none" stroke="{colors[topic]}" stroke-width="1.6"/>')
for topic in ['/odom_raw','/odometry/local','/imu/data']:
    coords=' '.join(f"{X(r['stamp_s']-start):.2f},{Y(r['yaw_rate'],390,rate_low,rate_high):.2f}" for r in series.get(topic,[]) if 0<=r['stamp_s']-start<=duration)
    parts.append(f'<polyline points="{coords}" fill="none" stroke="{colors[topic]}" stroke-width="1.5"/>')
for i,(topic,label) in enumerate([('/odom_raw','Gazebo real'),('/odometry/local','EKF local'),('/odometry/global','EKF global'),('/gps/course_heading','Correcciones GPS'),('/imu/data','IMU giro')]):
    x=85+i*200
    parts.append(f'<rect x="{x}" y="650" width="15" height="4" fill="{colors[topic]}"/><text x="{x+21}" y="657">{label}</text>')
parts.append('<text x="85" y="692">Puntos GPS: mediciones publicadas. Los huecos significan ausencia de corrección, no ausencia de heading.</text>')
parts.append('<text x="85" y="715">Captura open-loop, sin Nav2; IMU clean derivada de odometría Gazebo. Ver informe y métricas para limitaciones.</text></svg>')
root.joinpath('heading.svg').write_text('\n'.join(parts))
root.joinpath('heading.html').write_text('<!doctype html><meta charset="utf-8"><title>SALUS heading baseline</title><body style="margin:24px;background:#edf2f7">'+ '\n'.join(parts)+'</body>')
print(json.dumps(metrics,indent=2))
