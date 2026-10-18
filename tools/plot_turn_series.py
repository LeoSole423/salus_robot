#!/usr/bin/env python3
"""Plot independent physical/kinematic/estimated trajectories and errors."""
import argparse
import gzip
import json
from pathlib import Path
import math
from heading_turn_metrics import Interpolator, angle


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root=args.capture
    seq=json.loads((root/'sequence.json').read_text());start=seq['start_sim_s'];end=start+sum(r[1] for r in seq['sequence'])
    data=(root/'samples.jsonl').read_text() if (root/'samples.jsonl').exists() else gzip.decompress((root/'analysis-samples.jsonl.gz').read_bytes()).decode()
    series={}
    for line in data.splitlines():
        row=json.loads(line)
        if start<=row['stamp_s']<=end:series.setdefault(row['topic'],[]).append(row)
    for rows in series.values():rows.sort(key=lambda r:r['stamp_s'])
    truth_topic=seq.get('truth_topic','/odom_raw');truth=Interpolator(series[truth_topic])
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    paths=axes[0,0];heading=axes[0,1];position=axes[1,0];steering=axes[1,1]
    colors={truth_topic:'black','/odom_raw':'#777777','/odometry/local':'#2563eb','/odometry/global':'#dc2626'}
    labels={truth_topic:'Physical pose' if truth_topic!='/odom_raw' else 'Kinematic reference','/odom_raw':'Kinematic odometry','/odometry/local':'Local EKF','/odometry/global':'Global EKF'}
    for topic in dict.fromkeys([truth_topic,'/odom_raw','/odometry/local','/odometry/global']):
        rows=series.get(topic,[]);color=colors[topic];label=labels[topic]
        paths.plot([r['x'] for r in rows],[r['y'] for r in rows],color=color,label=label,lw=1.3)
        if topic==truth_topic:continue
        paired=[(r,truth(r['stamp_s'])) for r in rows];paired=[(r,t) for r,t in paired if t]
        heading.plot([r['stamp_s']-start for r,t in paired],[abs(math.degrees(angle(r['yaw_rad'],t['yaw_rad']))) for r,t in paired],color=color,label=label,lw=1.2)
        position.plot([r['stamp_s']-start for r,t in paired],[math.hypot(r['x']-t['x'],r['y']-t['y']) for r,t in paired],color=color,label=label,lw=1.2)
    rows=series.get('/controller/drive_telemetry',[])
    steering.plot([r['stamp_s']-start for r in rows],[r['steer'] for r in rows],color='#047857',label='Measured center steering')
    cursor=0.
    for name,duration,*_ in seq['sequence']:
        if name not in ('straight_before','straight_after','stop','cruise'):
            for ax in [heading,position,steering]:ax.axvspan(cursor,cursor+duration,color='#e9b44c',alpha=.15)
        cursor+=duration
    paths.set(xlabel='World X [m]',ylabel='World Y [m]');paths.axis('equal')
    heading.set(ylabel='Absolute heading error [deg]',xlabel='Simulation time from command start [s]')
    position.set(ylabel='Position error [m]',xlabel='Simulation time from command start [s]')
    steering.set(ylabel='Steering [deg]',xlabel='Simulation time from command start [s]')
    for ax in axes.flat:ax.grid(alpha=.3);ax.legend(fontsize=8)
    fig.suptitle(root.name+'\nPhysical pose comparison; no spatial fit or lag correction',fontsize=11)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=150);fig.savefig(args.output.with_suffix('.svg'));plt.close(fig)


if __name__=='__main__':main()
