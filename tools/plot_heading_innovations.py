#!/usr/bin/env python3
"""Static scientific plot of same-stream EKF diagnostic controls (matplotlib)."""
import argparse
import bisect
from collections import defaultdict
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(directory):
    directory = Path(directory)
    start = json.loads((directory/'sequence.json').read_text())['start_sim_s']
    series = defaultdict(list)
    for line in (directory/'samples.jsonl').read_text().splitlines():
        row=json.loads(line)
        series[row['topic']].append(row)
    for rows in series.values(): rows.sort(key=lambda r:r['stamp_s'])
    raw=series['/odom_raw']; times=[r['stamp_s'] for r in raw]
    def truth(t):
        i=bisect.bisect_left(times,t)
        if i==0 or i==len(times) or times[i]-times[i-1]>0.2:return None
        a,b=raw[i-1],raw[i]
        delta=math.atan2(math.sin(b['yaw_rad']-a['yaw_rad']),math.cos(b['yaw_rad']-a['yaw_rad']))
        return a['yaw_rad']+delta*(t-a['stamp_s'])/(b['stamp_s']-a['stamp_s'])
    topics=[('/odometry/local','EKF local','#64748b'),
            ('/odometry/global','Global actual','#dc2626'),
            ('/diagnostic/global_delayed_gps_reference','GPS sin compensar, mismo retardo','#f59e0b'),
            ('/diagnostic/global_compensated_gps','GPS compensado por montaje','#059669'),
            ('/diagnostic/global_no_gps_position','Sin posición GPS (diagnóstico)','#2563eb')]
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True,layout='constrained')
    for topic,label,color in topics:
        points=[]
        for r in series.get(topic,[]):
            ref=truth(r['stamp_s'])
            if ref is None or not 0<=r['stamp_s']-start<=23.5:continue
            error=math.degrees(math.atan2(math.sin(r['yaw_rad']-ref),math.cos(r['yaw_rad']-ref)))
            points.append((r['stamp_s']-start,error))
        if points:axes[0].plot(*zip(*points),label=label,color=color,lw=1.4)
    events=[json.loads(line) for line in (directory/'global-corrections.jsonl').read_text().splitlines()]
    for source,label,color in [('odom1_pose','Correcciones posición GPS','#dc2626'),
                               ('odom0_twist','Correcciones velocidad local','#2563eb'),
                               ('imu1_pose','Correcciones heading GPS','#a21caf')]:
        rows=[e for e in events if e['source']==source and 8<=e['stamp_s']-start<=14.3]
        net=0;points=[(8,0)]
        for e in rows:
            net+=math.degrees(e['k_innovation_yaw_rad'])
            points.append((e['stamp_s']-start,net))
        axes[1].step(*zip(*points),where='post',label=label,color=color,lw=1.8)
    for ax in axes:
        ax.axhline(0,color='#94a3b8',lw=.7)
        ax.axvspan(8,12.5,color='#fef3c7',alpha=.5)
        ax.grid(alpha=.2)
        ax.legend(fontsize=8,loc='best')
    axes[0].set_ylabel('Error yaw firmado [grados]');axes[0].set_ylim(-15,15)
    axes[1].set_ylabel('Corrección yaw acumulada\nentre t=8 y 14.3 s [grados]');axes[1].set_ylim(-20,20)
    axes[1].set_xlabel('Tiempo desde inicio [s simulados]');axes[1].set_xlim(0,23.5)
    fig.suptitle('SALUS — diagnóstico causal del EKF, '+directory.name.split('-')[-1]+'\nMismos sensores; perfiles experimentales sin TF ni autoridad de movimiento',fontsize=12)
    for extension in ('svg','png'):
        fig.savefig(directory/f'innovations-comparison.{extension}',dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory');args=parser.parse_args();plot(args.directory)
