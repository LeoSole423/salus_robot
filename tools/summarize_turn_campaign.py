#!/usr/bin/env python3
"""Recompute and preserve #328 captures, metrics, manifests and comparison plots."""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import sys
import subprocess
sys.path.insert(0,str(Path(__file__).resolve().parent))
from heading_turn_metrics import analyze


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('input',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--plot',action='store_true')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    outcomes=json.loads((args.input/'outcomes.json').read_text())
    summaries=[];hashes={}
    previous_hashes=json.loads((args.output/'raw-sha256.json').read_text()) if (args.output/'raw-sha256.json').exists() else {}
    for trial in sorted(outcomes,key=lambda t:t['id']):
        source=args.input/trial['id'];target=args.output/trial['id'];target.mkdir(exist_ok=True)
        item=dict(trial)
        if (source/'samples.jsonl').exists() and (source/'sequence.json').exists():
            try:item['metrics']=analyze(source)
            except (ValueError,KeyError,StopIteration) as exc:item['analysis_error']=str(exc)
        for filename in ['recipe.json','sensors.yaml','sequence.json','capture.json','provenance.json',
                         'global.yaml','controller-parameters.yaml','local-ekf-parameters.yaml','global-ekf-parameters.yaml',
                         'gps-map-parameters.yaml','gps-parameters.yaml','robot-localization-version.txt',
                         'sim_drive_sensor_adapter-parameters.yaml','sim_imu_from_odom-parameters.yaml',
                         'sim_gps_normalizer-parameters.yaml','turn-metrics.json','run.log']:
            if (source/filename).exists():shutil.copyfile(source/filename,target/filename)
        if (source/'samples.jsonl').exists():
            raw=(source/'samples.jsonl').read_bytes()
            hashes[trial['id']]=dict(sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw))
            compact=target/'analysis-samples.jsonl.gz'
            previous=previous_hashes.get(trial['id'],{})
            if not (compact.exists() and previous.get('sha256')==hashes[trial['id']]['sha256']):
                analysis=[]
                for line in raw.decode().splitlines():
                    row=json.loads(line)
                    for key in ['pose_covariance','twist_covariance']:
                        if key in row:
                            values=row.pop(key)
                            row[key+'_diagonal']=[values[i] for i in (0,7,14,21,28,35)]
                    analysis.append(json.dumps(row,separators=(',',':')))
                compact.write_bytes(gzip.compress(('\n'.join(analysis)+'\n').encode(),compresslevel=6,mtime=0))
            hashes[trial['id']]['analysis_sha256']=hashlib.sha256(compact.read_bytes()).hexdigest()
        capture=json.loads((source/'capture.json').read_text()) if (source/'capture.json').exists() else {}
        item['capture_success']=capture.get('success',False)
        item['outcome']='passed' if item['returncode']==0 else ('harness_failure_after_complete_capture' if item['capture_success'] else 'capture_failure')
        summaries.append(item)
    analysis_provenance=dict(head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        source_sha256={name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in ['summarize_turn_campaign.py','heading_turn_metrics.py']},
        data_encoding='all samples, only covariance off-diagonal entries omitted; unmodified raw sha256 provided')
    (args.output/'analysis-provenance.json').write_text(json.dumps(analysis_provenance,indent=2))
    (args.output/'summary.json').write_text(json.dumps(summaries,indent=2,allow_nan=False))
    (args.output/'raw-sha256.json').write_text(json.dumps(hashes,indent=2))
    with (args.output/'summary.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['id','kind','speed','requested_radius_m','manual_envelope','reference_kind','observed_radius_min_m','observed_radius_median_m','geometric_radius_p05_m','steady_radius_median_m','profile','seed','variant','returncode','valid','local_heading_rms_deg','global_heading_rms_deg','global_heading_max_deg','global_position_rms_m','global_position_max_m','tracking_rms_m','global_recovery_s','global_recovery_censored'])
        for s in summaries:
            m=s.get('metrics',{});local=m.get('localization',{}).get('/odometry/local',{});glob=m.get('localization',{}).get('/odometry/global',{})
            a=glob.get('all',{});rec=glob.get('recovery',{})
            writer.writerow([s['id'],s['kind'],s['speed'],s.get('radius',4.),s.get('manual_envelope',False),m.get('reference_kind'),m.get('motion',{}).get('observed_radius_m',{}).get('minimum'),m.get('motion',{}).get('observed_radius_m',{}).get('median'),m.get('motion',{}).get('geometric_radius_m',{}).get('p05'),m.get('motion',{}).get('steady_geometric_radius_m',{}).get('median'),s['profile'],s['seed'],s.get('variant','baseline'),s['returncode'],m.get('instrumentation_valid',False),local.get('all',{}).get('heading_deg',{}).get('rms'),a.get('heading_deg',{}).get('rms'),a.get('heading_deg',{}).get('maximum'),a.get('position_m',{}).get('rms'),a.get('position_m',{}).get('maximum'),m.get('tracking_command_model_m',{}).get('rms'),rec.get('seconds'),rec.get('censored')])
    groups={}
    for s in summaries:
        key=f"{s['kind']}/v{s['speed']}/R{s.get('radius',4)}/{s['profile']}/{s.get('variant','baseline')}/{'manual30' if s.get('manual_envelope') else 'auto18'}"
        groups.setdefault(key,[]).append(s)
    aggregates={}
    for key,group in groups.items():
        good=[s['metrics']['localization']['/odometry/global']['all'] for s in group if s.get('metrics',{}).get('instrumentation_valid') and s['returncode']==0]
        def spread(values):
            return dict(median=statistics.median(values),minimum=min(values),maximum=max(values),range=max(values)-min(values)) if values else None
        aggregates[key]=dict(attempts=len(group),successful=len(good),heading_rms_deg=spread([m['heading_deg']['rms'] for m in good]),heading_max_deg=spread([m['heading_deg']['maximum'] for m in good]),position_rms_m=spread([m['position_m']['rms'] for m in good]))
    (args.output/'aggregates.json').write_text(json.dumps(aggregates,indent=2))
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,1,figsize=(max(12,len(aggregates)*.34),12),layout='constrained')
        labels=list(aggregates)
        for ax,metric,label in [(axes[0],'heading_rms_deg','Global heading RMS [deg]'),(axes[1],'position_rms_m','Global position RMS [m]')]:
            for i,key in enumerate(labels):
                value=aggregates[key][metric]
                if value:
                    ax.errorbar(i,value['median'],yerr=[[value['median']-value['minimum']],[value['maximum']-value['median']]],fmt='o',capsize=3)
                else:ax.text(i,0,'FAIL',rotation=90)
            ax.set_ylabel(label);ax.set_xticks(range(len(labels)),[s.replace('/','\n') for s in labels],rotation=0,ha='center',fontsize=6);ax.grid(alpha=.3)
        references=sorted(set(s.get('metrics',{}).get('reference_kind','unavailable') for s in summaries))
        fig.suptitle('SALUS #328 — '+', '.join(references)+'\nMedian and observed range; failed trials retained in summary',fontsize=11)
        fig.savefig(args.output/'comparison.png',dpi=150);fig.savefig(args.output/'comparison.svg')
        plt.close(fig)
    print(json.dumps(aggregates,indent=2))


if __name__=='__main__':main()
