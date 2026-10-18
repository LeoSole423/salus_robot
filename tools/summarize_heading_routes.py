#!/usr/bin/env python3
"""Build a comparison of automatic-route outcomes without dropping failed trials."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaigns', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for campaign in args.campaigns:
        for source in sorted(campaign.glob('trials/*/summary.json')):
            summary = json.loads(source.read_text())
            metadata = summary.get('matrix_trial', {})
            row = dict(campaign=campaign.name, trial=source.parent.name,
                       reason=summary.get('reason'),
                       setup_failure='gates' not in summary,
                       nominal_speed_mps=metadata.get('requested_speed_mps'),
                       requested_radius_m=metadata.get('requested_radius_m'),
                       seed=metadata.get('sim_sensor_seed'),
                       functional_pass=bool(summary.get('gates')) and all(
                           g['state'] == 'pass' for g in summary.get('gates', [])))
            if not row['setup_failure']:
                side = metadata['direction']
                result = subprocess.run([
                    sys.executable, str(Path(__file__).with_name('heading_route_metrics.py')),
                    str(source.parent), '--side', side, '--radius',
                    str(metadata['requested_radius_m'] or 4.), '--kind',
                    'straight' if side == 'straight' else ('s' if source.parent.name.startswith('s_') else 'quarter')],
                    capture_output=True, text=True)
                if result.returncode == 0:
                    row['curve'] = json.loads(result.stdout)
                else:
                    row['analysis_failure'] = result.stderr
                row['parameter_capture'] = metadata.get('parameter_capture')
                row['reference_observation'] = summary['reference_observation']
                curve = row.get('curve', {})
                capture = row.get('parameter_capture') or {}
                row['selection_eligible'] = bool(
                    row['functional_pass'] and curve.get('completed_route') and
                    curve.get('heading_acquired_before_curve') and
                    curve.get('compensated_gnss_continuous_before_curve') and
                    curve.get('commands_positive_below_1p2_fraction') == 0. and
                    capture.get('returncode') == 0)
            rows.append(row)
    (args.output/'summary.json').write_text(json.dumps(rows, indent=2)+'\n')
    with (args.output/'summary.csv').open('w', newline='') as stream:
        fields = ['campaign', 'trial', 'reason', 'setup_failure', 'functional_pass',
                  'nominal_speed_mps', 'requested_radius_m', 'seed', 'selection_eligible',
                  'completed_curve', 'heading_acquired_before_curve',
                  'compensated_gnss_continuous_before_curve',
                  'curve_speed_p05_mps', 'curve_speed_median_mps',
                  'commands_positive_below_1p2_fraction']
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            values = dict(row, **row.get('curve', {}))
            writer.writerow({k: values.get(k) for k in fields})
    provenance = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                  for name in ['summarize_heading_routes.py', 'heading_route_metrics.py',
                               'heading_turn_metrics.py']}
    (args.output/'analysis-provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    print(json.dumps(dict(attempts=len(rows), functional_pass=sum(
        r['functional_pass'] for r in rows), completed_curves=sum(
            bool(r.get('curve', {}).get('completed_curve')) for r in rows))))


if __name__ == '__main__':
    main()
