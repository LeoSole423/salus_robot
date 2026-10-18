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
                    curve.get('map_output_continuous_before_curve') and
                    curve.get('commands_positive_below_1p2_fraction') == 0. and
                    capture.get('returncode') == 0)
                row['comparison_eligible'] = row['selection_eligible']
                available = curve.get('stream_availability', {})
                fresh = all(available.get(name, {}).get('max_gap_s', float('inf')) <= .2 and
                            available.get(name, {}).get('covered_fraction', 0.) >= .9
                            for name in ['local', 'global', 'imu'])
                imu_age = available.get('imu', {}).get('max_receive_age_s')
                row['recommendation_eligible'] = bool(
                    row['comparison_eligible'] and fresh and imu_age is not None and
                    -.05 <= imu_age <= .2 and curve.get('compensation_enabled') is True and
                    curve.get('kinematic_consistency', {}).get('planner_respects_model') and
                    curve.get('commands_stop_samples') == 0)

            rows.append(row)
    (args.output/'summary.json').write_text(json.dumps(rows, indent=2)+'\n')
    with (args.output/'summary.csv').open('w', newline='') as stream:
        fields = ['campaign', 'trial', 'reason', 'setup_failure', 'functional_pass',
                  'nominal_speed_mps', 'requested_radius_m', 'seed', 'selection_eligible', 'recommendation_eligible',
                  'completed_curve', 'heading_acquired_before_curve',
                  'compensated_gnss_continuous_before_curve',
                  'curve_speed_p05_mps', 'curve_speed_median_mps',
                  'commands_positive_below_1p2_fraction',
                  'physical_radius_median_m', 'gnss_delivery_age_median_s',
                  'local_heading_max_deg', 'local_heading_rms_deg',
                  'local_position_max_m', 'local_position_rms_m',
                  'global_heading_max_deg', 'global_heading_rms_deg',
                  'global_position_max_m', 'global_position_rms_m',
                  'causal_tracking_rms_m', 'causal_tracking_max_m',
                  'local_max_gap_s', 'global_max_gap_s', 'imu_max_gap_s']
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            values = dict(row, **row.get('curve', {}))
            values['physical_radius_median_m'] = values.get('central_physical_radius_m', {}).get('median')
            values['gnss_delivery_age_median_s'] = values.get('gnss_observed_delivery_age_s', {}).get('median')
            for name in ['local', 'global']:
                pose = values.get('curve_localization', {}).get(name, {})
                for quantity, unit in [('heading', 'deg'), ('position', 'm')]:
                    metric = pose.get(quantity+'_'+unit, {})
                    values[f'{name}_{quantity}_max_{unit}'] = metric.get('maximum')
                    values[f'{name}_{quantity}_rms_{unit}'] = metric.get('rms')
            tracking = row.get('reference_observation', {}).get('causal_published_plan_tracking', {})
            values['causal_tracking_rms_m'] = tracking.get('cross_track_rms_m')
            values['causal_tracking_max_m'] = tracking.get('cross_track_max_m')
            for name in ['local', 'global', 'imu']:
                values[name+'_max_gap_s'] = values.get('stream_availability', {}).get(name, {}).get('max_gap_s')
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
