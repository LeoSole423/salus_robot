#!/usr/bin/env python3
"""Summarize #328 Nav2 decisions separately from physical localization/tracking."""
import argparse
import ast
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path

from heading_turn_metrics import Interpolator, angle, stats


def read_csv(path):
    return list(csv.DictReader(path.open()))


def poses(path):
    result = []
    for item in read_csv(path):
        pose = ast.literal_eval(item['pose'])
        result.append(dict(stamp_s=float(item['stamp_s']), x=pose['x_m'],
                           y=pose['y_m'], yaw_rad=pose['yaw_rad'],
                           speed=float(item['linear_x_mps'])))
    return result


def summarize(root):
    results = []
    for source in sorted(root.glob('trials/*/summary.json')):
        folder = source.parent
        summary = json.loads(source.read_text())
        if 'gates' not in summary:
            results.append(dict(trial=folder.name, functional_pass=False,
                                failed_gates=['matrix_setup'],
                                errors=summary.get('errors', []),
                                terminal_status=summary.get('terminal_status'),
                                matrix_trial=summary.get('matrix_trial', {})))
            continue
        truth = poses(folder/'odometry_raw.csv')
        interpolation = Interpolator(truth)
        localization = {}
        for name in ['local', 'global']:
            estimated = poses(folder/f'odometry_{name}.csv')
            paired = [(r, interpolation(r['stamp_s'])) for r in estimated]
            available = [(r, t) for r, t in paired if t is not None]
            localization[name] = dict(
                heading_deg=stats([abs(math.degrees(angle(r['yaw_rad'], t['yaw_rad'])))
                                   for r, t in available]),
                position_m=stats([math.hypot(r['x']-t['x'], r['y']-t['y'])
                                  for r, t in available]),
                unpaired=len(paired)-len(available))
        health = Counter(r['reason'] for r in read_csv(folder/'path_health.csv'))
        gps = Counter(json.loads(r['data']).get('reason') for r in
                      read_csv(folder/'gnss_map_debug.csv'))
        manifest = json.loads((folder/'manifest.json').read_text())
        results.append(dict(
            trial=folder.name, ground_truth_topic=manifest['ground_truth_topic'],
            functional_pass=all(g['state'] == 'pass' for g in summary['gates']),
            failed_gates=[g['name'] for g in summary['gates'] if g['state'] != 'pass'],
            terminal_status=summary['terminal_status'], replans=summary['replans'],
            path_health_counts=dict(health), gps_map_reason_counts=dict(gps),
            physical_speed_max_mps=max(abs(r['speed']) for r in truth),
            localization=localization, reference_observation=summary['reference_observation'],
            operational_tolerance_m=summary['operational_tolerance_m']))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(summarize(args.input), indent=2)+'\n')
    sources = ['summarize_heading_navigation.py', 'heading_turn_metrics.py']
    args.output.with_name(args.output.stem+'-provenance.json').write_text(json.dumps(
        {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
         for name in sources}, indent=2)+'\n')


if __name__ == '__main__':
    main()
