#!/usr/bin/env python3
"""Evaluate completed physical curves separately from Nav2 terminal success."""
import argparse
import ast
import csv
import json
import math
from pathlib import Path

from heading_turn_metrics import Interpolator, angle, stats, circle_radius, distribution


def route_observation(poses, course_debug, map_debug, *, entry_x=28., radius=4., side=1):
    """Score a fixed-datum quarter turn without using truth for runtime control."""
    entered = next((r['stamp_s'] for r in poses if r['x_m'] >= entry_x-1), None)
    curve = [r for r in poses if r['x_m'] >= entry_x-1 and
             side*r['y_m'] <= radius+1 and abs(r['speed']) > .1]
    exits = [r for r in poses if side*r['y_m'] >= radius+6]
    degrees = [side*math.degrees(r['yaw_rad']) for r in curve]
    speed = sorted(abs(r['speed']) for r in curve)
    heading = []
    fixes = []
    for rows, target in [(course_debug, heading), (map_debug, fixes)]:
        for row in rows:
            stamp = float(row['stamp_s'])
            if entered is not None and entered-2 <= stamp <= entered:
                target.append((stamp, json.loads(row['data'])))
    map_valid = bool(fixes) and all(r.get('reason') == 'ok' for _, r in fixes)
    coverage = bool(fixes) and fixes[-1][0]-fixes[0][0] >= 1.7
    acquired = bool(heading) and any(r.get('valid', False) for _, r in heading)
    return dict(entry_stamp_s=entered, heading_acquired_before_curve=acquired,
                compensated_gnss_continuous_before_curve=bool(map_valid and coverage),
                completed_curve=bool(degrees and max(degrees) >= 81 and exits),
                turn_max_deg=max(degrees) if degrees else None,
                curve_samples=len(curve),
                curve_speed_p05_mps=speed[int(.05*(len(speed)-1))] if speed else None,
                curve_speed_median_mps=speed[len(speed)//2] if speed else None,
                curve_speed_below_1p2_fraction=(sum(v < 1.2 for v in speed)/len(speed)
                                               if speed else None))


def read(path):
    return list(csv.DictReader(path.open()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('--radius', type=float, default=4.)
    parser.add_argument('--side', choices=['left', 'right'], required=True)
    args = parser.parse_args()
    rows = []
    for row in read(args.bundle/'odometry_raw.csv'):
        pose = ast.literal_eval(row['pose'])
        rows.append(dict(pose, stamp_s=float(row['stamp_s']),
                         speed=float(row['linear_x_mps'])))
    result = route_observation(rows, read(args.bundle/'gnss_heading_debug.csv'),
                               read(args.bundle/'gnss_map_debug.csv'),
                               radius=args.radius, side=1 if args.side == 'left' else -1)
    start = result['entry_stamp_s']
    curve = [r for r in rows if r['x_m'] >= 27 and
             (1 if args.side == 'left' else -1)*r['y_m'] <= args.radius+1 and
             abs(r['speed']) > .1]
    end = max((r['stamp_s'] for r in curve), default=None)
    result['curve_end_stamp_s'] = end
    truth = Interpolator([dict(r, x=r['x_m'], y=r['y_m']) for r in rows])
    result['curve_localization'] = {}
    for name in ['local', 'global']:
        paired = []
        for row in read(args.bundle/f'odometry_{name}.csv'):
            stamp = float(row['stamp_s'])
            if start is None or end is None or not start <= stamp <= end:
                continue
            reference = truth(stamp)
            if reference is not None:
                paired.append((ast.literal_eval(row['pose']), reference))
        result['curve_localization'][name] = dict(
            heading_deg=stats([abs(math.degrees(angle(r['yaw_rad'], t['yaw_rad'])))
                               for r, t in paired]),
            position_m=stats([math.hypot(r['x_m']-t['x'], r['y_m']-t['y'])
                              for r, t in paired]))
    radii = []
    for row in curve:
        t = row['stamp_s']
        points = [truth(t+delta) for delta in [-.25, 0., .25]]
        if all(p is not None for p in points):
            radius = circle_radius(*points)
            if radius is not None:
                radii.append(radius)
    result['curve_physical_radius_m'] = distribution(radii)
    (args.bundle/'route-observation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
