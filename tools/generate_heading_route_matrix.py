#!/usr/bin/env python3
"""Generate portable ordered-route experiments without altering product defaults."""
import argparse
import math
from pathlib import Path
import yaml


def route_goals(kind, radius, side):
    if not math.isfinite(radius) or radius < .94/math.tan(math.radians(30)):
        raise ValueError('radius exceeds configured manual steering envelope')
    points = [('acquire', 20., 0., 0.), ('entry', 28., 0., 0.)]
    if kind == 'straight':
        points += [('exit', 60., 0., 0.)]
    else:
        points += [('curve_end', 28.+radius, side*radius, side*math.pi/2)]
        if kind == 's':
            points += [('second_end', 28.+2*radius, side*2*radius, 0.),
                       ('exit', 44.+2*radius, side*2*radius, 0.)]
        else:
            points += [('exit', 28.+radius, side*(radius+16.), side*math.pi/2)]
    return [dict(id=name, forward_m=x, lateral_m=y, yaw_offset_rad=yaw,
                 timeout_s=120., expected_turn='any') for name, x, y, yaw in points]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--radius', type=float, default=4.)
    parser.add_argument('--speeds', nargs='+', type=float, default=[1.2, 1.6, 1.8])
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--seed', type=int, default=6500)
    parser.add_argument('--kind', choices=['quarter', 'straight', 's'], default='quarter')
    parser.add_argument('--profile', choices=['clean', 'independent_nominal', 'degraded'],
                        default='independent_nominal')
    args = parser.parse_args()
    if any(not math.isfinite(v) or not 1.2 <= v <= 2. for v in args.speeds):
        parser.error('nominal speed must be within [1.2,2.0] m/s')
    if args.repetitions < 1 or args.seed < 0:
        parser.error('positive repetitions and nonnegative seed required')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'scenarios').mkdir(); (args.output/'matrices').mkdir()
    cases = []
    for side, sign in ([('straight', 1)] if args.kind == 'straight' else
                       [('left', 1), ('right', -1)]):
        name = f'{args.kind}_{side}'
        scenario = dict(schema_version=1, id=name, world='free',
                        spawn=dict(x_m=0., y_m=0., yaw_rad=0.),
                        goals=route_goals(args.kind, args.radius, sign))
        (args.output/'scenarios'/f'{name}.yaml').write_text(yaml.safe_dump(scenario))
        cases.append(dict(id=name, scenario=f'scenarios/{name}.yaml', direction=side,
                          requested_radius_m=None if side == 'straight' else args.radius))
    matrix = dict(schema_version=1, id=args.output.name, repetitions=args.repetitions,
                  max_speed_mps=max(args.speeds), speeds_mps=args.speeds, cases=cases,
                  sim_sensor_profile=args.profile, sim_sensor_seed=args.seed)
    target = args.output/'matrices/matrix.yaml'
    target.write_text(yaml.safe_dump(matrix, sort_keys=False))
    print(target)


if __name__ == '__main__':
    main()
