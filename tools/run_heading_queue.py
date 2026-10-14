#!/usr/bin/env python3
"""Queue open-loop heading captures using the navigation domain allocator.

The Nav2 matrix runner owns closed-loop goals; this queue preserves the exact
open-loop reference instead. Default serial execution avoids resource overload.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'src/salus_evaluation'))
from salus_evaluation.isolation import allocated_trial_isolation


def run_trial(item):
    workspace, prefix, scenario = item
    trial = f'{prefix}-{scenario}'
    with allocated_trial_isolation(REPO/'artifacts'/trial,
            run_token=prefix, trial_id=scenario) as isolation:
        result = subprocess.run(['bash', str(REPO/'tools/run_heading_capture.sh'),
            workspace, trial, str(isolation.ros_domain_id), scenario], cwd=REPO)
        return dict(scenario=scenario, domain=isolation.ros_domain_id,
                    returncode=result.returncode, artifact=f'artifacts/{trial}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace')
    parser.add_argument('prefix')
    parser.add_argument('--jobs', type=int, default=1)
    parser.add_argument('--scenarios', nargs='+', choices=['right','left','gentle','oscillations'],
                        default=['right','left','gentle','oscillations'])
    args = parser.parse_args()
    if not 1 <= args.jobs <= 4:
        parser.error('jobs must be in 1..4; validate isolation before parallel runs')
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        outcomes = list(pool.map(run_trial, [(args.workspace,args.prefix,s) for s in args.scenarios]))
    output = REPO/'artifacts'/f'{args.prefix}-queue.json'
    output.write_text(json.dumps(outcomes, indent=2))
    return int(any(o['returncode'] for o in outcomes))


if __name__ == '__main__':
    sys.exit(main())
