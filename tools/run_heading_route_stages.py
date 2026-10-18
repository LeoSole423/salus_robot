#!/usr/bin/env python3
"""Run bounded route-research stages sequentially through the isolated matrix harness."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for stage in json.loads(args.manifest.read_text()):
        output = REPO/stage['output']
        if output.exists():
            raise ValueError('refusing to overwrite stage: '+str(output))
        prerequisite = stage.get('requires_completed')
        if prerequisite:
            eligible = []
            directions = set()
            root = REPO/prerequisite
            for path in root.glob('trials/*/route-observation.json'):
                observation = json.loads(path.read_text())
                if (observation.get('completed_route') and
                        observation.get('commands_positive_below_1p2_fraction') == 0.):
                    eligible.append(path)
                    metadata = json.loads((path.parent/'summary.json').read_text()).get('matrix_trial', {})
                    directions.add(metadata.get('direction'))
            if len(eligible) < 2 or not {'left', 'right'} <= directions:
                results.append(dict(id=stage['id'], outcome='skipped',
                                    reason='preceding geometry lacks two completed curves'))
                (args.output/'outcomes.json').write_text(json.dumps(results, indent=2))
                continue
        reference = stage.get('requires_valid_pair')
        if reference:
            rows = json.loads((REPO/reference).read_text())
            valid = [r for r in rows if r.get('selection_eligible') and
                     r['nominal_speed_mps'] == 1.8 and r['seed'] == 6501]
            if not (any(r['trial'].startswith('left-') for r in valid) and
                    any(r['trial'].startswith('right-') for r in valid)):
                results.append(dict(id=stage['id'], outcome='skipped',
                                    reason='1.8 m/s matched reference pair not valid'))
                (args.output/'outcomes.json').write_text(json.dumps(results, indent=2))
                continue
        flags = ['--jobs', '2', '--evaluation-ground-truth-topic', '/simulation/ground_truth']
        for field, flag in [('planner_radius', '--planner-minimum-turning-radius'),
                            ('nav2_params', '--experimental-nav2-params'),
                            ('launch_file', '--simulation-launch-file'),
                            ('trial_start_sim_s', '--trial-start-sim-s')]:
            if field in stage:
                value = stage[field]
                if field in ('nav2_params', 'launch_file'):
                    value = '/ros2_ws/'+value
                flags += [flag, str(value)]
        command = ['./tools/nav_eval.sh', 'matrix', stage['matrix'], stage['output']]+flags
        print(json.dumps(dict(stage=stage['id'], command=command)), flush=True)
        environment = dict(os.environ)
        if 'sensor_profile_file' in stage:
            environment['SALUS_NAV_EVAL_SENSOR_PROFILE_FILE'] = str(REPO/stage['sensor_profile_file'])
        with (args.output/(stage['id']+'.log')).open('w') as log:
            result = subprocess.run(command, cwd=REPO, env=environment,
                                    stdout=log, stderr=subprocess.STDOUT)
        analysis = subprocess.run([
            sys.executable, str(REPO/'tools/summarize_heading_routes.py'), str(output),
            '--output', str(output/'analysis')], cwd=REPO, capture_output=True, text=True)
        results.append(dict(id=stage['id'], outcome='executed', returncode=result.returncode,
                            analysis_returncode=analysis.returncode,
                            analysis_stdout=analysis.stdout, analysis_stderr=analysis.stderr))
        (args.output/'outcomes.json').write_text(json.dumps(results, indent=2))
        subprocess.run([sys.executable, str(REPO/'tools/archive_route_campaign.py'),
                        str(output), str(args.output/'evidence'/stage['id'])], check=True)
        print(json.dumps(results[-1]), flush=True)


if __name__ == '__main__':
    main()
