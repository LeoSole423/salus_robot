#!/usr/bin/env python3
"""Serial, isolated #328 experiment queue. No hardware or product defaults."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src/salus_evaluation'))
from salus_evaluation.isolation import allocated_trial_isolation


def sequence(kind, speed, radius=4.0):
    if kind not in ('straight', 'oscillations', 'right', 'left', 's', 'half'):
        raise ValueError(kind)
    if not 0.8 <= speed <= 1.6 or radius < 4 or not math.isfinite(radius):
        raise ValueError('campaign requires 0.8..1.6 m/s and R>=4 m')
    turn = math.pi / 2 * radius / speed
    rate = speed / radius
    phases = [('straight_before', 8., speed, 0.)]
    if kind in ('straight', 'oscillations'):
        phases += [('cruise', 8., speed, 0.)]
    else:
        phases += [('entry', 1., speed, rate if kind == 'left' else -rate),
                   ('curve', (2*turn if kind == 'half' else turn)-1., speed,
                    rate if kind == 'left' else -rate)]
        if kind == 's':
            phases += [('curve_reverse', turn, speed, rate)]
    return phases + [('straight_after', 8., speed, 0.), ('stop', 3., 0., 0.)]


def sensor_profile(intervention):
    base = yaml.safe_load((REPO/'src/salus_simulation/config/sensor_profiles/clean.yaml').read_text())
    nominal = yaml.safe_load((REPO/'src/salus_simulation/config/sensor_profiles/independent_nominal.yaml').read_text())
    if intervention == 'nominal':
        return nominal
    for value in base.values():
        value['ros__parameters']['sim_sensor_profile'] = 'independent_nominal'
    if intervention == 'gyro':
        for key in ('imu.gyro_bias_rad_s', 'imu.gyro_noise_stddev_rad_s'):
            base['sim_imu_from_odom']['ros__parameters'][key] = nominal['sim_imu_from_odom']['ros__parameters'][key]
    elif intervention == 'wheel':
        for key in ('wheel.traction_scale','wheel.noise_stddev_mps','steering.bias_deg','steering.noise_stddev_deg'):
            base['sim_drive_sensor_adapter']['ros__parameters'][key] = nominal['sim_drive_sensor_adapter']['ros__parameters'][key]
    elif intervention in ('latency100', 'latency300'):
        base['sim_gps_normalizer']['ros__parameters']['gnss.latency_s'] = .1 if intervention == 'latency100' else .3
    elif intervention == 'outage':
        # Absolute windows set after the probe establishes its start, via a
        # simulation-only relay would change product wiring. Use repeated
        # deterministic windows and report their actual intersection instead.
        base['sim_gps_normalizer']['ros__parameters']['gnss.forced_dropout_start_s'] = [float(i) for i in range(10,100,12)]
        base['sim_gps_normalizer']['ros__parameters']['gnss.forced_dropout_end_s'] = [float(i+4) for i in range(10,100,12)]
    elif intervention != 'clean':
        raise ValueError(intervention)
    return base


def run(args):
    root = REPO / args.output
    root.mkdir(parents=True, exist_ok=False)
    recipes = json.loads(Path(args.matrix).read_text())
    names=[t['id'] for t in recipes]
    if len(names)!=len(set(names)) or any(not n or Path(n).name!=n or n in ('.','..') for n in names):
        raise ValueError('trial ids must be unique simple directory names')
    outcomes = []
    def execute(trial):
        name = trial['id']
        directory = root/name
        directory.mkdir()
        recipe = dict(trial, sequence=sequence(trial['kind'], trial['speed'], trial.get('radius',4.)))
        (directory/'recipe.json').write_text(json.dumps(recipe,indent=2))
        profile = sensor_profile(trial['profile'])
        (directory/'sensors.yaml').write_text(yaml.safe_dump(profile))
        env = dict(os.environ, HEADING_CPUS='6', HEADING_NETWORK='bridge', HEADING_FASTDDS_TRANSPORTS='UDPv4', HEADING_RECIPE='/ros2_ws/'+str((directory/'recipe.json').relative_to(REPO)),
                   HEADING_SEED=str(trial['seed']), HEADING_SENSOR_FILE=str(directory/'sensors.yaml'))
        if trial.get('variant','baseline') not in ('baseline','lagged'):
            raise ValueError('unknown EKF variant')
        if trial.get('variant') == 'lagged':
            config = yaml.safe_load((REPO/'src/salus_localization/config/localization_global_sim.yaml').read_text())
            config['ekf_filter_node_global']['ros__parameters'].update(smooth_lagged_data=True, history_length=1.0)
            (directory/'global.yaml').write_text(yaml.safe_dump(config))
            env['HEADING_GLOBAL_CONFIG'] = '/ros2_ws/'+str((directory/'global.yaml').relative_to(REPO))
        with allocated_trial_isolation(directory, run_token=root.name, trial_id=name) as isolation:
            rel = str(directory.relative_to(REPO/'artifacts'))
            result = subprocess.run(['bash',str(REPO/'tools/run_heading_capture.sh'),str(REPO),rel,
                                     str(isolation.ros_domain_id),
                                     'oscillations' if trial['kind']=='oscillations' else 'right',
                                     'independent_nominal'],env=env,cwd=REPO,timeout=300)
            outcome=dict(trial, returncode=result.returncode,domain=isolation.ros_domain_id)
        print(name, result.returncode, flush=True)
        return outcome
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        pending={pool.submit(execute,t): t for t in recipes}
        for future in as_completed(pending):
            try: outcomes.append(future.result())
            except Exception as exc: outcomes.append(dict(pending[future],returncode=99,error=str(exc)))
            (root/'outcomes.json').write_text(json.dumps(outcomes,indent=2))
    return int(any(o['returncode'] for o in outcomes))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--jobs',type=int,choices=[1,2],default=1)
    p.add_argument('matrix'); p.add_argument('output',help='new directory below artifacts/')
    a=p.parse_args()
    if not a.output.startswith('artifacts/'):
        p.error('output must be under artifacts/')
    sys.exit(run(a))
