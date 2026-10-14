#!/usr/bin/env python3
"""Attribute native robot_localization 3.5.4 corrections, not topic correlations."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import re

NUMBER = r'[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d{1,3})?'


def values(text):
    return [float(n) for n in re.findall(NUMBER, text)]


def array_after(block, label):
    match = re.search(re.escape(label)+r'\s*\[([^]]*)\]', block, re.S)
    if not match:
        raise ValueError(f'missing {label}')
    return values(match[1])


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def parse_correction(block):
    if 'Ekf::correct ' not in block:
        return None  # Initializer, not a Kalman correction.
    stamp = re.search(r'Measurement time is (\d+)', block)
    source = re.search(r'FilterBase::processMeasurement \(([^)]+)\)', block)
    if not stamp or not source:
        raise ValueError('missing measurement stamp/source')
    correction = block.split('Ekf::correct ', 1)[1]
    before = array_after(correction, 'State is:')
    after = array_after(correction, 'Corrected full state is:')
    innovation = array_after(correction, 'Innovation is:')
    gain = array_after(correction, 'Kalman gain subset is:')
    indices = [int(n) for n in array_after(correction, 'Update indices are:')]
    if len(before) != 15 or len(after) != 15 or len(indices) != len(innovation) or len(gain) != 15*len(indices):
        raise ValueError('unexpected state/matrix dimensions')
    if not all(math.isfinite(n) for n in before+after+innovation+gain):
        raise ValueError('nonfinite filter data')
    yaw_gain = gain[5*len(indices):6*len(indices)]
    contributions = [k*residual for k,residual in zip(yaw_gain, innovation)]
    actual_delta = angle(after[5]-before[5])
    predicted_delta = angle(sum(contributions))
    covariance = array_after(correction, 'Corrected full estimate error covariance is:')
    if len(covariance) != 225:
        raise ValueError('unexpected covariance dimensions')
    return dict(stamp_s=int(stamp[1])/1e9, source=source[1],
        yaw_before_rad=before[5], yaw_after_rad=after[5],
        yaw_delta_rad=actual_delta, k_innovation_yaw_rad=predicted_delta,
        reconciliation_error_rad=angle(actual_delta-predicted_delta),
        update_indices=indices, innovation=innovation, yaw_gain=yaw_gain,
        yaw_contributions_rad=dict(zip(map(str, indices), contributions)),
        yaw_variance_after=covariance[5*15+5])


def parse_log(path):
    stats = dict(measurements=0, initializations=0, corrections=0, malformed=0, incomplete=0)
    events = []
    block = None
    with Path(path).open() as stream:
        for line in stream:
            if line.startswith('------ FilterBase::processMeasurement ('):
                if block is not None:
                    stats['incomplete'] += 1
                block = [line]
            elif block is not None:
                block.append(line)
                if line.startswith('------ /FilterBase::processMeasurement ('):
                    stats['measurements'] += 1
                    try:
                        event = parse_correction(''.join(block))
                        if event is None:
                            stats['initializations'] += 1
                        else:
                            events.append(event)
                            stats['corrections'] += 1
                    except ValueError:
                        stats['malformed'] += 1
                    block = None
    if block is not None:
        stats['incomplete'] += 1
    return events, stats


def summarize(events, begin, end):
    sources = defaultdict(list)
    for e in events:
        if begin <= e['stamp_s'] <= end:
            sources[e['source']].append(e)
    output = {}
    for source, rows in sources.items():
        by_index = defaultdict(float)
        for e in rows:
            for index, delta in e['yaw_contributions_rad'].items():
                by_index[index] += math.degrees(delta)
        output[source] = dict(corrections=len(rows),
            net_yaw_correction_deg=math.degrees(sum(e['yaw_delta_rad'] for e in rows)),
            abs_yaw_correction_sum_deg=math.degrees(sum(abs(e['yaw_delta_rad']) for e in rows)),
            largest_yaw_correction_deg=math.degrees(max(abs(e['yaw_delta_rad']) for e in rows)),
            net_contribution_by_state_index_deg=dict(by_index),
            max_reconciliation_error_deg=math.degrees(max(abs(e['reconciliation_error_rad']) for e in rows)))
    return output


def analyze(directory):
    directory = Path(directory)
    seq = json.loads((directory/'sequence.json').read_text())
    start = seq['start_sim_s']
    samples = [json.loads(line) for line in (directory/'samples.jsonl').read_text().splitlines()]
    gate = [r for r in samples if r['topic']=='/gps/course_heading/debug' and r.get('turn_state')=='STRONG_TURN']
    intervals = dict(maneuver=(start+8, start+14.3))
    if gate:
        intervals['strong_turn_observed']=(gate[0]['stamp_s'],gate[-1]['stamp_s'])
    result = dict(schema_version=1, start_sim_s=start, intervals=intervals, filters={})
    for name in ('local', 'global', 'no_gps_position', 'compensated_gps', 'delayed_gps_reference'):
        if not (directory/f'{name}-ekf-debug.log').exists():
            if name in ('local','global'):
                raise ValueError(f'missing required {name} debug log')
            continue
        events, stats = parse_log(directory/f'{name}-ekf-debug.log')
        with (directory/f'{name}-corrections.jsonl').open('w') as stream:
            for event in events:
                stream.write(json.dumps(event)+'\n')
        result['filters'][name] = dict(parser=stats,
            intervals={label:summarize(events,*bounds) for label,bounds in intervals.items()},
            worst_corrections=sorted([e for e in events if start+8<=e['stamp_s']<=start+14.3],
                                    key=lambda e:abs(e['yaw_delta_rad']),reverse=True)[:10])
        if stats['malformed'] or stats['incomplete'] or not events:
            raise ValueError(f'{name}: incomplete/unrecognized native log: {stats}')
        if max(abs(e['reconciliation_error_rad']) for e in events)>0.0003:
            raise ValueError('K innovation and state correction do not reconcile within native print precision')
    (directory/'innovations.json').write_text(json.dumps(result,indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    args = parser.parse_args()
    print(json.dumps(analyze(args.directory),indent=2))
