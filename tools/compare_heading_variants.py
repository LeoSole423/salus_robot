#!/usr/bin/env python3
"""Pair #328 trial summaries by physical recipe and seed, retaining failures."""
import argparse
import json
from pathlib import Path


def compare(summaries):
    pairs = {}
    for trial in summaries:
        key = tuple(trial.get(k) for k in
                    ('kind', 'speed', 'radius', 'profile', 'seed', 'manual_envelope', 'hold_s'))
        pairs.setdefault(key, {})[trial.get('variant', 'baseline')] = trial
    results = []
    for key, variants in sorted(pairs.items(), key=lambda item: str(item[0])):
        if 'baseline' not in variants or 'lagged' not in variants:
            continue
        row = dict(zip(('kind', 'speed', 'radius', 'profile', 'seed',
                        'manual_envelope', 'hold_s'), key))
        row['comparison_eligible'] = all(
            t['returncode'] == 0 and t['metrics']['instrumentation_valid']
            for t in variants.values())
        for name, trial in variants.items():
            m = trial['metrics']
            g = m['localization']['/odometry/global']
            row[name] = dict(id=trial['id'], returncode=trial['returncode'],
                             heading_max_deg=g['all']['heading_deg']['maximum'],
                             heading_rms_deg=g['all']['heading_deg']['rms'],
                             position_rms_m=g['all']['position_m']['rms'],
                             recovery=g['recovery'],
                             continuity=m['continuity']['/odometry/global'])
        row['position_rms_reduction_fraction'] = (
            1-row['lagged']['position_rms_m']/row['baseline']['position_rms_m'])
        results.append(row)
    return results


def plot(pairs, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(max(10, len(pairs)*.8), 10),
                             layout='constrained')
    labels = [f"{p['kind']} v{p['speed']} R{p['radius']}\n{p['profile']} s{p['seed']}"+
              ("\nEXPORT FAILED" if not p['comparison_eligible'] else "") for p in pairs]
    for ax, field, ylabel in zip(axes,
            ['position_rms_m', 'heading_max_deg', 'recovery'],
            ['Global position RMS [m]', 'Global heading max [deg]', 'Recovery [s]']):
        for variant, offset, color in [('baseline', -.18, 'C0'), ('lagged', .18, 'C1')]:
            values = [p[variant][field] if field != 'recovery' else
                      p[variant][field].get('seconds') for p in pairs]
            bars = ax.bar([i+offset for i in range(len(pairs))],
                          [v if v is not None else 0 for v in values], .35,
                          label=variant, color=color)
            for bar, p, value in zip(bars, pairs, values):
                if not p['comparison_eligible'] or value is None:
                    bar.set_hatch('//'); bar.set_alpha(.4)
        ax.set_ylabel(ylabel); ax.grid(axis='y', alpha=.3)
        ax.set_xticks(range(len(pairs)), labels, fontsize=7)
        ax.legend()
    fig.suptitle('SALUS #328 — paired seeds; failed pairs excluded from selection')
    fig.savefig(output.with_suffix('.png'), dpi=160)
    fig.savefig(output.with_suffix('.svg'))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summaries', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    trials = [trial for path in args.summaries for trial in json.loads(path.read_text())]
    pairs = compare(trials)
    args.output.write_text(json.dumps(pairs, indent=2)+'\n')
    if args.plot:
        plot(pairs, args.output)


if __name__ == '__main__':
    main()
