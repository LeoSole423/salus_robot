#!/usr/bin/env python3
"""Plot physical paths and the first published plan from navigation bundles."""
import argparse
import ast
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(args.captures), squeeze=False,
                             figsize=(6*len(args.captures), 6), layout='constrained')
    for ax, capture in zip(axes[0], args.captures):
        summary = json.loads((capture/'summary.json').read_text())
        plans = list(csv.DictReader((capture/'plans.csv').open()))
        first = [p for p in plans if p['plan_index'] == '0']
        if first:
            ax.plot([float(p['x_m']) for p in first], [float(p['y_m']) for p in first],
                    '--', label='First published plan')
        poses = [ast.literal_eval(p['pose']) for p in
                 csv.DictReader((capture/'odometry_raw.csv').open())]
        ax.plot([p['x_m'] for p in poses], [p['y_m'] for p in poses],
                label='Physical world pose')
        goal = summary['goal']
        ax.scatter([goal['x_m']], [goal['y_m']], marker='x', s=90, label='Requested goal')
        ax.set_title(capture.name.replace('-rep01', '')+'\n'+
                     f"terminal={summary['terminal_status']}, replans={summary['replans']}",
                     fontsize=9)
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel('map x [m]'); ax.set_ylabel('map y [m]')
        ax.grid(alpha=.3); ax.legend(fontsize=8)
    fig.suptitle('SALUS #328 — navigation, without changing maneuver protection')
    fig.savefig(args.output.with_suffix('.png'), dpi=160)
    fig.savefig(args.output.with_suffix('.svg'))
    plt.close(fig)


if __name__ == '__main__':
    main()
