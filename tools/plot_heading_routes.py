#!/usr/bin/env python3
"""Plot every available route trial, retaining failures and intervention labels."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = json.loads(args.summary.read_text())
    groups = [
        ('epoch-antenna-active', 'Antena compensada'),
        ('epoch-antenna-control', 'Control sin compensar'),
        ('coherent24', '24° / 1.2'), ('coherent30', '30° / 1.2'),
        ('coherent24-fast', '24° / 1.6–1.8'),
        ('coherent24-holdout', '24° / reservadas 1.6'),
        ('verified300', 'GNSS 300 ms'),
        ('verified300-lagged', '300 ms + EKF temporal'),
        ('speed2-nav', 'Piloto 2.0'),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(16, 7), layout='constrained')
    axes[0].set_xlabel('Máximo global heading en curva (°)')
    axes[1].set_xlabel('RMS posición global en curva (m)')
    axes[2].set_xlabel('Radio físico central mediano (m)')
    labels = []
    for index, (name, label) in enumerate(groups):
        selected = [r for r in rows if r['campaign'] == 'issue328-'+name]
        completed = sum(bool(r.get('curve', {}).get('completed_route')) for r in selected)
        passed = sum(r['functional_pass'] for r in selected)
        labels.append(f'{label}\ncurvas {completed}/{len(selected)}; gates {passed}/{len(selected)}')
        for number, row in enumerate(selected):
            curve = row.get('curve', {})
            pose = curve.get('curve_localization', {}).get('global', {})
            values = [pose.get('heading_deg', {}).get('maximum'),
                      pose.get('position_m', {}).get('rms'),
                      curve.get('central_physical_radius_m', {}).get('median')]
            for ax, value in zip(axes, values):
                if value is not None:
                    y = index + (number-(len(selected)-1)/2)*.035
                    ax.scatter(value, y, marker='o' if row['functional_pass'] else 'x',
                               color='tab:blue' if row['functional_pass'] else 'tab:red', s=24)
    for ax in axes:
        ax.set_yticks(range(len(groups)), labels if ax == axes[0] else [])
        ax.invert_yaxis(); ax.grid(axis='x', alpha=.25)
    fig.suptitle('PC / Fortress: todos los datos disponibles; × = fallo funcional\n'
                 'Sin ajuste de desfase ni alineación espacial; denominadores incluyen setup fallido')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)


if __name__ == '__main__':
    main()
