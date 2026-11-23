#!/usr/bin/env python3
"""Plot recorded Gazebo ground truth and signed motor commands; no vehicle access."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_csv(path):
    with path.open() as stream:
        return [[float(v) for v in row] for row in list(csv.reader(stream))[1:]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases', nargs='+', type=Path)
    parser.add_argument('--completion-angle', type=float, help='Override completion angle for historical records')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(len(args.cases), 2, figsize=(12, 3*len(args.cases)), squeeze=False)
    results = []
    for directory, (left, right) in zip(args.cases, axes):
        result = json.loads((directory/'result.json').read_text())
        results.append(result)
        truth = read_csv(directory/'ground_truth.csv')
        motors = read_csv(directory/'motor_commands.csv')
        trigger = result.get('trigger_time', 0)
        left.plot([r[0]-trigger for r in truth], [r[1] for r in truth], label='Gazebo actual tilt')
        completion = args.completion_angle or result.get('completion_degrees', 60)
        left.axhline(60, color='red', linestyle='--', label='60 deg first coast')
        if completion < 60:
            left.axhline(completion, color='green', linestyle=':', label=f'{completion:g} deg stable completion')
        left.set(ylabel='Tilt from upright (deg)', ylim=(-5, 185), title=f"{result['case']}: {'PASS' if result['passed'] else 'FAIL'}")
        for i in range(4):
            right.plot([r[0]-trigger for r in motors], [r[i+1] for r in motors], label=f'Motor {i+1}')
        right.set(ylabel='Signed motor command (rad/s)', title=f"First pair (zero based): {result.get('first_pair')}")
        for axis in (left, right):
            axis.grid(alpha=.25)
            axis.set_xlabel('Time after RC mode switch (s)')
            axis.legend(fontsize=8, loc='best')
    fig.tight_layout()
    fig.savefig(args.output/'recovery.png', dpi=160)
    (args.output/'summary.json').write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
