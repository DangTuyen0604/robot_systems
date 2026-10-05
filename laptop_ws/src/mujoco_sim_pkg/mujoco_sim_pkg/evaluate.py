#!/usr/bin/env python3
"""
Run the MuJoCo scenario N times and report lane-keeping and sign metrics.

    ros2 run mujoco_sim_pkg evaluate --runs 5 --output docs/results/mujoco_evaluation
"""

import argparse
from datetime import date
import json
import math
import platform
import statistics

import mujoco

from mujoco_sim_pkg.evaluation import evaluate, MIN_STOP_TIME
from mujoco_sim_pkg.scenario import run_scenario

COLUMNS = [
    ('finish_time_s', 'Finish time (s)', '{:.1f}'),
    ('stem_mean_error_mm', 'Stem mean err (mm)', '{:.1f}'),
    ('stem_max_error_mm', 'Stem max err (mm)', '{:.1f}'),
    ('branch_mean_error_mm', 'Branch mean err (mm)', '{:.1f}'),
    ('branch_max_error_mm', 'Branch max err (mm)', '{:.1f}'),
    ('kept_lane', 'Kept lane', '{}'),
    ('took_left_branch', 'Left branch', '{}'),
    ('completed', 'Completed', '{}'),
    ('slow_hold_s', 'Slow at 20 sign (s)', '{:.1f}'),
    ('stop_hold_s', 'Stopped at STOP (s)', '{:.1f}'),
]


def cpu_name():
    try:
        with open('/proc/cpuinfo') as cpuinfo:
            for line in cpuinfo:
                if line.startswith('model name'):
                    return line.split(':', 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or 'unknown'


def summary_row(results, key, fmt):
    values = [r[key] for r in results]
    if isinstance(values[0], bool):
        return f'{sum(values)}/{len(values)}'
    finite = [v for v in values if not math.isnan(v)]
    if not finite:
        return '-'
    mean = statistics.mean(finite)
    spread = statistics.pstdev(finite) if len(finite) > 1 else 0.0
    return f'{fmt.format(mean)} ± {fmt.format(spread)}'


def to_markdown(results, args):
    lines = [
        '# MuJoCo scenario evaluation',
        '',
        f'- Date: {date.today().isoformat()}',
        f'- Runs: {len(results)}, each until the robot stops at the end of the route, '
        f'sign detection {"on (YOLOv8 ONNX, 320 px)" if args.signs else "off"}',
        f'- Machine: {cpu_name()}, MuJoCo {mujoco.__version__}, real-time factor 1.0',
        '- Finish time: from launch until the robot is 2.9 m up the left branch.',
        '- Lane error: distance of base_link from the lane centre on the straight '
        'stem (x 0-1.0 m) and left branch (y 0.7-2.95 m), from ground-truth /odom.',
        f'- Kept lane: error always below 100 mm. Stop counted from {MIN_STOP_TIME:.0f} s.',
        '',
        '| Run | ' + ' | '.join(title for _, title, _ in COLUMNS) + ' |',
        '|---|' + '---|' * len(COLUMNS),
    ]
    for i, r in enumerate(results, 1):
        cells = [fmt.format(r[key]) for key, _, fmt in COLUMNS]
        lines.append(f'| {i} | ' + ' | '.join(cells) + ' |')
    lines.append('| **All** | ' + ' | '.join(
        summary_row(results, key, fmt) for key, _, fmt in COLUMNS) + ' |')
    return '\n'.join(lines) + '\n'


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--max-duration', dest='duration', type=float, default=150.0,
                        help='upper bound per run; a run ends when the robot stops at the end')
    parser.add_argument('--no-signs', dest='signs', action='store_false',
                        help='run without the YOLO sign node (lane following only)')
    parser.add_argument('--output', help='write <output>.md and <output>.json')
    args = parser.parse_args(args)

    results = []
    for i in range(args.runs):
        print(f'Run {i + 1}/{args.runs}...', flush=True)
        metrics = evaluate(run_scenario(args.duration, signs=args.signs)).as_dict()
        print('  ' + ', '.join(f'{k}={v:.1f}' if isinstance(v, float) else f'{k}={v}'
                               for k, v in metrics.items()), flush=True)
        results.append(metrics)

    report = to_markdown(results, args)
    print(report)
    if args.output:
        with open(args.output + '.md', 'w') as md:
            md.write(report)
        with open(args.output + '.json', 'w') as js:
            json.dump({'signs': args.signs, 'duration': args.duration, 'runs': results},
                      js, indent=2)


if __name__ == '__main__':
    main()
