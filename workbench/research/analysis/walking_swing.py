"""Measure swing tracking in production walking JSONL from the debug probe.

Use steady samples two seconds after each command start, free swing progress
0.05..0.95, and exact latched starts/depths. A descending airborne sample is
stalled when desired Vz < -0.1 m/s, foot Z > 0.03 m and actual Vz >= -0.03 m/s.
These sample statistics are separate from completed walking and stop gates.
The original all-descent metric is retained. ``late_descent`` also reports
progress >=0.70, separating a brief apex turnaround from a foot held in air
late in the commanded descent.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def summarize_swing_tracking(rows):
    from elesim_sim.robot.go2.pympc_controller import swing_foot_reference

    rows = list(rows)
    cfg = next(r for r in rows if r.get('kind') == 'configuration')
    duration = (1. - cfg['gait_duty']) / cfg['gait_hz']
    cycle_time = cfg['duration']+cfg.get('stop', 0.)
    observations = []
    for row in rows:
        if (row.get('kind') != 'sample' or row.get('pose_stage') != 'walk'
                or 'centroidal_control' not in row):
            continue
        elapsed = row['t']-cfg['settle']
        if (not 0. <= elapsed < cfg.get('cycles', 1)*cycle_time
                or not 2. <= elapsed % cycle_time < cfg['duration']):
            continue
        control, gate, legs = row['centroidal_control'], row['contact_gate'], row['leg_control']
        if 'swing_starts' not in legs:
            raise ValueError('exact latched swing starts were not recorded')
        for leg in range(4):
            progress = gate['swing_progress'][leg]
            if (control['contacts'][leg] != 0 or gate['search'][leg]
                    or not .05 <= progress <= .95):
                continue
            target, desired = swing_foot_reference(
                progress, legs['swing_starts'][leg], legs['touchdowns'][leg],
                duration_s=duration, touchdown_depth_m=legs['touchdown_depths'][leg],
            )
            actual = np.asarray(control['foot_velocity_world'][leg])
            foot = np.asarray(control['feet_world'][leg])
            observations.append((progress, target-foot, desired, actual, foot[2]))
    result = {}
    for name, selected in (
        ('all', observations),
        ('accelerating_half', [r for r in observations if r[0] < .5]),
        ('decelerating_half', [r for r in observations if r[0] >= .5]),
        ('late_descent', [r for r in observations if r[0] >= .7]),
    ):
        if not selected:
            result[name] = dict(n=0)
            continue
        desired, actual = np.array([r[2] for r in selected]), np.array([r[3] for r in selected])
        downward = [r for r in selected if r[2][2] < -.1 and r[4] > .03]
        result[name] = dict(
            n=len(selected),
            position_error_p50_p95_mm=(1000*np.percentile(
                np.linalg.norm([r[1] for r in selected], axis=1), [50, 95])).tolist(),
            velocity_projection_ratio=float(np.sum(actual*desired)/max(np.sum(desired**2), 1e-12)),
            velocity_error_rms_mps=float(np.sqrt(np.mean(np.sum((actual-desired)**2, axis=1)))),
            descending_airborne_samples=len(downward),
            descending_airborne_stalled_fraction=(float(np.mean([
                r[3][2] >= -.03 for r in downward])) if downward else None),
        )
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args(argv)
    report = []
    for path in sorted(args.directory.glob('*.jsonl')):
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        report.append(dict(name=path.stem, **summarize_swing_tracking(rows)))
    (args.directory/'swing-summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
