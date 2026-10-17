"""Check body progress separately from swing tracking and survival.

For straight XY commands, require >=50% speed tracking, pitch RMS <=6 deg,
roll RMS <=4 deg, body height in 0.24..0.35 m and every recorded stop complete.
These are candidate screening thresholds, not full locomotion acceptance.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics


def summarize_motion(rows):
    rows = list(rows)
    cfg = next(row for row in rows if row.get('kind') == 'configuration')
    command = cfg['command']
    requested = math.hypot(*command[:2])
    if requested <= 1e-9 or abs(command[2]) > 1e-9:
        raise ValueError('motion screening requires a straight XY command')
    cycle_time = cfg['duration'] + cfg['stop']
    steady = [row for row in rows if row.get('kind') == 'sample'
              and row.get('pose_stage') == 'walk'
              and row['t'] >= cfg['settle']
              and 2. <= (row['t']-cfg['settle']) % cycle_time < cfg['duration']]
    if not steady:
        raise ValueError('no steady commanded walking samples')
    along, across = [], []
    for row in steady:
        velocity = row.get('centroidal_control', {}).get('base_velocity_world')
        if velocity is None or len(velocity) != 3 or not all(map(math.isfinite, velocity)):
            raise ValueError('missing finite measured base velocity')
        yaw = row['rpy'][2]
        x = (math.cos(yaw)*command[0]-math.sin(yaw)*command[1])/requested
        y = (math.sin(yaw)*command[0]+math.cos(yaw)*command[1])/requested
        along.append(x*velocity[0]+y*velocity[1])
        across.append(-y*velocity[0]+x*velocity[1])
    speed = statistics.mean(along)
    rms = [math.degrees(math.sqrt(statistics.mean(row['rpy'][i]**2 for row in steady)))
           for i in (0, 1)]
    heights = [row['position'][2] for row in steady]
    summaries = [row for row in rows if row.get('kind') == 'summary']
    summary = summaries[-1] if summaries else {}
    stops = summary.get('cycle_stops', [])
    complete = bool(summary.get('completed') and len(stops) == cfg.get('cycles', 1)
                    and all(stop['completed'] for stop in stops)
                    and summary.get('last', {}).get('pose_stage') == 'stand'
                    and not any(row.get('fallen') or row.get('faulted') for row in steady))
    failures = []
    if not complete:
        failures.append('completion_or_stop')
    if speed/requested < .5:
        failures.append('speed_tracking')
    if rms[0] > 4.:
        failures.append('roll')
    if rms[1] > 6.:
        failures.append('pitch')
    if min(heights) < .24 or max(heights) > .35:
        failures.append('height')
    return dict(passed=not failures, failures=failures, samples=len(steady),
                requested_speed_mps=requested, mean_along_speed_mps=speed,
                speed_ratio=speed/requested,
                across_speed_rms_mps=math.sqrt(statistics.mean(v*v for v in across)),
                roll_pitch_rms_deg=rms,
                mean_signed_pitch_deg=math.degrees(statistics.mean(row['rpy'][1] for row in steady)),
                body_height_min_mean_max_m=[min(heights), statistics.mean(heights), max(heights)])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args(argv)
    result = []
    for path in sorted(args.directory.glob('*.jsonl')):
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        result.append(dict(name=path.stem, **summarize_motion(rows)))
    (args.directory/'motion-summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
