"""Prepare a small two-band train/validation/test pilot from a v3 Hugmap.

Select one fully sampled local triangle per roll at a fixed diameter. Three
anchors cover only that triangle, never the entire band. No physics claim is
made from a static solved grid or a component label.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def prepare(source, diameter_mm=150.):
    path = Path(source)
    raw = path.read_bytes()
    table = json.loads(raw)
    meta = table['metadata']
    if meta.get('schema') != 'hugmap-table' or meta.get('schema_version') != 3:
        raise ValueError('Require Hugmap v3 with independently solved +/-90 roll branches')
    if meta.get('object', {}).get('shape') != 'circle':
        raise ValueError('This experiment requires a circle map')
    maps = [m for m in table['maps'] if float(m['diameter_mm']) == float(diameter_mm)]
    if len(maps) != 1 or maps[0]['status'] != 'complete':
        raise ValueError('Need one complete map at the requested diameter')
    dx, dy = (float(meta['config'][k]) for k in ('dx_mm', 'dy_mm'))
    if not all(math.isfinite(v) and v > 0 for v in (dx, dy, diameter_mm)):
        raise ValueError('Invalid grid or diameter')
    digest = hashlib.sha256(raw).hexdigest()
    cases, triangles = [], []
    for band, roll in ((1, -90), (2, 90)):
        points = {}
        for row in maps[0]['results']:
            if row['status'] != 'found':
                continue
            for solution in row.get('roll_solutions', []):
                if solution['roll_deg'] != roll:
                    continue
                values = [float(solution[k]) for k in ('seg1_deg', 'seg2_deg', 'mu_min')]
                if (not all(math.isfinite(v) for v in values) or abs(values[0]) > 180
                        or not -180 <= values[1] <= 0 or values[2] < 0):
                    raise ValueError('Invalid physical solution')
                xy = (round(float(row['x_mm']), 6), round(float(row['y_mm']), 6))
                if xy in points:
                    raise ValueError('Duplicate position/roll solution')
                points[xy] = values
        if not points:
            raise ValueError(f'No solutions for band {band}')
        median = np.median(np.array(list(points)), axis=0)
        # Local 4-cell triangle: all 15 grid samples must have a solution.
        # Requiring samples does not certify the continuous interior.
        offsets = [(i, j) for i in range(5) for j in range(5-i)]
        selected = None
        for x, y in sorted(points, key=lambda p: ((p[0]-median[0])**2+(p[1]-median[1])**2, p)):
            patch = {(i, j): (round(x+i*dx, 6), round(y+j*dy, 6)) for i, j in offsets}
            if all(p in points for p in patch.values()):
                selected = patch
                break
        if selected is None:
            raise ValueError(f'Band {band} has no fully sampled local 4-cell triangle; do not bridge its gaps')
        branch = f'band-{band}-local-{digest[:12]}'
        anchors = {(0, 0), (4, 0), (0, 4)}
        test = {(2, 1)}  # held out before training; do not use for anchor adaptation
        ids = []
        for offset, xy in selected.items():
            q1, q2, mu = points[xy]
            cid = f'b{band}-x{xy[0]:g}-y{xy[1]:g}-d{diameter_mm:g}'
            split = 'train' if offset in anchors else 'test' if offset in test else 'validation'
            start = [0., math.radians(roll), 0., 0.]
            goal = [0., math.radians(roll), math.radians(q1)/5, math.radians(q2)/5]
            cases.append(dict(case_id=cid, split=split, band=band, roll_deg=roll,
                              diameter_mm=diameter_mm, xy_mm=list(xy), branch=branch,
                              start_command=start, goal_command=goal, mu_min=mu))
            if split == 'train':
                ids.append(cid)
        triangles.append(dict(branch=branch, anchor_ids=ids, continuous_interior_verified=False))
    return dict(schema='elesim-hug-pilot', schema_version=1, source_sha256=digest,
                source_coordinates=meta.get('coordinates'), cases=cases, triangles=triangles,
                simulation_context=None, readiness='needs-world-frame-and-physics-calibration')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--diameter-mm', type=float, default=150.)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = prepare(args.source, args.diameter_mm)
    body = json.dumps(result, allow_nan=False, indent=2)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        stream.write(body+'\n')
    print(f'Wrote {len(result["cases"])} cases (six training anchors); physics calibration is required.')


if __name__ == '__main__':
    main()
