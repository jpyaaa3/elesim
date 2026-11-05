"""Offline interpolation. Training and replay remain explicit experiments."""
import argparse
import json
from pathlib import Path
from elesim_sim.rl.trajectory import Trajectory
from .interpolate import interpolate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--anchors', nargs=3, required=True, type=Path)
    parser.add_argument('--target', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--samples', default=201, type=int)
    args = parser.parse_args()
    result = interpolate([Trajectory.load(p) for p in args.anchors],
                         json.loads(args.target.read_text()), samples=args.samples)
    result.save(args.output)
    print(f'Wrote candidate {args.output}; physics replay is still required.')


if __name__ == '__main__':
    main()
