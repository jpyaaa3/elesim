#!/usr/bin/env python3
"""Headless walking baseline runner (requires Genesis + convex_mpc)."""

from __future__ import annotations

import argparse
import os
import time

from engine.profile.walking_scenarios import BASELINE_SCENARIOS, ArmPosePreset, arm_pose_as_q
from engine.profile.walking_metrics import WalkingMetricsLogger, WalkingMetricsMeta


def main() -> None:
    ap = argparse.ArgumentParser(description="Run GO2 walking baseline scenarios")
    ap.add_argument("--duration-s", type=float, default=5.0)
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--scenario", type=int, default=-1, help="index into BASELINE_SCENARIOS, -1=all")
    args = ap.parse_args()

    os.environ["ELESIM_WALKING_METRICS"] = "1"
    print("Baseline scenarios:")
    for i, (preset, motion, vel, turn) in enumerate(BASELINE_SCENARIOS):
        print(f"  [{i}] {preset.value} + {motion} vel={vel} turn={turn}")
    print("Launch full sim (host+ctrl) for integrated runs; this script documents scenario metadata.")

    indices = [args.scenario] if args.scenario >= 0 else list(range(len(BASELINE_SCENARIOS)))
    for i in indices:
        preset, motion, vel, turn = BASELINE_SCENARIOS[i]
        run_id = f"{preset.value}_{motion}_{int(time.time())}"
        meta = WalkingMetricsMeta(
            run_id=run_id,
            arm_preset=preset.value,
            turn_direction=turn,
            command_source=f"scripted_{motion}",
            extra={"vel": vel, "duration_s": args.duration_s},
        )
        logger = WalkingMetricsLogger(run_id=run_id, log_dir=args.log_dir, meta=meta)
        q = arm_pose_as_q(preset)
        meta.extra["arm_q"] = q
        logger._write_meta()
        logger.close()
        print(f"Wrote meta for scenario {i}: {run_id}")


if __name__ == "__main__":
    main()
