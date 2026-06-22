#!/usr/bin/env python3
"""Demo helpers for walking gaze integration (run with host+sim+ctrl)."""

from __future__ import annotations

import argparse
import os
import time


def main() -> None:
    ap = argparse.ArgumentParser(description="Walking gaze demo workflow notes")
    ap.add_argument(
        "demo",
        choices=("demo1-standing", "demo2-off", "demo2-on", "demo3-approach", "demo4-stop-grasp"),
    )
    ap.add_argument("--duration-s", type=float, default=8.0)
    args = ap.parse_args()

    os.environ["ELESIM_WALKING_METRICS"] = "1"
    run_id = f"{args.demo}_{int(time.time())}"
    print(f"Demo: {args.demo} | run_id={run_id} | duration={args.duration_s}s")
    print("Use ctrl UI GO2 panel buttons or ControlService API:")
    if args.demo == "demo1-standing":
        print("  1) GO2 Stop  2) start_gaze_stabilizer_standing()  3) perception ON")
    elif args.demo == "demo2-off":
        print("  gaze_enable_base_ff=false in config.ini; walk forward; gaze OFF")
    elif args.demo == "demo2-on":
        print("  gaze_enable_base_ff=true; start_gaze_stabilizer_walking(); compare via analyze_walking_metrics.py --compare")
    elif args.demo == "demo3-approach":
        print("  walking + gaze ON, no grasp")
    else:
        print("  start_demo4_stop_and_grasp() — walk, stop, gaze stop, LJI e2e")
    print(f"Analyze: python tools/analyze_walking_metrics.py {run_id} --log-dir logs/walking_baseline")


if __name__ == "__main__":
    main()
