#!/usr/bin/env python3
"""Dynamic gaze walking demos (stabilization only — no grasp/LJI)."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.config import load_app_config
from elesim_controller.pick import ControlClient, ControlService, PanelState
from elesim_controller.experiment.run_context import RunContext
from misc.research.experiments.walking_baseline import (
    _apply_preset,
    _connect_service,
    _trial_run_id,
    _validate_gaze_config,
)


def _run_analyzer(run_ids: list[str], log_dir: str) -> None:
    script = ROOT / "misc" / "tooling" / "analysis" / "analyze_walking_metrics.py"
    subprocess.run(
        [sys.executable, str(script), "--log-dir", log_dir, "--compare", *run_ids],
        check=False,
    )


def demo_standing_gaze(args: argparse.Namespace, service: ControlService) -> None:
    run_id = args.run_id or _trial_run_id(args.run_prefix, "standing", "idle", 1)
    ctx = RunContext.from_cli(run_id=run_id, arm_preset="neutral", go2_motion="standing", gaze_mode="uv")
    ctx.validate_env_run_id(strict=args.strict_run_id)
    ctx.write_meta(args.log_dir)
    service.start_gaze_stabilizer_standing(run_id=run_id)
    time.sleep(float(args.duration))
    service.stop_gaze_stabilizer()
    print(f"[demo] standing_gaze complete run_id={run_id}")


def demo_walking_compare(args: argparse.Namespace, service: ControlService) -> None:
    modes = ["off", "uv", "uv_ff"]
    run_ids: list[str] = []
    for i, gaze in enumerate(modes, start=1):
        run_id = args.run_id or _trial_run_id(args.run_prefix, args.preset, args.motion, i)
        env = os.environ.get("ELESIM_RUN_ID", "").strip()
        if env and env != run_id and args.strict_run_id:
            print(f"Set ELESIM_RUN_ID={run_id!r}, restart sim, then --run-id {run_id}")
            return
        ctx = RunContext.from_cli(
            run_id=run_id,
            arm_preset=args.preset,
            go2_motion=args.motion,
            gaze_mode=gaze,
            notes=f"walking_compare:{gaze}",
        )
        ctx.validate_env_run_id(strict=args.strict_run_id)
        ctx.write_meta(args.log_dir)
        if gaze == "off":
            _apply_preset(service, args.preset)
        if gaze != "off":
            service.start_gaze_stabilizer_walking(run_id=run_id, gaze_mode=gaze)
        t_end = time.time() + float(args.duration)
        vx = float(args.vx)
        while time.time() < t_end:
            service.send_go2_velocity(vx=vx, vy=0.0, wz=0.0)
            time.sleep(0.05)
        service.stop_gaze_stabilizer()
        service.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
        run_ids.append(run_id)
        if i < len(modes):
            print(f"[demo] restart sim with next run_id before continuing ({modes[i]})")
            break
    if len(run_ids) >= 2:
        _run_analyzer(run_ids, args.log_dir)


def demo_walking_approach_no_grasp(args: argparse.Namespace, service: ControlService) -> None:
    run_id = args.run_id or _trial_run_id(args.run_prefix, args.preset, "approach", 1)
    ctx = RunContext.from_cli(run_id=run_id, arm_preset=args.preset, go2_motion="approach", gaze_mode="uv_ff")
    ctx.validate_env_run_id(strict=args.strict_run_id)
    ctx.write_meta(args.log_dir)
    service.start_gaze_stabilizer_walking(run_id=run_id, gaze_mode="uv_ff")
    t_end = time.time() + float(args.duration)
    while time.time() < t_end:
        service.send_go2_velocity(vx=float(args.vx), vy=0.0, wz=0.0)
        time.sleep(0.05)
    service.stop_gaze_stabilizer()
    service.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
    print(f"[demo] walking_approach_no_grasp complete (no grasp) run_id={run_id}")


def demo_dynamic_gaze_stabilization(args: argparse.Namespace, service: ControlService) -> None:
    demo_walking_compare(args, service)


DEMOS = {
    "standing_gaze": demo_standing_gaze,
    "walking_compare": demo_walking_compare,
    "walking_approach_no_grasp": demo_walking_approach_no_grasp,
    "dynamic_gaze_stabilization": demo_dynamic_gaze_stabilization,
}


def main() -> None:
    ap = argparse.ArgumentParser(description="Gaze walking experiment demos (no grasp)")
    ap.add_argument("--demo", required=True, choices=sorted(DEMOS.keys()))
    ap.add_argument("--config", default="controller/config/default.yaml")
    ap.add_argument("--run-id", default="")
    ap.add_argument("--run-prefix", default="exp_demo")
    ap.add_argument("--preset", default="bent_upward")
    ap.add_argument("--motion", default="backward")
    ap.add_argument("--vx", type=float, default=-0.2)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--gaze", default="", help="optional override for single-mode demos (pitch_preview|uv|...)")
    args = ap.parse_args()

    bundle = load_app_config(args.config)
    if str(args.gaze).strip().lower() == "pitch_preview":
        _validate_gaze_config("pitch_preview", bundle.gaze_stabilizer_config)
    print(
        json.dumps(
            {
                "demo": args.demo,
                "gaze_config": {
                    "enable_base_ff": bundle.gaze_stabilizer_config.enable_base_ff,
                    "uv_gain": bundle.gaze_stabilizer_config.uv_gain,
                },
            },
            indent=2,
        )
    )
    service = _connect_service(args.config)
    DEMOS[args.demo](args, service)
    service.close()


if __name__ == "__main__":
    main()
