#!/usr/bin/env python3
"""Walking baseline runner - control and sim agents must be pre-started."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.config import load_app_config, load_runtime_role_config
from elesim_controller.pick import ControlClient, ControlService, PanelState
from elesim_controller.experiment.run_context import RunContext
from elesim_controller.observability.walking_scenarios import BASELINE_SCENARIOS, ArmPosePreset, arm_pose_as_q


def _parse_gaze(raw: str) -> str:
    mode = str(raw).strip().lower()
    if mode not in ("off", "uv", "uv_ff", "pitch_preview"):
        raise SystemExit(f"unknown --gaze {raw!r} (off|uv|uv_ff|pitch_preview)")
    return mode


def _validate_gaze_config(gaze: str, gaze_cfg) -> None:
    mode = str(gaze).strip().lower()
    if mode == "pitch_preview":
        if not bool(getattr(gaze_cfg, "preview_enable", False)):
            raise SystemExit("pitch_preview requested but gaze_preview_enable=false in config")


def _trial_run_id(
    run_prefix: str,
    preset: str,
    motion: str,
    trial: int,
    *,
    run_id_stem: str = "",
) -> str:
    stem = str(run_id_stem).strip()
    if stem:
        return f"{stem}_{trial:03d}"
    return f"{run_prefix}_{preset}_{motion}_{trial:03d}"


def _connect_service(config_path: str) -> ControlService:
    bundle = load_app_config(config_path)
    client = ControlClient(
        endpoint=load_runtime_role_config(ROOT / "controller/config/runtime.yaml").bind_endpoint,
        cfg=bundle.mapping_config,
    )
    host = client.refresh_state()
    if not host.connected:
        raise SystemExit("target not reachable - start router, simulator, and controller first")
    ownership_enable = False
    if hasattr(bundle, "experiment_config"):
        ownership_enable = bool(getattr(bundle.experiment_config, "ownership_enable", False))
    service = ControlService(
        PanelState(),
        client=client,
        mapping_cfg=bundle.mapping_config,
        ik_cfg=bundle.ik_config,
        config_path=config_path,
        perception_cfg=bundle.perception_config,
        pick_cfg=bundle.pick_config,
        gaze_cfg=bundle.gaze_stabilizer_config,
        ownership_enable=ownership_enable,
    )
    return service


def _apply_preset(service: ControlService, preset: ArmPosePreset | str) -> None:
    q = arm_pose_as_q(preset)
    service.state.set_q(*q)
    service.send_current_target(source="experiment", force=True)


def _run_trial(
    *,
    service: ControlService,
    run_id: str,
    preset: str,
    motion: str,
    vx: float,
    vy: float,
    wz: float,
    duration: float,
    gaze: str,
    log_dir: str,
    notes: str,
    strict_run_id: bool,
) -> None:
    ctx = RunContext.from_cli(
        run_id=run_id,
        arm_preset=preset,
        go2_motion=motion,
        gaze_mode=gaze,
        notes=notes,
    )
    if strict_run_id:
        ctx.validate_env_run_id(strict=True)
    else:
        ctx.validate_env_run_id(strict=False)
    ctx.write_meta(log_dir)

    _apply_preset(service, preset)

    if gaze == "off":
        service.stop_gaze_stabilizer()
    elif motion in ("forward", "backward"):
        service.start_gaze_stabilizer_walking(run_id=run_id, gaze_mode=gaze)
    else:
        service.start_gaze_stabilizer_standing(run_id=run_id)

    t_end = time.time() + float(duration)
    while time.time() < t_end:
        service.send_go2_velocity(vx=float(vx), vy=float(vy), wz=float(wz))
        time.sleep(0.05)

    service.stop_gaze_stabilizer()
    service.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
    print(f"[baseline] done run_id={run_id}")
    print(f"  walking: {Path(log_dir) / f'{run_id}_walking.csv'}")
    print(f"  camera:  {Path(log_dir) / f'{run_id}_camera.csv'}")
    print(f"  meta:    {Path(log_dir) / f'{run_id}_meta.json'}")


def main() -> None:
    ap = argparse.ArgumentParser(description="GO2 walking baseline (ControlService only)")
    ap.add_argument("--config", default="controller/config/default.yaml")
    ap.add_argument("--run-id", default="", help="must match ELESIM_RUN_ID on sim process")
    ap.add_argument("--run-prefix", default="exp_baseline")
    ap.add_argument("--preset", default="neutral")
    ap.add_argument("--motion", default="forward", choices=["forward", "backward", "turn"])
    ap.add_argument("--vx", type=float, default=None)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--wz", type=float, default=None)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--gaze", default="off", type=_parse_gaze)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--trial-index", type=int, default=0, help="1-based trial index for --repeat workflow")
    ap.add_argument("--scenario", type=int, default=-1, help="index into BASELINE_SCENARIOS")
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--notes", default="")
    ap.add_argument("--headless-hint", action="store_true")
    ap.add_argument("--strict-run-id", action="store_true", default=True)
    ap.add_argument("--print-trim-sweep-checklist", action="store_true")
    args = ap.parse_args()

    if args.headless_hint:
        print("[hint] set enable_viewer=false in the simulator config for headless sim")

    if args.print_trim_sweep_checklist:
        print("Pitch-trim sweep (sim restart required per case):")
        print("  1. Edit mpc_pitch_trim_* in the simulator config")
        print("  2. export ELESIM_RUN_ID=exp_trim_<case>_001")
        print("  3. restart elesim-simulator with ELESIM_WALKING_METRICS=1")
        print("  4. run walking_baseline --preset bent_upward --motion backward")
        print("  5. analyze_walking_metrics.py --compare baseline vs trim case")
        return

    service = _connect_service(args.config)
    _validate_gaze_config(args.gaze, service._gaze_cfg)

    if args.scenario >= 0:
        preset, motion, vel, turn = BASELINE_SCENARIOS[args.scenario]
        vx = float(vel[0]) if args.vx is None else float(args.vx)
        vy = float(vel[1]) if args.vy is None else float(args.vy)
        wz = float(vel[2]) if args.wz is None else float(args.wz)
        run_id = args.run_id or _trial_run_id(args.run_prefix, preset.value, motion, 1)
        _run_trial(
            service=service,
            run_id=run_id,
            preset=preset.value,
            motion=motion,
            vx=vx,
            vy=vy,
            wz=wz,
            duration=args.duration,
            gaze=args.gaze,
            log_dir=args.log_dir,
            notes=args.notes or f"scenario={args.scenario} turn={turn}",
            strict_run_id=bool(args.strict_run_id),
        )
        service.close()
        return

    vx = 0.35 if args.vx is None and args.motion == "forward" else (-0.35 if args.vx is None and args.motion == "backward" else float(args.vx or 0.0))
    vy = float(args.vy)
    wz = 0.5 if args.wz is None and args.motion == "turn" else float(args.wz or 0.0)

    repeat = max(1, int(args.repeat))
    for trial in range(1, repeat + 1):
        if args.trial_index > 0 and trial != int(args.trial_index):
            continue
        run_id = args.run_id or _trial_run_id(args.run_prefix, args.preset, args.motion, trial)
        env_rid = os.environ.get("ELESIM_RUN_ID", "").strip()
        if env_rid and env_rid != run_id:
            print(
                f"[baseline] trial {trial}/{repeat}: set ELESIM_RUN_ID={run_id!r} and restart elesim-simulator, then re-run with "
                f"--run-id {run_id!r} --trial-index {trial}"
            )
            if args.strict_run_id:
                raise SystemExit(1)
        _run_trial(
            service=service,
            run_id=run_id,
            preset=args.preset,
            motion=args.motion,
            vx=vx,
            vy=vy,
            wz=wz,
            duration=args.duration,
            gaze=args.gaze,
            log_dir=args.log_dir,
            notes=args.notes,
            strict_run_id=bool(args.strict_run_id),
        )
        if trial < repeat:
            print(f"[baseline] completed trial {trial}/{repeat}; restart sim with next ELESIM_RUN_ID before continuing")
            break

    service.close()


if __name__ == "__main__":
    main()
