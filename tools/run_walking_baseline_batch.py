#!/usr/bin/env python3
"""Headless batch runner: host + sim restart per trial, auto Start Perception, walking baseline."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.config_loader import load_app_config_from_ini
from engine.controller import ControlClient
from engine.gaze_stabilizer.gait_phase_preview import resolve_gait_period_s
from tools.walking_baseline import _connect_service, _run_trial, _trial_run_id, _validate_gaze_config


def _host_reachable(config_path: str) -> bool:
    bundle = load_app_config_from_ini(config_path)
    client = ControlClient(endpoint=bundle.sim_config.host_ctrl_port, cfg=bundle.mapping_config)
    try:
        host = client.refresh_state()
        return bool(host.connected)
    finally:
        client.close()


def _ensure_host(config_path: str, *, log_dir: Path, restart: bool) -> subprocess.Popen | None:
    if restart:
        subprocess.run(["pkill", "-f", "python host.py"], check=False)
        time.sleep(1.0)
    elif _host_reachable(config_path):
        print("[batch] host already reachable")
        return None
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "host.log"
    print(f"[batch] starting host.py -> {log_path}")
    fh = open(log_path, "a", encoding="utf-8")
    proc = subprocess.Popen(
        [sys.executable, "host.py", "--config", config_path],
        cwd=str(ROOT),
        stdout=fh,
        stderr=subprocess.STDOUT,
    )
    deadline = time.time() + 30.0
    while time.time() < deadline:
        if proc.poll() is not None:
            raise SystemExit(f"host.py exited early (code={proc.returncode}); see {log_path}")
        if _host_reachable(config_path):
            print("[batch] host ready")
            return proc
        time.sleep(0.5)
    raise SystemExit("host did not become reachable within 30s")


def _sim_log_text(log_path: Path) -> str:
    if not log_path.is_file():
        return ""
    try:
        return log_path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


def _sim_log_ready(log_path: Path) -> bool:
    text = _sim_log_text(log_path)
    return "[sim_camera] publisher bound" in text


def _wait_sim_ready(sim_log: Path, *, timeout_s: float) -> bool:
    deadline = time.time() + float(timeout_s)
    while time.time() < deadline:
        if _sim_log_ready(sim_log):
            time.sleep(5.0)
            return True
        time.sleep(1.0)
    return False


def _start_sim(config_path: str, run_id: str, log_path: Path) -> subprocess.Popen:
    env = os.environ.copy()
    env["ELESIM_RUN_ID"] = run_id
    env["ELESIM_WALKING_METRICS"] = "1"
    env["PYTHONUNBUFFERED"] = "1"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    fh = open(log_path, "w", encoding="utf-8")
    print(f"[batch] starting sim.py run_id={run_id!r} -> {log_path}")
    proc = subprocess.Popen(
        [sys.executable, "sim.py", "--config", config_path],
        cwd=str(ROOT),
        env=env,
        stdout=fh,
        stderr=subprocess.STDOUT,
    )
    return proc


def _stop_proc(proc: subprocess.Popen | None, *, label: str, grace_s: float = 8.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    print(f"[batch] stopping {label} (pid={proc.pid})")
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=grace_s)
        return
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=3.0)


def _wait_perception(service, config_path: str, *, timeout_s: float) -> bool:
    bundle = load_app_config_from_ini(config_path)
    service.start_perception_capture(config=bundle.perception_config)
    deadline = time.time() + float(timeout_s)
    while time.time() < deadline:
        if service.client is not None:
            host = service.client.refresh_state()
            if host.perceived_center_uv is not None and float(host.perceived_object_confidence) > 0.0:
                u, v = host.perceived_center_uv
                print(f"[batch] perception ready uv=({u:.3f},{v:.3f}) conf={host.perceived_object_confidence:.2f}")
                return True
        time.sleep(0.2)
    print("[batch] warning: perception UV not received before timeout")
    return False


def _preview_trial_kwargs(bundle, gaze: str) -> dict:
    mode = str(gaze).strip().lower()
    if mode == "preview":
        gcfg = bundle.gaze_stabilizer_config
        gait_period_s = resolve_gait_period_s(
            gait_period_s=float(gcfg.gait_period_s),
            gait_hz=float(bundle.go2_locomotion_config.gait_hz),
        )
        return {
            "preview_enable": True,
            "preview_type": "gait_phase",
            "gait_period_s": float(gait_period_s),
            "preview_horizon_s": float(gcfg.gait_preview_horizon_s),
            "gait_template_path": str(gcfg.gait_template_path or ""),
        }
    if mode == "pitch_preview":
        gcfg = bundle.gaze_stabilizer_config
        return {
            "preview_enable": True,
            "preview_type": "pitch_rate",
            "gait_period_s": 0.0,
            "preview_horizon_s": float(gcfg.preview_tau_s),
            "gait_template_path": "",
        }
    return {}


def main() -> None:
    ap = argparse.ArgumentParser(description="Batch walking baseline with headless sim + perception")
    ap.add_argument("--config", default="config.ini")
    ap.add_argument("--run-prefix", default="exp_baseline")
    ap.add_argument("--preset", default="neutral")
    ap.add_argument("--motion", default="forward", choices=["forward", "backward", "turn"])
    ap.add_argument("--gaze", default="uv")
    ap.add_argument("--max-duration", "--duration", type=float, default=30.0, dest="max_duration",
                    help="Hard cap on trial wall time (s). --duration is an alias.")
    ap.add_argument("--stop-at-standoff", type=float, default=0.85,
                    help="Stop when GO2 base-to-object horizontal distance <= this (m). 0 disables.")
    ap.add_argument("--no-eye-video", action="store_true", help="Skip per-trial eye-in-hand MP4 recording")
    ap.add_argument("--pose-settle-s", type=float, default=2.0)
    ap.add_argument("--video-preroll-s", type=float, default=2.0)
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--trial-start", type=int, default=1, help="1-based first trial index")
    ap.add_argument("--sim-warmup-s", type=float, default=180.0)
    ap.add_argument("--perception-warmup-s", type=float, default=20.0)
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--restart-host", action="store_true", default=True)
    ap.add_argument("--no-restart-host", action="store_false", dest="restart_host")
    ap.add_argument("--batch-log-dir", default="logs/walking_baseline/_batch")
    ap.add_argument("--notes", default="headless batch")
    ap.add_argument(
        "--run-id-stem",
        default="",
        help="If set, run ids are {stem}_{trial:03d} (e.g. neutral_forward_gait_preview)",
    )
    args = ap.parse_args()

    config_path = str(args.config)
    bundle = load_app_config_from_ini(config_path)
    if str(args.gaze).strip().lower() in ("preview", "pitch_preview"):
        _validate_gaze_config(
            str(args.gaze),
            bundle.gaze_stabilizer_config,
            gait_hz=float(bundle.go2_locomotion_config.gait_hz),
        )
    batch_log_dir = Path(args.batch_log_dir)
    batch_log_dir.mkdir(parents=True, exist_ok=True)
    host_proc = _ensure_host(config_path, log_dir=batch_log_dir, restart=bool(args.restart_host))

    trials = max(1, int(args.trials))
    trial_start = max(1, int(args.trial_start))
    preview_kwargs = _preview_trial_kwargs(bundle, str(args.gaze))

    try:
        for trial in range(trial_start, trial_start + trials):
            run_id = _trial_run_id(
                args.run_prefix,
                args.preset,
                args.motion,
                trial,
                run_id_stem=str(args.run_id_stem),
            )
            os.environ["ELESIM_RUN_ID"] = run_id
            os.environ["ELESIM_WALKING_METRICS"] = "1"

            sim_log = batch_log_dir / f"{run_id}_sim.log"
            sim_proc = _start_sim(config_path, run_id, sim_log)
            if not _wait_sim_ready(sim_log, timeout_s=float(args.sim_warmup_s)):
                _stop_proc(sim_proc, label="sim")
                raise SystemExit(f"sim not ready for {run_id}; see {sim_log}")

            service = _connect_service(config_path)
            try:
                _wait_perception(service, config_path, timeout_s=float(args.perception_warmup_s))
                _run_trial(
                    service=service,
                    run_id=run_id,
                    preset=args.preset,
                    motion=args.motion,
                    vx=0.35 if args.motion == "forward" else (-0.35 if args.motion == "backward" else 0.0),
                    vy=0.0,
                    wz=0.5 if args.motion == "turn" else 0.0,
                    max_duration=float(args.max_duration),
                    gaze=str(args.gaze),
                    log_dir=args.log_dir,
                    notes=str(args.notes),
                    strict_run_id=True,
                    config_path=config_path,
                    stop_at_standoff=float(args.stop_at_standoff) if float(args.stop_at_standoff) > 0.0 else None,
                    save_eye_video=not bool(args.no_eye_video),
                    pose_settle_s=float(args.pose_settle_s),
                    video_preroll_s=float(args.video_preroll_s),
                    **preview_kwargs,
                )
            finally:
                service.close()

            _stop_proc(sim_proc, label="sim")
            print(f"[batch] trial {trial - trial_start + 1}/{trials} complete: {run_id}")
    finally:
        pass  # keep host running for manual inspection

    print(f"[batch] all {trials} trial(s) finished")


if __name__ == "__main__":
    main()
