#!/usr/bin/env python3
"""Headless batch runner with one Sim and in-process DDS Pilot per trial."""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from workbench.research.experiments.batch_readiness import sim_log_ready


def _connect_service(config_path: str, *, runtime_config_path=None):
    from workbench.research.experiments.walking_baseline import _connect_service as connect

    return connect(config_path, runtime_config_path=runtime_config_path)


def _wait_sim_ready(sim_log: Path, *, timeout_s: float, proc: subprocess.Popen | None = None) -> bool:
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            tail = sim_log.read_text(encoding="utf-8", errors="replace")[-12000:]
            raise SystemExit(f"Sim exited with code {proc.returncode}; log: {sim_log.resolve()}\n{tail}")
        if sim_log_ready(sim_log):
            time.sleep(5.0)
            if proc is None or proc.poll() is None:
                return True
            continue
        time.sleep(1.0)
    return False


def _start_sim(config_path: str, run_id: str, log_path: Path, *, runtime_config=None,
               working_directory=None) -> subprocess.Popen:
    env = os.environ.copy()
    env["ELESIM_RUN_ID"] = run_id
    env["ELESIM_WALKING_METRICS"] = "1"
    env["PYTHONUNBUFFERED"] = "1"
    env["PYTHONPATH"] = os.pathsep.join((str(ROOT), str(ROOT / "payload/runtime/common/protocol"),
        str(ROOT / "payload/runtime/docker/sim/app"), str(ROOT / "model"),
        *(p for p in env.get("PYTHONPATH", "").split(os.pathsep) if p and Path(p).is_absolute())))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"[batch] starting sim run_id={run_id!r} -> {log_path}")
    command = [sys.executable, "-m", "elesim_sim.main", "--config", config_path, "--no-viewer"]
    if runtime_config:
        command += ["--runtime-config", str(runtime_config)]
    with log_path.open("x", encoding="utf-8") as fh:
        proc = subprocess.Popen(
            command,
            cwd=str(working_directory or ROOT),
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
    from elesim_pilot.config import load_app_config

    bundle = load_app_config(config_path)
    service.start_perception_capture(config=bundle.perception_config)
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        if service.client is not None:
            host = service.client.refresh_state()
            if host.perceived_center_uv is not None and float(host.perceived_object_confidence) > 0.0:
                u, v = host.perceived_center_uv
                print(f"[batch] perception ready uv=({u:.3f},{v:.3f}) conf={host.perceived_object_confidence:.2f}")
                return True
        time.sleep(0.2)
    print("[batch] warning: perception UV not received before timeout")
    return False



def _require_contact_rows(run_id: str, sim_log: Path, *, log_dir=None) -> None:
    contact = Path(log_dir or ROOT / "logs/walking_baseline") / f"{run_id}_contact.csv"
    if contact.is_file():
        with contact.open(newline="", encoding="utf-8") as stream:
            if next(csv.DictReader(stream), None) is not None:
                return
    raise SystemExit(
        f"No contact samples recorded: {contact}. "
        f"Trial is incomplete; inspect Sim controller/fault log: {sim_log}"
    )


def assess_trial(log_dir: Path, run_id: str, timing: dict, motion: str) -> dict:
    """Evaluate movement in simulation time, not wall time or CSV existence."""
    import math
    path = log_dir / f"{run_id}_walking.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [r for r in csv.DictReader(stream)
                if timing["sim_start_s"] <= float(r["sim_time_s"]) <= timing["sim_end_s"]]
    valid = len(rows) >= 2 and timing["command_sim_s"] > 0
    keys = ("sim_time_s", "base_pos_x", "base_pos_y", "base_pos_z", "base_roll", "base_pitch", "base_yaw", "fall_flag")
    valid = valid and all(math.isfinite(float(r[k])) for r in rows for k in keys)
    valid = valid and (float(rows[0]["sim_time_s"]) <= timing["sim_start_s"]+.5
                       and float(rows[-1]["sim_time_s"]) >= timing["sim_end_s"]-.5)
    fallen = not valid or any(float(r["fall_flag"]) != 0 for r in rows)
    progress = 0.
    if valid:
        axis = "base_yaw" if motion == "turn" else "base_pos_x"
        progress = (float(rows[-1][axis])-float(rows[0][axis])) * (-1 if motion == "backward" else 1)
    speed = .5 if motion == "turn" else .35
    minimum = max(.05, .1*speed*timing["command_sim_s"])
    return {"complete": True, "accepted": bool(valid and not fallen and progress >= minimum),
            "run_id": run_id, "timing": timing, "walking_rows": len(rows),
            "fallen": fallen, "commanded_progress": progress, "minimum_progress": minimum,
            "scope": "DDS walking execution/no fall/progress; gaze improvement requires paired analysis"}


def main(argv=None) -> dict:
    from elesim_pilot.config import load_app_config
    from workbench.research.experiments.walking_baseline import (
        _run_trial, _trial_run_id, _validate_gaze_config,
    )

    ap = argparse.ArgumentParser(description="Batch walking baseline with headless sim + perception")
    ap.add_argument("--config", "--pilot-config", default=str(ROOT / "payload/config/pilot/config.yaml"),
                    help="Pilot application configuration")
    ap.add_argument("--sim-config", default=str(ROOT / "payload/config/sim/config.yaml"),
                    help="Sim application configuration")
    ap.add_argument("--pilot-runtime", type=Path)
    ap.add_argument("--sim-runtime", type=Path)
    ap.add_argument("--working-directory", type=Path)
    ap.add_argument("--result", type=Path)
    ap.add_argument("--run-prefix", default="exp_baseline")
    ap.add_argument("--preset", default="neutral")
    ap.add_argument("--motion", default="forward", choices=["forward", "backward", "turn"])
    ap.add_argument("--gaze", default="uv", choices=("off", "uv", "uv_ff", "pitch_preview"))
    ap.add_argument("--max-duration", "--duration", type=float, default=30.0, dest="max_duration",
                    help="Hard cap on trial wall time (s). --duration is an alias.")
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--trial-start", type=int, default=1, help="1-based first trial index")
    ap.add_argument("--sim-warmup-s", type=float, default=180.0)
    ap.add_argument("--perception-warmup-s", type=float, default=20.0)
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--batch-log-dir", default="logs/walking_baseline/_batch")
    ap.add_argument("--notes", default="headless batch")
    ap.add_argument(
        "--run-id-stem",
        default="",
        help="If set, run ids are {stem}_{trial:03d} (e.g. neutral_forward_preview_pos)",
    )
    args = ap.parse_args(argv)
    import math
    if args.trials < 1 or args.trial_start < 1 or not math.isfinite(args.max_duration) or args.max_duration <= 0:
        ap.error("trials, trial-start and duration must be positive")

    config_path = str(Path(args.config).resolve())
    sim_config_path = str(Path(args.sim_config).resolve())
    bundle = load_app_config(config_path)
    from workbench.research.experiments.walking_scenarios import validate_arm_pose
    validate_arm_pose(args.preset, bundle.mapping_config)
    if str(args.gaze).strip().lower() == "pitch_preview":
        _validate_gaze_config(str(args.gaze), bundle.gaze_stabilizer_config)
    batch_log_dir = Path(args.batch_log_dir).resolve()
    batch_log_dir.mkdir(parents=True, exist_ok=True)
    log_dir = Path(args.log_dir).resolve()
    log_dir.mkdir(parents=True, exist_ok=True)

    trials = int(args.trials)
    trial_start = int(args.trial_start)

    results = []
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
        work_dir = (args.working_directory or batch_log_dir / run_id).resolve()
        work_dir.mkdir(parents=True, exist_ok=True)
        trial_logs = work_dir / "logs/walking_baseline"
        for directory in (trial_logs, log_dir):
            if any((directory / f"{run_id}_{suffix}").exists() for suffix in (
                    "walking.csv", "contact.csv", "meta.json", "sim_meta.json", "camera.csv")):
                raise FileExistsError(f"earlier evidence exists for {run_id}; use a new run ID")
        sim_proc = _start_sim(sim_config_path, run_id, sim_log, runtime_config=args.sim_runtime,
                              working_directory=work_dir)
        service = None
        try:
            if not _wait_sim_ready(sim_log, timeout_s=float(args.sim_warmup_s), proc=sim_proc):
                raise SystemExit(f"sim not ready for {run_id}; see {sim_log}")

            # RunContext writes its own metadata later. Retain Sim's original
            # controller/rate/config record instead of overwriting the only copy.
            from shutil import copyfile
            sim_meta = trial_logs / f"{run_id}_meta.json"
            if sim_meta.is_file():
                copyfile(sim_meta, sim_meta.with_name(f"{run_id}_sim_meta.json"))

            from workbench.research.support.output import in_directory
            with in_directory(work_dir):
                service = _connect_service(config_path, runtime_config_path=args.pilot_runtime)
                if str(args.gaze).strip().lower() != "off":
                    if not _wait_perception(
                        service,
                        config_path,
                        timeout_s=float(args.perception_warmup_s),
                    ):
                        raise RuntimeError("perception did not become ready; trial not executed")
                timing = _run_trial(
                    service=service,
                    run_id=run_id,
                    preset=args.preset,
                    motion=args.motion,
                    vx=(
                        0.35
                        if args.motion == "forward"
                        else -0.35 if args.motion == "backward" else 0.0
                    ),
                    vy=0.0,
                    wz=0.5 if args.motion == "turn" else 0.0,
                    duration=float(args.max_duration),
                    gaze=str(args.gaze),
                    log_dir=str(trial_logs),
                    notes=str(args.notes),
                    strict_run_id=True,
                )
        finally:
            try:
                if service is not None:
                    service.close()
            finally:
                _stop_proc(sim_proc, label="sim")
        # Sim's logger writes relative to its working directory. Collect exact
        # run-owned files into the requested output; never search other runs.
        from shutil import copyfile
        for suffix in ("walking.csv", "contact.csv", "meta.json", "sim_meta.json", "camera.csv"):
            source = trial_logs / f"{run_id}_{suffix}"
            target = log_dir / source.name
            if source.is_file() and source != target:
                if target.exists():
                    raise FileExistsError(f"refusing to overwrite earlier trial evidence: {target}")
                copyfile(source, target)
        _require_contact_rows(run_id, sim_log, log_dir=log_dir)
        results.append(assess_trial(log_dir, run_id, timing, args.motion))
        print(
            f"[batch] trial {trial - trial_start + 1}/{trials} complete: {run_id}"
        )
    print(f"[batch] all {trials} trial(s) finished")
    result = {"complete": True, "accepted": all(r["accepted"] for r in results), "trials": results}
    if args.result:
        args.result.write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == "__main__":
    raise SystemExit(0 if main()["accepted"] else 1)
