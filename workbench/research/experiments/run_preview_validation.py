#!/usr/bin/env python3
"""Run preview pos/neg validation trials (neutral + forward)."""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_pilot.config import load_app_config
from workbench.research.experiments import run_walking_baseline_batch as batch
from workbench.research.experiments.config_overlay import write_config_overlay
from workbench.research.experiments.walking_baseline import _run_trial, _validate_gaze_config


def _run_group(
    *,
    config_path: str,
    sign_label: str,
    run_stem: str,
    trials: int,
    duration: float,
    vx: float,
    sim_warmup_s: float,
    perception_warmup_s: float,
    log_dir: Path,
    batch_log_dir: Path,
) -> None:
    host_proc = batch._ensure_host(config_path, log_dir=batch_log_dir, restart=False)
    service = batch._connect_service(config_path)
    try:
        for trial in range(1, trials + 1):
            run_id = f"{run_stem}_{trial:03d}"
            os.environ["ELESIM_RUN_ID"] = run_id
            os.environ["ELESIM_WALKING_METRICS"] = "1"
            sim_log = batch_log_dir / f"{run_id}_sim.log"
            sim_proc = batch._start_sim(config_path, run_id, sim_log)
            try:
                if not batch._wait_sim_ready(sim_log, timeout_s=sim_warmup_s):
                    raise SystemExit(f"sim not ready for {run_id}; see {sim_log}")
                batch._wait_perception(service, config_path, timeout_s=perception_warmup_s)
                _run_trial(
                    service=service,
                    run_id=run_id,
                    preset="neutral",
                    motion="forward",
                    vx=float(vx),
                    vy=0.0,
                    wz=0.0,
                    duration=float(duration),
                    gaze="pitch_preview",
                    log_dir=str(log_dir),
                    notes=f"pitch_preview validation {sign_label}",
                    strict_run_id=True,
                )
            finally:
                batch._stop_proc(sim_proc, label="sim")
    finally:
        service.close()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="payload/config/pilot/config.yaml")
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--batch-log-dir", default="logs/walking_baseline/_preview_validation")
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--vx", type=float, default=0.35)
    ap.add_argument("--sim-warmup-s", type=float, default=120.0)
    ap.add_argument("--perception-warmup-s", type=float, default=20.0)
    ap.add_argument("--sign", choices=("pos", "neg", "both"), default="both")
    args = ap.parse_args()

    bundle_log = Path(args.batch_log_dir)
    bundle_log.mkdir(parents=True, exist_ok=True)
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    overlay_dir = Path(tempfile.gettempdir()) / "elesim_preview_validation"

    def sign_config(label: str, b_pitch: float) -> str:
        path = write_config_overlay(
            args.config,
            overlay_dir / f"preview_{label}.yaml",
            {
                "behaviors.gaze.preview.enable": True,
                "behaviors.gaze.preview.b_pitch": b_pitch,
            },
        )
        bundle = load_app_config(str(path))
        _validate_gaze_config("pitch_preview", bundle.gaze_stabilizer_config)
        return str(path)

    if args.sign in ("pos", "both"):
        config_path = sign_config("pos", 0.05)
        _run_group(
            config_path=config_path,
            sign_label="b_pitch=+0.05",
            run_stem="neutral_forward_preview_pos",
            trials=int(args.trials),
            duration=float(args.duration),
            vx=float(args.vx),
            sim_warmup_s=float(args.sim_warmup_s),
            perception_warmup_s=float(args.perception_warmup_s),
            log_dir=log_dir,
            batch_log_dir=bundle_log,
        )

    if args.sign in ("neg", "both"):
        config_path = sign_config("neg", -0.05)
        _run_group(
            config_path=config_path,
            sign_label="b_pitch=-0.05",
            run_stem="neutral_forward_preview_neg",
            trials=int(args.trials),
            duration=float(args.duration),
            vx=float(args.vx),
            sim_warmup_s=float(args.sim_warmup_s),
            perception_warmup_s=float(args.perception_warmup_s),
            log_dir=log_dir,
            batch_log_dir=bundle_log,
        )


if __name__ == "__main__":
    main()
