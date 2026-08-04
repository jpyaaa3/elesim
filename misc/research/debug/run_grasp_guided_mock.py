#!/usr/bin/env python3
"""Run Look -> Aim -> Grasp (guided) against sim mock object.

Requires elesim-sim on the configured DDS graph and perception mode=mock.
This tool starts its own Pilot participant.

Example:
  python misc/research/debug/run_grasp_guided_mock.py --object 0.5 0.0 1.2
  python misc/research/debug/run_grasp_guided_mock.py --phases aim,grasp
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_pilot.vision.pick.core import ObjectPickPhase


def _parse_phases(raw: str) -> list[str]:
    out = [p.strip().lower() for p in str(raw).split(",") if p.strip()]
    allowed = {"look", "aim", "grasp"}
    bad = [p for p in out if p not in allowed]
    if bad:
        raise SystemExit(f"unknown phases: {bad} (allowed: {', '.join(sorted(allowed))})")
    return out or ["look", "aim", "grasp"]


def _wait_pick_done(service: Any, *, timeout_s: float, label: str) -> bool:
    deadline = time.time() + float(max(timeout_s, 1.0))
    while time.time() < deadline:
        phase = str(service.state.pick_phase)
        if phase == ObjectPickPhase.DONE.value and not service.state.pick_running:
            return True
        if service.state.pick_failed:
            print(f"[mock-grasp] {label} failed | {service.state.pick_status_msg}")
            return False
        time.sleep(0.05)
    print(f"[mock-grasp] {label} timeout after {timeout_s:.1f}s")
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="Look/Aim/Grasp with mock perception object.")
    ap.add_argument("--config", default=str(ROOT / "pilot/config/default.yaml"))
    ap.add_argument(
        "--object",
        nargs=3,
        type=float,
        default=(0.5, 0.0, 1.2),
        metavar=("X", "Y", "Z"),
        help="mock object world position [m]",
    )
    ap.add_argument(
        "--phases",
        default="look,aim,grasp",
        help="comma-separated: look, aim, grasp",
    )
    ap.add_argument("--timeout", type=float, default=600.0, help="per-phase timeout [s]")
    args = ap.parse_args()
    from misc.tools.pilot_runtime import start_tool_pilot

    phases = _parse_phases(str(args.phases))
    object_xyz = tuple(float(v) for v in args.object)

    service = start_tool_pilot(
        str(args.config),
        runtime_config_path=ROOT / "pilot/config/runtime.yaml",
    )
    bundle = service.bundle
    if str(bundle.perception_config.mode).strip().lower() != "mock":
        print(
            "[mock-grasp] warning: perception mode is not 'mock' in the pilot config - "
            "set mode=mock for sim-only testing"
        )

    try:
        host_state = service.refresh_host_state()
        if host_state is None or not host_state.connected:
            print(
                "[mock-grasp] target not connected - start sim and pilot "
                "on the same DDS graph first"
            )
            return 1

        service.set_mock_object_world(*object_xyz)
        if not service.publish_mock_object_world():
            print("[mock-grasp] warning: mock object publish did not ack")

        exit_code = 0
        for phase in phases:
            if phase == "look":
                service.start_look()
            elif phase == "aim":
                service.start_aim()
            else:
                service.start_grasp()
            ok = _wait_pick_done(service, timeout_s=float(args.timeout), label=phase)
            if not ok:
                exit_code = 1
                break
            print(f"[mock-grasp] {phase} done | phase={service.state.pick_phase}")
        return exit_code
    finally:
        service.close()


if __name__ == "__main__":
    raise SystemExit(main())
