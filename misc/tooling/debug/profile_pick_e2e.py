#!/usr/bin/env python3
"""Profile Look / Ready pick phases (full stack or compute-only).

Full stack (requires elesim-router + elesim-simulator + elesim-controller):
  ELESIM_PROFILE_PICK=1 python misc/tooling/debug/profile_pick_e2e.py --phases look,ready

Compute-only (no host):
  ELESIM_PROFILE_PICK=1 python misc/tooling/debug/profile_pick_e2e.py --compute-only --cprofile
"""

from __future__ import annotations

import argparse
import cProfile
import os
import pstats
import sys
import time
from io import StringIO
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.vision.perception_bridge.hand_eye import load_hand_eye_transform
from elesim_controller.config import load_runtime_role_config
from elesim_controller.robot.arm import ik as ik_pipeline
from elesim_controller.pick import ControlClient, ControlService, PanelState
from elesim_controller.vision.pick.core import ObjectPickPhase
from elesim_controller.observability.pick_timing import (
    PickTimingCollector,
    enabled as pick_profile_enabled,
    format_report,
    install_fk_counter,
    reset_fk_count,
    uninstall_fk_counter,
)
from elesim_controller.vision.visual_servoing.feasible_ready_pose import resolve_feasible_ready_pose


def _parse_phases(raw: str) -> list[str]:
    out = [p.strip().lower() for p in str(raw).split(",") if p.strip()]
    allowed = {"look", "ready"}
    bad = [p for p in out if p not in allowed]
    if bad:
        raise SystemExit(f"unknown phases: {bad} (allowed: look, ready)")
    return out or ["look", "ready"]


def _wait_pick_done(service: ControlService, *, timeout_s: float, label: str) -> bool:
    deadline = time.time() + float(max(timeout_s, 1.0))
    while time.time() < deadline:
        if str(service.state.pick_phase) == ObjectPickPhase.DONE.value and not service.state.pick_running:
            return True
        if service.state.pick_failed:
            print(f"[profile] {label} failed | {service.state.pick_status_msg}")
            return False
        time.sleep(0.05)
    print(f"[profile] {label} timeout after {timeout_s:.1f}s")
    return False


def _run_compute_only(
    *,
    config_path: str,
    object_xyz: tuple[float, float, float],
    phases: list[str],
    use_cprofile: bool,
) -> int:
    os.environ.setdefault("ELESIM_PROFILE_PICK", "1")
    bundle, ik_context = ik_pipeline.load_solver_context(config_path)
    pk = bundle.pick_config
    ik_cfg = bundle.ik_config
    seed = (0.0, 0.0, 0.0, 0.0)
    preferred = (1.0, 0.0, 0.0)

    hand_eye_transform = None
    hand_eye_parent_frame = "node9"
    hand_eye_path = str(bundle.sim_config.hand_eye_config).strip()
    if hand_eye_path:
        try:
            hand_eye_transform, hand_eye_meta = load_hand_eye_transform(hand_eye_path)
            hand_eye_parent_frame = str(hand_eye_meta.get("parent_frame", "node9"))
        except Exception:
            hand_eye_transform = None

    def _run_once(phase: str) -> None:
        timing = PickTimingCollector()
        install_fk_counter()
        reset_fk_count()
        t0 = time.perf_counter()
        standoff = float(pk.look_pose_standoff_m if phase == "look" else pk.ready_pose_standoff_m)
        result = resolve_feasible_ready_pose(
            object_world=object_xyz,
            preferred_dir=preferred,
            standoff_m=standoff,
            ik_context=dict(ik_context),
            current_seed=seed,
            position_tol_m=float(ik_cfg.tol),
            max_iters=max(int(ik_cfg.max_iters), 1),
            tweak_rounds=int(pk.ik_align_rounds),
            max_dir_error_deg=float(
                pk.look_pose_max_dir_error_deg if phase == "look" else pk.ready_pose_max_dir_error_deg
            ),
            skip_search_under_deg=float(
                pk.look_pose_skip_search_under_deg
                if phase == "look"
                else pk.ready_pose_skip_search_under_deg
            ),
            lateral_offsets_m=tuple(
                pk.look_pose_lateral_offsets_m if phase == "look" else pk.ready_pose_lateral_offsets_m
            ),
            height_offsets_m=tuple(
                pk.look_pose_height_offsets_m if phase == "look" else pk.ready_pose_height_offsets_m
            ),
            look_dot_min=float(
                pk.look_pose_look_dot_min if phase == "look" else pk.ready_pose_look_dot_min
            ),
            hand_eye_transform=hand_eye_transform,
            hand_eye_parent_frame=hand_eye_parent_frame,
            align_top_k=int(pk.look_pose_align_top_k) if phase == "look" else 0,
            align_mode=str(pk.ik_align_mode),
            align_skip_under_deg=float(pk.ik_align_skip_under_deg),
            timing=timing,
        )
        profile = timing.to_profile(
            phase=phase,
            t_total_s=time.perf_counter() - t0,
            success=bool(result.success),
        )
        print(format_report(profile))
        print(
            f"[profile] compute-only {phase} | success={result.success} "
            f"reason={result.reason} evaluated={result.evaluated_count}"
        )
        uninstall_fk_counter()

    for phase in phases:
        if use_cprofile:
            pr = cProfile.Profile()
            pr.enable()
            _run_once(phase)
            pr.disable()
            stream = StringIO()
            pstats.Stats(pr, stream=stream).sort_stats("cumulative").print_stats(20)
            print(stream.getvalue())
        else:
            _run_once(phase)
    return 0


def _run_e2e(
    *,
    config_path: str,
    object_xyz: tuple[float, float, float],
    phases: list[str],
    timeout_s: float,
    use_cprofile: bool,
) -> int:
    os.environ.setdefault("ELESIM_PROFILE_PICK", "1")
    if not pick_profile_enabled():
        print("[profile] warning: set ELESIM_PROFILE_PICK=1 for detailed spans")

    bundle, ik_context = ik_pipeline.load_solver_context(config_path)
    hand_eye_transform = None
    hand_eye_parent_frame = "node9"
    hand_eye_path = str(bundle.sim_config.hand_eye_config).strip()
    if hand_eye_path:
        try:
            hand_eye_transform, hand_eye_meta = load_hand_eye_transform(hand_eye_path)
            hand_eye_parent_frame = str(hand_eye_meta.get("parent_frame", "node9"))
        except Exception:
            hand_eye_transform = None

    runtime = load_runtime_role_config(ROOT / "controller/config/runtime.yaml")
    link = ControlClient(runtime.bind_endpoint, cfg=bundle.mapping_config)
    state = PanelState()
    service = ControlService(
        state,
        client=link,
        mapping_cfg=bundle.mapping_config,
        ik_cfg=bundle.ik_config,
        ik_context=ik_context,
        config_path=config_path,
        perception_cfg=bundle.perception_config,
        pick_cfg=bundle.pick_config,
        hand_eye_transform=hand_eye_transform,
        hand_eye_parent_frame=hand_eye_parent_frame,
        use_hardware=bool(bundle.sim_config.use_hardware),
    )

    try:
        host_state = service.refresh_host_state()
        if host_state is None or not host_state.connected:
            print("[profile] target not connected - start router, simulator, and controller first")
            return 1

        service.set_mock_object_world(*object_xyz)
        if not service.publish_mock_object_world():
            print("[profile] warning: mock object publish did not ack")

        exit_code = 0
        for phase in phases:
            if phase == "look":
                if use_cprofile:
                    pr = cProfile.Profile()
                    pr.enable()
                service.start_look()
                ok = _wait_pick_done(service, timeout_s=timeout_s, label="look")
                if use_cprofile:
                    pr.disable()
                    stream = StringIO()
                    pstats.Stats(pr, stream=stream).sort_stats("cumulative").print_stats(20)
                    print(stream.getvalue())
            elif phase == "ready":
                if use_cprofile:
                    pr = cProfile.Profile()
                    pr.enable()
                service.start_ready_pose()
                ok = _wait_pick_done(service, timeout_s=timeout_s, label="ready")
                if use_cprofile:
                    pr.disable()
                    stream = StringIO()
                    pstats.Stats(pr, stream=stream).sort_stats("cumulative").print_stats(20)
                    print(stream.getvalue())
            else:
                continue

            if service._last_pick_profile is not None:
                print(format_report(service._last_pick_profile))
            if not ok:
                exit_code = 1
        return exit_code
    finally:
        service.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="Profile Look / Ready pick phases.")
    ap.add_argument(
        "--config",
        default=str(ROOT / "controller/config/default.yaml"),
        help="path to the controller config",
    )
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
        default="look,ready",
        help="comma-separated phases: look, ready",
    )
    ap.add_argument(
        "--timeout",
        type=float,
        default=300.0,
        help="per-phase wait timeout [s] (E2E only, default 5 min)",
    )
    ap.add_argument(
        "--compute-only",
        action="store_true",
        help="run resolve_feasible_ready_pose without host/sim",
    )
    ap.add_argument(
        "--cprofile",
        action="store_true",
        help="print cProfile top-20 per phase",
    )
    args = ap.parse_args()

    phases = _parse_phases(args.phases)
    object_xyz = (float(args.object[0]), float(args.object[1]), float(args.object[2]))

    if bool(args.compute_only):
        return _run_compute_only(
            config_path=str(args.config),
            object_xyz=object_xyz,
            phases=phases,
            use_cprofile=bool(args.cprofile),
        )
    return _run_e2e(
        config_path=str(args.config),
        object_xyz=object_xyz,
        phases=phases,
        timeout_s=float(args.timeout),
        use_cprofile=bool(args.cprofile),
    )


if __name__ == "__main__":
    raise SystemExit(main())
