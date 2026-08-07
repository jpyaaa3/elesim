"""
Binds a roll-sweep scan to the running control stack, and reports the fit.

The scan needs four things from the stack: the FK context and hand-eye
extrinsics (to build poses), the live joint state (to know where roll actually
is), a way to command roll, and a ZED. This module supplies the glue so
``ControlService`` only gains start/stop/progress.

The report stage mirrors the bench's: fit the fused cloud, classify the wall
with per-point provenance, then fit the single best frame the same way so the
fused-vs-single comparison is apples to apples. Raw frames are written BEFORE
fitting, so a bad fit never costs a re-scan.
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import asdict
from typing import Any, Callable, Optional, Sequence

import numpy as np

from . import geometry
from .fk_pose import FkPoseProvider
from .fusion import FusionResult, classify_surface_world
from .plan import RollScanConfig, build_plan
from .roll_sweep import RollSweepScan, ScanProgress


class ScanUnavailable(RuntimeError):
    """Raised when the stack cannot support a scan (no FK, no camera, ...)."""


def _brief(fit: dict[str, Any]) -> dict[str, Any]:
    def num(key: str, scale: float = 1.0, nd: int = 2) -> Optional[float]:
        v = fit.get(key)
        return None if v is None else round(float(v) * scale, nd)

    return {
        "diameter_mm": num("diameter", 1000.0),
        "arc_span_deg": num("arc_span_deg", 1.0, 1),
        "residual_rms_mm": num("residual_rms", 1000.0),
        "radius_sigma_mm": num("radius_sigma", 1000.0),
        "surface": str(fit.get("surface", "")),
    }


def fit_and_report(
    res: FusionResult,
    cfg: RollScanConfig,
    *,
    frames: Sequence[np.ndarray],
    cam_positions: Sequence[np.ndarray],
    gt_diameter_m: Optional[float] = None,
    log: Callable[[str], None] = print,
) -> dict[str, Any]:
    """Fit the fused cloud, compare against the best single view, write outputs."""
    if not geometry.available():
        raise ScanUnavailable(geometry.status())

    os.makedirs(cfg.outdir, exist_ok=True)
    stem = os.path.join(cfg.outdir, cfg.label)

    if cfg.save_frames_npz and frames:
        try:
            np.savez(
                stem + "_frames.npz",
                points=np.vstack([f for f in frames if len(f)]).astype(np.float32),
                frame_sizes=np.array([len(f) for f in frames], dtype=np.int64),
                cams=np.asarray(cam_positions, dtype=np.float32),
            )
            log(f"[scan] raw frames -> {stem}_frames.npz")
        except Exception as exc:  # noqa: BLE001
            log(f"[scan] frame save failed: {exc}")

    fused = res.fused
    for _ in range(3):
        kept = geometry.remove_dominant_plane(fused, min_fraction=0.15)
        if len(kept) == len(fused):
            break
        fused = kept
    geometry.write_ply(stem + "_fused.ply", fused)
    log(f"[scan] fused cloud -> {stem}_fused.ply ({len(fused)} pts)")

    fit = geometry.fit_cylinder(
        fused,
        inlier_tol=cfg.inlier_tol,
        rng=np.random.default_rng(0),
        prefer_exterior=False,
        min_inlier_ratio=0.15,
    )
    surface, ext_frac = classify_surface_world(
        res.stacked, res.cam_positions, fit["axis"], fit["axis_point"], fit["radius"],
        tol=2 * cfg.inlier_tol,
    )
    if surface != "exterior":
        # an open container leaves BOTH walls in the fused cloud; strip this ring
        # and give the remainder one chance to yield the exterior
        ax = np.asarray(fit["axis"], dtype=float)
        ax = ax / max(float(np.linalg.norm(ax)), 1e-12)
        X = fused - np.asarray(fit["axis_point"], dtype=float)
        dist = np.linalg.norm(X - (X @ ax)[:, None] * ax, axis=1)
        rest = fused[np.abs(dist - float(fit["radius"])) > 2 * cfg.inlier_tol]
        if len(rest) > 500:
            try:
                fit2 = geometry.fit_cylinder(
                    rest, inlier_tol=cfg.inlier_tol, rng=np.random.default_rng(1),
                    prefer_exterior=False, min_inlier_ratio=0.15,
                )
                s2, e2 = classify_surface_world(
                    res.stacked, res.cam_positions, fit2["axis"], fit2["axis_point"],
                    fit2["radius"], tol=2 * cfg.inlier_tol,
                )
                if s2 == "exterior" and float(fit2.get("conditioning", 0.0)) >= 3.0:
                    fit2["secondary_interior_diameter"] = fit["diameter"]
                    fit, surface, ext_frac = fit2, s2, e2
            except Exception:  # noqa: BLE001
                pass
    fit["surface"] = surface
    fit["exterior_fraction"] = ext_frac

    # single-view baseline: among the 3 largest frames keep the best-conditioned
    # fit ("largest" alone can be a residue-heavy frame and fit garbage)
    single: Optional[dict[str, Any]] = None
    order = np.argsort([len(f) for f in frames])[::-1][:3]
    for k in order:
        fr = np.asarray(frames[int(k)], dtype=float)
        if len(fr) < 300:
            continue
        try:
            cand = geometry.fit_cylinder(
                fr, inlier_tol=cfg.inlier_tol, rng=np.random.default_rng(0),
                prefer_exterior=False, min_inlier_ratio=0.15,
            )
        except Exception:  # noqa: BLE001
            continue
        s1, e1 = classify_surface_world(
            fr,
            np.tile(np.asarray(cam_positions[int(k)], dtype=float).reshape(1, 3), (len(fr), 1)),
            cand["axis"], cand["axis_point"], cand["radius"], tol=2 * cfg.inlier_tol,
        )
        # a single view can never see more than ~180 deg of an exterior wall
        if s1 == "exterior" and float(cand.get("arc_span_deg", 0.0)) > 185.0:
            s1 = "mixed"
        cand["surface"] = s1
        cand["exterior_fraction"] = e1
        if single is None or (
            float(cand.get("conditioning", 0.0)) > float(single.get("conditioning", 0.0))
            and cand["surface"] != "mixed"
        ):
            single = cand

    report: dict[str, Any] = {
        "pose_source": "fk",  # the whole point: no VIO in this path
        "registration": "transform-and-merge (no ICP)",
        "n_frames": res.n_frames,
        "sweeps": res.sweeps,
        "roll_span_deg": round(res.roll_span_deg, 1),
        "view_span_deg": round(res.view_span_deg, 1),
        "observation_span_deg": round(res.observation_span_deg, 1),
        "camera_travel_mm": round(res.baseline_span_m * 1000.0, 1),
        "fused": _brief(fit),
        "single_view_best": _brief(single) if single else None,
        "gt_diameter_m": gt_diameter_m,
        "fused_err_mm": (
            round((float(fit["diameter"]) - float(gt_diameter_m)) * 1000.0, 2)
            if gt_diameter_m
            else None
        ),
        "notes": list(res.notes),
        "outputs": {
            "fused_ply": stem + "_fused.ply",
            "report_json": stem + "_scan.json",
            "cross_section_txt": stem + "_cross_section.txt",
            "frames_npz": (stem + "_frames.npz") if cfg.save_frames_npz else None,
        },
    }

    with open(stem + "_scan.json", "w", encoding="utf-8") as f:
        json.dump({"report": report, "config": asdict(cfg), "fit": _jsonable(fit)}, f, indent=2)
    try:
        cs = fit.get("cross_section") or {}
        verts = cs.get("newarc_vertices")
        if verts:
            with open(stem + "_cross_section.txt", "w", encoding="utf-8") as f:
                f.write(str(verts) + "\n")
    except Exception as exc:  # noqa: BLE001
        log(f"[scan] cross-section write failed: {exc}")

    log(json.dumps(report, indent=2))
    return report


def _jsonable(obj: Any) -> Any:
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    return obj


def build_scan(
    *,
    cfg: RollScanConfig,
    ik_context: dict[str, Any],
    hand_eye_transform: np.ndarray,
    hand_eye_parent_frame: str,
    read_q4: Callable[[], tuple[Sequence[float], float]],
    command_roll_deg: Callable[[float], None],
    on_progress: Optional[Callable[[ScanProgress], None]] = None,
    camera_factory: Optional[Callable[[], Any]] = None,
    log: Callable[[str], None] = print,
) -> RollSweepScan:
    """
    Assemble a ready-to-start scan.

    ``camera_factory`` is injected so a test or the sim can substitute a frame
    source; the default opens a real ZED.
    """
    if not ik_context:
        raise ScanUnavailable("no ik_context: FK poses unavailable (load a solver context first)")
    if hand_eye_transform is None:
        raise ScanUnavailable("no hand-eye extrinsics: cannot place the camera on the arm")

    pose = FkPoseProvider(
        ik_context=ik_context,
        hand_eye_transform=hand_eye_transform,
        parent_frame=hand_eye_parent_frame,
    )

    holder: dict[str, Any] = {"cam": None}

    def _open() -> None:
        if camera_factory is not None:
            holder["cam"] = camera_factory()
            return
        from .zed_capture import ZedScanCamera

        cam = ZedScanCamera(
            resolution=cfg.resolution,
            depth_mode=cfg.depth_mode,
            fps=cfg.fps,
            min_depth=cfg.min_depth,
            max_depth=cfg.max_depth,
            confidence=cfg.confidence,
            texture_confidence=cfg.texture_confidence,
            want_color=False,
        )
        cam.warmup(10)
        holder["cam"] = cam
        log(f"[scan] ZED open | {cfg.resolution} {cfg.depth_mode} intrinsics={cam.intrinsics}")

    def _close() -> None:
        cam = holder.get("cam")
        holder["cam"] = None
        if cam is not None and hasattr(cam, "close"):
            cam.close()

    def _grab() -> Optional[np.ndarray]:
        cam = holder.get("cam")
        if cam is None:
            return None
        frame = cam.grab()
        if frame is None:
            return None
        from .zed_capture import valid_points

        return valid_points(frame.xyz, min_depth=cfg.min_depth, max_depth=cfg.max_depth)

    def _intrinsics() -> Optional[Any]:
        cam = holder.get("cam")
        return None if cam is None else getattr(cam, "intrinsics", None)

    return RollSweepScan(
        cfg=cfg,
        pose_provider=pose,
        read_q4=read_q4,
        command_roll=command_roll_deg,
        grab_points=_grab,
        on_progress=on_progress,
        open_camera=_open,
        close_camera=_close,
        read_intrinsics=_intrinsics,
    )


def describe_plan(cfg: RollScanConfig) -> str:
    p = build_plan(cfg)
    return (
        f"roll {p.lo_deg:+.0f}..{p.hi_deg:+.0f} deg, step {p.step_deg:g} deg, "
        f"{p.sweeps} sweep(s) -> {p.n_stops} stops, ~{p.estimated_duration_s(cfg):.0f} s"
    )
