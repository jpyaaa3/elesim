"""
Roll-sweep scan: traverse roll limit-to-limit, capture at fixed ANGLE steps,
repeat, fuse with FK poses.

Why angle-triggered and not time-triggered: the capture trigger is the roll
angle crossing the next step boundary, so the angular coverage of the fused
cloud is a property of the plan, not of how fast the joint happened to move.
A time-triggered sweep bunches frames wherever the joint slowed down.

Why repeated sweeps: each pass re-observes the same angles from a slightly
different settled position, so averaging across passes suppresses per-frame
depth noise without ICP. Passes alternate direction (min->max, max->min) so no
extra travel is wasted returning to the start.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Sequence

import numpy as np

from .fk_pose import FkPoseProvider, FkPoseSample
from .plan import RollScanConfig, RollSweepPlan, build_plan
from .fusion import (
    FusionResult,
    anchor_from_frame,
    box_crop,
    build_provenance,
    camera_depth_gate,
    clean_world_frame,
    fuse,
    transform_points,
    voxel_downsample,
)


def _summarize_notes(notes: Sequence[str], top: int = 3) -> str:
    """Collapse per-frame notes into the reasons that actually dominated."""
    if not notes:
        return "no per-frame notes recorded"
    buckets: dict[str, int] = {}
    for n in notes:
        key = str(n).split(":", 1)[-1].strip() if ":" in str(n) else str(n)
        buckets[key] = buckets.get(key, 0) + 1
    ranked = sorted(buckets.items(), key=lambda kv: -kv[1])[:top]
    return "drops: " + ", ".join(f"{k} x{v}" for k, v in ranked)


@dataclass
class ScanProgress:
    """Snapshot for the UI. Plain values only -- crosses a thread boundary."""

    running: bool = False
    phase: str = "idle"  # idle | opening | sweeping | fusing | fitting | done | failed
    stop_index: int = 0
    n_stops: int = 0
    sweep: int = 0
    n_sweeps: int = 0
    roll_cmd_deg: float = 0.0
    roll_actual_deg: float = 0.0
    frames_kept: int = 0
    points_kept: int = 0
    msg: str = ""
    diameter_mm: Optional[float] = None
    arc_span_deg: Optional[float] = None
    residual_rms_mm: Optional[float] = None
    surface: str = ""

    @property
    def fraction(self) -> float:
        return 0.0 if self.n_stops <= 0 else min(1.0, self.stop_index / float(self.n_stops))


class RollSweepScan:
    """
    Runs the sweep on a worker thread.

    Collaborators are injected so the same object serves the live rig, the sim,
    and the self-test:
      ``pose_provider``  FK -> world<-camera pose
      ``read_q4()``      current (linear, roll, theta1, theta2) + its age
      ``command_roll(deg)`` move the roll joint
      ``grab_points()``  (N, 3) camera-frame points for the current view
    """

    def __init__(
        self,
        *,
        cfg: RollScanConfig,
        pose_provider: FkPoseProvider,
        read_q4: Callable[[], tuple[Sequence[float], float]],
        command_roll: Callable[[float], None],
        grab_points: Callable[[], Optional[np.ndarray]],
        on_progress: Optional[Callable[[ScanProgress], None]] = None,
        open_camera: Optional[Callable[[], None]] = None,
        close_camera: Optional[Callable[[], None]] = None,
        read_intrinsics: Optional[Callable[[], Optional[Any]]] = None,
    ) -> None:
        self.cfg = cfg
        self.plan = build_plan(cfg)
        self._pose = pose_provider
        self._read_q4 = read_q4
        self._command_roll = command_roll
        self._grab = grab_points
        self._on_progress = on_progress
        self._open_camera = open_camera
        self._close_camera = close_camera
        self._read_intrinsics = read_intrinsics

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._progress = ScanProgress(n_stops=self.plan.n_stops, n_sweeps=self.plan.sweeps)
        self._result: Optional[FusionResult] = None
        self._report: Optional[dict[str, Any]] = None
        # kept frames live on the object, not just in _run's locals, so the
        # fitting stage can run in a separate thread without re-capturing
        self._frames: list[np.ndarray] = []
        self._cams: list[np.ndarray] = []
        self._looks: list[np.ndarray] = []
        self._rolls: list[float] = []
        self._stage_logs = 0

    # ---------------- lifecycle ----------------

    def is_running(self) -> bool:
        t = self._thread
        return bool(t is not None and t.is_alive())

    def progress(self) -> ScanProgress:
        with self._lock:
            p = self._progress
            return ScanProgress(**{k: getattr(p, k) for k in p.__dataclass_fields__})

    def result(self) -> Optional[FusionResult]:
        with self._lock:
            return self._result

    def report(self) -> Optional[dict[str, Any]]:
        with self._lock:
            return dict(self._report) if self._report else None

    def start(self) -> None:
        if self.is_running():
            raise RuntimeError("roll scan already running")
        self._stop.clear()
        with self._lock:
            self._progress = ScanProgress(
                running=True, phase="opening", n_stops=self.plan.n_stops,
                n_sweeps=self.plan.sweeps, msg="starting",
            )
            self._result = None
            self._report = None
        self._thread = threading.Thread(target=self._run, name="roll-sweep-scan", daemon=True)
        self._thread.start()

    def stop(self, *, timeout_s: float = 10.0) -> bool:
        self._stop.set()
        t = self._thread
        if t is None:
            return True
        t.join(timeout=float(timeout_s))
        return not t.is_alive()

    def wait(self, *, timeout_s: Optional[float] = None) -> bool:
        """Block until the sweep thread finishes. Returns True if it did."""
        t = self._thread
        if t is None:
            return True
        t.join(timeout=timeout_s)
        return not t.is_alive()

    def frames(self) -> list[np.ndarray]:
        """Per-frame world points kept by the sweep (empty until it finishes)."""
        with self._lock:
            return [f.copy() for f in self._frames]

    def cam_positions(self) -> list[np.ndarray]:
        """Camera origin per kept frame, index-aligned with ``frames()``."""
        with self._lock:
            return [c.copy() for c in self._cams]

    def roll_angles_deg(self) -> list[float]:
        """Settled roll angle per kept frame, index-aligned with ``frames()``."""
        with self._lock:
            return list(self._rolls)

    # ---------------- internals ----------------

    def _set(self, **kw: Any) -> None:
        with self._lock:
            for k, v in kw.items():
                setattr(self._progress, k, v)
            snap = ScanProgress(
                **{k: getattr(self._progress, k) for k in self._progress.__dataclass_fields__}
            )
        if self._on_progress is not None:
            try:
                self._on_progress(snap)
            except Exception:  # noqa: BLE001
                pass

    def _sample_pose(self) -> Optional[FkPoseSample]:
        q4, age = self._read_q4()
        if q4 is None:
            return None
        stale = float(age) > float(self.cfg.max_stale_pose_s)
        try:
            return self._pose.sample(q4, stale=stale, age_s=float(age))
        except Exception as exc:  # noqa: BLE001
            self._set(msg=f"FK pose failed: {exc}")
            return None

    def _goto_roll(self, target_deg: float) -> Optional[FkPoseSample]:
        """Command roll and wait until it settles. Returns the settled pose."""
        self._command_roll(float(target_deg))
        deadline = time.time() + float(self.cfg.step_timeout_s)
        tol = float(self.cfg.settle_tol_deg)
        stable_since: Optional[float] = None
        last: Optional[FkPoseSample] = None
        while not self._stop.is_set():
            last = self._sample_pose()
            if last is not None:
                err = abs(math.degrees(last.roll_rad) - float(target_deg))
                if err <= tol:
                    if stable_since is None:
                        stable_since = time.time()
                    elif time.time() - stable_since >= float(self.cfg.settle_s):
                        return last
                else:
                    stable_since = None
            if time.time() > deadline:
                # timeout is not fatal: record the pose we actually have, so the
                # frame is fused at its true angle rather than dropped
                return last
            time.sleep(0.01)
        return None

    def _capture_at(
        self,
        pose: FkPoseSample,
        center: Optional[np.ndarray],
        notes: list[str],
        tag: str,
    ) -> tuple[Optional[np.ndarray], int]:
        """Grab, transform and keep one frame. Returns (anchor, points kept)."""
        pts_cam = self._grab()
        if pts_cam is None or len(pts_cam) == 0:
            notes.append(f"{tag}: no depth")
            return center, 0
        if center is None:
            center = anchor_from_frame(pts_cam, pose.R, pose.t, box_half=self.cfg.box_half)
            if center is None:
                notes.append(f"{tag}: anchor too sparse")
                return None, 0
        n_valid = len(pts_cam)
        pts_cam = camera_depth_gate(pts_cam, pose.R, pose.t, center, self.cfg.box_half)
        n_gated = len(pts_cam)
        pw = transform_points(pts_cam, pose.R, pose.t)
        pw = box_crop(pw, center, self.cfg.box_half)
        n_box = len(pw)
        pw = clean_world_frame(pw)
        n_clean = len(pw)
        if self._stage_logs < 6:
            # per-stage counts for the first few frames: "too few points" alone
            # cannot distinguish an object out of the box from plane removal
            # eating the object, and those need opposite fixes
            self._stage_logs += 1
            notes.append(
                f"{tag}: valid={n_valid} gated={n_gated} box={n_box} clean={n_clean}"
            )
        if n_clean <= int(self.cfg.min_points_per_frame):
            why = "nothing in the crop box" if n_box <= int(self.cfg.min_points_per_frame) \
                else f"plane removal left {n_clean} of {n_box}"
            notes.append(f"{tag}: too few points -- {why}")
            return center, 0
        pw = voxel_downsample(pw, self.cfg.frame_voxel)
        with self._lock:
            self._frames.append(pw)
            self._cams.append(np.asarray(pose.t, dtype=float).copy())
            self._looks.append(np.asarray(pose.look, dtype=float).copy())
            self._rolls.append(math.degrees(pose.roll_rad))
        return center, len(pw)

    def _visible_window(
        self, center: np.ndarray, lo: float, hi: float
    ) -> tuple[float, float]:
        """Roll range over which the anchored target stays in frame."""
        intr = None
        if self._read_intrinsics is not None:
            try:
                intr = self._read_intrinsics()
            except Exception:  # noqa: BLE001
                intr = None
        if intr is None:
            return lo, hi
        q4, _age = self._read_q4()
        try:
            return self._pose.visible_roll_window_deg(
                q4, center,
                fx=intr.fx, fy=intr.fy, cx=intr.cx, cy=intr.cy,
                width=intr.width, height=intr.height, lo_deg=lo, hi_deg=hi,
            )
        except Exception:  # noqa: BLE001
            return lo, hi

    def _run_continuous(self, notes: list[str]) -> Optional[np.ndarray]:
        """
        Traverse each leg at a fixed rate and capture on the fly.

        The commanded angle is a time-based ramp rather than a sequence of
        setpoints, so the joint never decelerates to a stop; a frame is kept
        whenever the MEASURED roll has advanced a full step since the last kept
        one, which keeps angular coverage a property of the plan even though the
        capture cadence is whatever the pipeline can sustain.
        """
        cfg = self.cfg
        lo, hi = cfg.span_deg()
        rate = max(float(cfg.sweep_rate_deg_s), 1e-6)
        center: Optional[np.ndarray] = None
        kept_total = 0
        stop_i = 0
        # anchor BEFORE traversing, from where the arm already is: the anchor
        # decides the crop box and the visible window, and taking it at a joint
        # limit (as the leg loop would) is the least representative viewpoint
        first = self._sample_pose()
        if first is not None:
            center, kept = self._capture_at(first, None, notes, "anchor")
            if kept:
                kept_total += kept
                stop_i += 1
        for leg in range(max(int(cfg.sweeps), 1)):
            if self._stop.is_set():
                notes.append(f"stopped by user during sweep {leg + 1}")
                break
            start, end = (lo, hi) if leg % 2 == 0 else (hi, lo)
            begin = self._goto_roll(start)
            if begin is None:
                notes.append(f"sweep {leg + 1}: could not reach the start angle")
                continue
            if center is not None and leg == 0:
                lo2, hi2 = self._visible_window(center, lo, hi)
                if (hi2 - lo2) < (hi - lo) - 1e-6:
                    notes.append(
                        f"visible window {lo2:+.0f}..{hi2:+.0f} deg of {lo:+.0f}..{hi:+.0f} "
                        f"(target leaves the frame outside it)"
                    )
                    lo, hi = lo2, hi2
                    start, end = (lo, hi) if leg % 2 == 0 else (hi, lo)
            sign = 1.0 if end >= start else -1.0
            t0 = time.time()
            last_kept: Optional[float] = None
            deadline = t0 + abs(end - start) / rate + float(cfg.step_timeout_s) * 3.0
            while not self._stop.is_set():
                cmd = start + sign * rate * (time.time() - t0)
                cmd = min(cmd, end) if sign > 0 else max(cmd, end)
                self._command_roll(float(cmd))

                before = self._sample_pose()
                pts_ready = before is not None
                if pts_ready:
                    after = self._sample_pose()
                    straddle = abs(
                        math.degrees((after or before).roll_rad - before.roll_rad)
                    )
                    mid = after if after is not None else before
                    roll_now = math.degrees(mid.roll_rad)
                    advanced = last_kept is None or abs(roll_now - last_kept) >= float(cfg.step_deg)
                    if straddle > float(cfg.max_pose_straddle_deg):
                        notes.append(f"leg{leg}: pose straddled {straddle:.2f} deg, frame dropped")
                    elif advanced:
                        center, kept = self._capture_at(mid, center, notes, f"leg{leg}@{roll_now:+.1f}")
                        if kept:
                            last_kept = roll_now
                            kept_total += kept
                        stop_i += 1
                        self._set(
                            stop_index=stop_i, sweep=leg + 1,
                            roll_cmd_deg=float(cmd), roll_actual_deg=float(roll_now),
                            frames_kept=len(self._frames), points_kept=kept_total,
                            msg=f"roll {roll_now:+.1f} deg  kept {kept}",
                        )
                reached = abs(cmd - end) < 1e-6 and (
                    before is not None
                    and abs(math.degrees(before.roll_rad) - end) <= float(cfg.settle_tol_deg) * 3.0
                )
                if reached or time.time() > deadline:
                    if not reached:
                        notes.append(f"sweep {leg + 1}: traverse timed out at cmd {cmd:+.1f}")
                    break
        return center

    def _finish(self, center: Optional[np.ndarray], notes: list[str]) -> None:
        """Fuse what the sweep kept and publish the result. Shared by both modes."""
        with self._lock:
            frames = list(self._frames)
            cams = list(self._cams)
            looks = list(self._looks)
            rolls = list(self._rolls)
        if len(frames) < 2:
            self._set(
                running=False, phase="failed",
                msg=(f"need >=2 usable frames, got {len(frames)}. "
                     f"{_summarize_notes(notes)}"),
            )
            return

        self._set(phase="fusing", msg=f"fusing {len(frames)} frames")
        fused = fuse(frames, voxel=self.cfg.fuse_voxel)
        stacked, cam_rows = build_provenance(frames, cams)
        samples = [
            FkPoseSample(R=np.eye(3), t=c, q4=np.zeros(4), roll_rad=math.radians(r))
            for c, r in zip(cams, rolls)
        ]
        res = FusionResult(
            fused=fused,
            stacked=stacked,
            cam_positions=cam_rows,
            n_frames=len(frames),
            center=center,
            roll_span_deg=(max(rolls) - min(rolls)) if rolls else 0.0,
            view_span_deg=self._pose.view_span_deg(
                [FkPoseSample(R=_look_to_R(l), t=c, q4=np.zeros(4), roll_rad=0.0)
                 for l, c in zip(looks, cams)]
            ),
            observation_span_deg=self._pose.observation_span_deg(
                samples, center if center is not None else fused.mean(axis=0)
            ),
            baseline_span_m=self._pose.baseline_span_m(samples),
            sweeps=self.plan.sweeps,
            notes=notes,
        )
        with self._lock:
            self._result = res
        self._set(
            running=False, phase="done",
            msg=(f"{len(frames)} frames, {len(fused)} pts, "
                 f"roll span {res.roll_span_deg:.0f} deg, "
                 f"object-view span {res.observation_span_deg:.0f} deg, "
                 f"camera travel {res.baseline_span_m*1000:.0f} mm"),
        )

    def _run(self) -> None:
        with self._lock:
            self._frames, self._cams, self._looks, self._rolls = [], [], [], []
        frames = self._frames
        cams = self._cams
        looks = self._looks
        rolls = self._rolls
        center: Optional[np.ndarray] = None
        notes: list[str] = []
        try:
            if self._open_camera is not None:
                self._open_camera()
            if bool(getattr(self.cfg, "continuous", False)):
                self._set(
                    phase="sweeping",
                    msg=(f"continuous, {self.plan.sweeps} sweep(s) at "
                         f"{self.cfg.sweep_rate_deg_s:g} deg/s"),
                )
                center = self._run_continuous(notes)
                return self._finish(center, notes)

            self._set(phase="sweeping", msg=f"{self.plan.n_stops} stops")
            per_sweep = max(1, self.plan.n_stops // max(self.plan.sweeps, 1))
            for idx, target in enumerate(self.plan.angles_deg):
                if self._stop.is_set():
                    notes.append(f"stopped by user at stop {idx}/{self.plan.n_stops}")
                    break
                sweep_i = idx // per_sweep
                pose = self._goto_roll(target)
                if pose is None:
                    notes.append(f"stop {idx}: no pose")
                    continue
                actual_deg = math.degrees(pose.roll_rad)
                center, kept = self._capture_at(pose, center, notes, f"stop {idx}")
                self._set(
                    stop_index=idx + 1,
                    sweep=min(sweep_i + 1, self.plan.sweeps),
                    roll_cmd_deg=float(target),
                    roll_actual_deg=float(actual_deg),
                    frames_kept=len(frames),
                    points_kept=int(sum(len(f) for f in frames)),
                    msg=f"roll {actual_deg:+.1f} deg  kept {kept}",
                )

            self._finish(center, notes)
        except Exception as exc:  # noqa: BLE001
            self._set(running=False, phase="failed", msg=f"scan failed: {exc}")
        finally:
            if self._close_camera is not None:
                try:
                    self._close_camera()
                except Exception:  # noqa: BLE001
                    pass


def _look_to_R(look: np.ndarray) -> np.ndarray:
    """Minimal rotation whose +Z column is ``look`` (only the axis is used)."""
    z = np.asarray(look, dtype=float)
    n = float(np.linalg.norm(z))
    if n < 1e-12:
        return np.eye(3)
    z = z / n
    a = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(a, z)
    x /= max(float(np.linalg.norm(x)), 1e-12)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)
