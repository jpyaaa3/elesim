"""
Roll-sweep parameters and stop-angle planning.

Kept free of camera, FK and geometry-backend imports so the config loader and
the UI can read scan parameters without pulling in the ZED SDK or mutating
``sys.path`` to find the fitting bench.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class RollScanConfig:
    """Scan parameters. Angles in degrees, lengths in metres."""

    # Angles here are JOINT degrees q, where q=0 is the middle of the roll range.
    # The control/motor unit u is a different scale: u 0..360 maps to q +90..-90
    # (2:1, and inverted by command_direction), so u=0 is one end, u=180 the
    # centre, u=360 the other end. A q value of 4 deg is 8 u.
    roll_min_deg: float = -90.0
    roll_max_deg: float = 90.0
    # Total q span to sweep, CENTRED on the roll angle the anchor was taken at.
    # 0 means the full joint range. Sweeping the full range starts at an end,
    # 90 deg of q away from centre, where the object being scanned is usually not
    # in view at all -- which is how a live sweep kept exactly one frame.
    span_deg: float = 90.0
    # Roll angle treated as "centre" (q=0 is the middle of the range, u=180).
    # The scan parks here before it starts and returns here when it ends, so a
    # run always begins and finishes from a known, object-facing pose instead of
    # wherever the previous action left the joint.
    home_roll_deg: float = 0.0
    return_home: bool = True
    home_timeout_s: float = 6.0
    # keep a margin off the mechanical limit; the sweep is not trying to
    # exercise the joint stop, and pressing into it stalls the settle detector
    margin_deg: float = 3.0
    step_deg: float = 4.0
    sweeps: int = 1
    # Capture WHILE the joint traverses, instead of stopping at each angle.
    # Stopping cost ~0.3 s per stop in settle alone, and stop-and-go buys nothing
    # here: the pose comes from the measured joint state either way, so a frame
    # taken mid-traverse is registered just as well as one taken at rest.
    continuous: bool = True
    # chosen so the frame cadence lands near one frame per step_deg; too fast and
    # the traverse outruns per-frame processing, leaving gaps in coverage
    sweep_rate_deg_s: float = 12.0
    settle_s: float = 0.12
    settle_tol_deg: float = 0.35
    step_timeout_s: float = 2.0
    # a frame whose pose straddles more than this much roll is dropped: at
    # 12 deg/s a 50 ms joint-read period is ~0.6 deg of ambiguity, and anything
    # much larger would register the cloud at an angle the arm was never at
    max_pose_straddle_deg: float = 1.5
    box_half: float = 0.15
    frame_voxel: float = 0.002
    fuse_voxel: float = 0.002
    inlier_tol: float = 0.005
    min_depth: float = 0.15
    max_depth: float = 1.5
    min_points_per_frame: int = 200
    max_stale_pose_s: float = 0.15
    resolution: str = "HD1080"
    depth_mode: str = "NEURAL"
    fps: int = 15
    confidence: int = 50
    texture_confidence: int = 100
    label: str = "rollscan"
    outdir: str = "engine/logs/roll_scan"
    save_frames_npz: bool = True

    def limit_span_deg(self) -> tuple[float, float]:
        """Usable sweep range after the limit margin, in joint degrees."""
        lo = float(self.roll_min_deg) + float(self.margin_deg)
        hi = float(self.roll_max_deg) - float(self.margin_deg)
        if hi <= lo:
            mid = 0.5 * (float(self.roll_min_deg) + float(self.roll_max_deg))
            return mid, mid
        return lo, hi

    def centred_span_deg(self, centre_deg: float) -> tuple[float, float]:
        """``span_deg`` about ``centre_deg``, clamped to the joint limits."""
        lo, hi = self.limit_span_deg()
        span = float(self.span_deg)
        if span <= 0.0 or span >= (hi - lo):
            return lo, hi
        half = 0.5 * span
        c = min(max(float(centre_deg), lo + half), hi - half)
        return c - half, c + half

    # kept so plan building and older callers keep working unchanged
    def span_deg_range(self) -> tuple[float, float]:
        return self.limit_span_deg()


@dataclass
class RollSweepPlan:
    """The ordered roll angles the scan will stop at."""

    angles_deg: list[float]
    sweeps: int
    step_deg: float
    lo_deg: float
    hi_deg: float

    @property
    def n_stops(self) -> int:
        return len(self.angles_deg)

    def coverage_deg(self) -> float:
        return abs(self.hi_deg - self.lo_deg)

    _PIXELS = {"VGA": 672 * 376, "HD720": 1280 * 720, "HD1080": 1920 * 1080,
               "HD2K": 2208 * 1242}

    def estimated_duration_s(self, cfg: RollScanConfig) -> float:
        """
        Rough wall-clock estimate, for the UI to show before committing.

        Per-frame cost is dominated by touching every pixel of the cloud
        (validity mask, transform, crop), which scales with resolution -- an
        HD1080 frame is 2.1 M points. Measured at roughly 0.15 s per megapixel
        on a Jetson, so the constant is not resolution-independent.
        """
        px = self._PIXELS.get(str(cfg.resolution).strip().upper(), self._PIXELS["HD1080"])
        grab_s = max(1.0 / max(float(cfg.fps), 1.0), 0.0)
        process_s = 0.16 * (px / 1.0e6)   # measured ~0.33 s/frame at HD1080
        per_frame = grab_s + process_s
        if bool(cfg.continuous):
            # bounded by traverse time; frames are taken on the fly, so the only
            # way processing dominates is if it cannot keep up with the rate
            travel_s = self.coverage_deg() / max(float(cfg.sweep_rate_deg_s), 1e-6)
            return self.sweeps * max(travel_s, self.n_stops / self.sweeps * per_frame)
        return self.n_stops * (float(cfg.settle_s) + per_frame)


def build_plan(cfg: RollScanConfig) -> RollSweepPlan:
    # the scan parks at home first, so the planned range is the span about
    # home -- keeping stop-and-go and continuous mode on the same geometry
    lo, hi = cfg.centred_span_deg(cfg.home_roll_deg)
    step = abs(float(cfg.step_deg))
    sweeps = max(int(cfg.sweeps), 1)
    if step < 1e-6 or hi <= lo:
        return RollSweepPlan(angles_deg=[lo], sweeps=sweeps, step_deg=step,
                             lo_deg=lo, hi_deg=hi)
    n = int(math.floor((hi - lo) / step + 1e-9)) + 1
    forward = [lo + i * step for i in range(n)]
    if forward[-1] < hi - 1e-9:
        forward.append(hi)
    angles: list[float] = []
    for s in range(sweeps):
        leg = list(forward) if (s % 2 == 0) else list(reversed(forward))
        if angles and abs(leg[0] - angles[-1]) < 1e-9:
            # a reversal would otherwise capture the turn angle twice back to
            # back: same pose, same view, no new information
            leg = leg[1:]
        angles.extend(leg)
    return RollSweepPlan(angles_deg=angles, sweeps=sweeps, step_deg=step,
                         lo_deg=lo, hi_deg=hi)
