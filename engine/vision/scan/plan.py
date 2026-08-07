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

    roll_min_deg: float = -90.0
    roll_max_deg: float = 90.0
    # keep a margin off the mechanical limit; the sweep is not trying to
    # exercise the joint stop, and pressing into it stalls the settle detector
    margin_deg: float = 3.0
    step_deg: float = 2.0
    sweeps: int = 2
    settle_s: float = 0.12
    settle_tol_deg: float = 0.35
    step_timeout_s: float = 2.0
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

    def span_deg(self) -> tuple[float, float]:
        """Usable sweep range after the limit margin."""
        lo = float(self.roll_min_deg) + float(self.margin_deg)
        hi = float(self.roll_max_deg) - float(self.margin_deg)
        if hi <= lo:
            mid = 0.5 * (float(self.roll_min_deg) + float(self.roll_max_deg))
            return mid, mid
        return lo, hi


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
        process_s = 0.15 * (px / 1.0e6)
        return self.n_stops * (float(cfg.settle_s) + grab_s + process_s)


def build_plan(cfg: RollScanConfig) -> RollSweepPlan:
    lo, hi = cfg.span_deg()
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
