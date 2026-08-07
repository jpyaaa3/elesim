"""
FK camera pose provider -- the replacement for the ZED's VIO pose.

``camera_world_transform`` already composes the node9 link pose (from
``_forward_link_tf`` on the 4-DOF joint vector) with the hand-eye extrinsics,
which is exactly the world<-camera transform the fusion stage needs. This
module wraps it so the scan loop can ask for a pose by joint state and get
back the same (R, t) pair the VIO path used, plus the provenance needed to
audit a capture afterwards.

Frame conventions
-----------------
``T_world_camera`` maps points in the camera OPTICAL frame (+X right, +Y down,
+Z forward -- the ZED SDK's IMAGE coordinate system, and the convention
``hand_eye.camera.json`` is written in) into the arm's world frame. A ZED
``MEASURE.XYZRGBA`` point is already in that optical frame, so it needs no
extra axis permutation before being transformed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

import numpy as np

from engine.vision.perception_bridge.hand_eye import (
    camera_world_transform,
    load_hand_eye_transform,
)


@dataclass(frozen=True)
class FkPoseSample:
    """One camera pose derived from joint state."""

    R: np.ndarray  # 3x3, world <- camera optical
    t: np.ndarray  # 3, camera optical origin in world
    q4: np.ndarray  # the joint vector it came from
    roll_rad: float
    stale: bool = False  # joint state was older than the freshness budget
    age_s: float = 0.0

    @property
    def look(self) -> np.ndarray:
        """Optical +Z (view direction) in world."""
        return self.R[:, 2]

    @property
    def right(self) -> np.ndarray:
        """Optical +X in world."""
        return self.R[:, 0]

    @property
    def down(self) -> np.ndarray:
        """Optical +Y in world."""
        return self.R[:, 1]


class FkPoseProvider:
    """
    Turns (linear, roll, theta1, theta2) into a world<-camera pose.

    Deliberately stateless apart from the FK context and the extrinsics: the
    scan loop owns the joint state, so a replayed capture reproduces the exact
    poses of the live one (the VIO path could not do this -- its poses were
    unrecoverable once the session ended).
    """

    def __init__(
        self,
        *,
        ik_context: dict[str, Any],
        hand_eye_transform: Optional[np.ndarray] = None,
        hand_eye_path: Optional[str] = None,
        parent_frame: str = "node9",
    ) -> None:
        if not ik_context:
            raise ValueError("FkPoseProvider needs a populated ik_context")
        self._ctx = dict(ik_context)
        if hand_eye_transform is None:
            if not hand_eye_path:
                raise ValueError("provide hand_eye_transform or hand_eye_path")
            hand_eye_transform, meta = load_hand_eye_transform(hand_eye_path)
            parent_frame = str(meta.get("parent_frame", parent_frame))
        self._T_parent_camera = np.asarray(hand_eye_transform, dtype=float).reshape(4, 4).copy()
        self._parent_frame = str(parent_frame)

    @property
    def parent_frame(self) -> str:
        return self._parent_frame

    @property
    def hand_eye_transform(self) -> np.ndarray:
        return self._T_parent_camera.copy()

    def transform(self, q4: Sequence[float]) -> np.ndarray:
        """world<-camera 4x4 for a joint vector."""
        return camera_world_transform(
            self._ctx,
            np.asarray(q4, dtype=float).reshape(4),
            self._T_parent_camera,
            parent_frame=self._parent_frame,
        )

    def sample(
        self,
        q4: Sequence[float],
        *,
        stale: bool = False,
        age_s: float = 0.0,
    ) -> FkPoseSample:
        q = np.asarray(q4, dtype=float).reshape(4)
        T = self.transform(q)
        return FkPoseSample(
            R=np.ascontiguousarray(T[:3, :3]),
            t=np.ascontiguousarray(T[:3, 3]),
            q4=q.copy(),
            roll_rad=float(q[1]),
            stale=bool(stale),
            age_s=float(age_s),
        )

    def visible_roll_window_deg(
        self,
        q4: Sequence[float],
        target_world: np.ndarray,
        *,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        width: int,
        height: int,
        lo_deg: float,
        hi_deg: float,
        margin_frac: float = 0.15,
        samples: int = 181,
    ) -> tuple[float, float]:
        """
        The sub-range of roll over which ``target_world`` stays inside the image.

        This arm rolls about its BASE, and the optical axis lies close to that
        axis, so rolling translates the camera on a wide arc without re-aiming
        it: the target slides across the frame and leaves it. Concretely, a 174
        deg sweep moves the optical centre ~486 mm while a 65 deg HFOV spans only
        ~385 mm at 0.3 m, so most of the joint's range points nowhere near the
        object. Sweeping it anyway costs traverse time and yields nothing.

        Returns the contiguous window around the best angle, shrunk by
        ``margin_frac`` of the image so the target is not clipped at the border.
        Falls back to the full range if the target is never projected inside.
        """
        q = np.asarray(q4, dtype=float).reshape(4).copy()
        tw = np.asarray(target_world, dtype=float).reshape(3)
        mx, my = float(width) * margin_frac, float(height) * margin_frac
        angles = np.linspace(float(lo_deg), float(hi_deg), int(samples))
        inside = np.zeros(len(angles), dtype=bool)
        for i, deg in enumerate(angles):
            q[1] = np.radians(float(deg))
            try:
                T = self.transform(q)
            except Exception:  # noqa: BLE001
                continue
            pc = (T[:3, :3].T) @ (tw - T[:3, 3])
            if pc[2] <= 1e-3:
                continue
            u = float(fx) * pc[0] / pc[2] + float(cx)
            v = float(fy) * pc[1] / pc[2] + float(cy)
            inside[i] = (mx <= u <= float(width) - mx) and (my <= v <= float(height) - my)

        if not inside.any():
            return float(lo_deg), float(hi_deg)
        # longest contiguous run of visible angles
        best_len = best_start = run_start = 0
        run = 0
        for i, ok in enumerate(inside):
            if ok:
                if run == 0:
                    run_start = i
                run += 1
                if run > best_len:
                    best_len, best_start = run, run_start
            else:
                run = 0
        return float(angles[best_start]), float(angles[best_start + best_len - 1])

    def baseline_span_m(self, samples: Sequence[FkPoseSample]) -> float:
        """
        How far the optical centre actually travelled across a capture.

        Reported because it is the honest bound on what fusion can add: a roll
        sweep moves the camera on a short arc, so a near-zero span means the
        views are nearly co-located and extra frames buy angular coverage of
        the object's surface, not triangulation baseline.
        """
        if len(samples) < 2:
            return 0.0
        P = np.array([s.t for s in samples], dtype=float)
        return float(np.linalg.norm(P.max(axis=0) - P.min(axis=0)))

    def view_span_deg(self, samples: Sequence[FkPoseSample]) -> float:
        """
        Angular spread of the optical AXES.

        Note this is often ~0 on this arm: the roll joint is a base roll about
        X, so when the arm is nearly straight the optical axis lies along the
        roll axis and rolling does not re-aim the camera at all -- it swings the
        camera sideways instead. Use ``observation_span_deg`` for the coverage
        that actually matters to fusion.
        """
        if len(samples) < 2:
            return 0.0
        L = np.array([s.look for s in samples], dtype=float)
        L = L / np.maximum(np.linalg.norm(L, axis=1, keepdims=True), 1e-12)
        G = np.clip(L @ L.T, -1.0, 1.0)
        return float(np.degrees(np.arccos(G.min())))

    def observation_span_deg(
        self, samples: Sequence[FkPoseSample], target: np.ndarray
    ) -> float:
        """
        Angle subtended at the object by the camera positions.

        This is the coverage that buys new surface: two views 60 deg apart
        around the object see different walls even if their optical axes are
        parallel. On a base-roll arm this is the large number and
        ``view_span_deg`` is the small one, which is why both are reported.
        """
        if len(samples) < 2:
            return 0.0
        P = np.array([s.t for s in samples], dtype=float) - np.asarray(
            target, dtype=float
        ).reshape(1, 3)
        n = np.linalg.norm(P, axis=1, keepdims=True)
        keep = (n > 1e-6).reshape(-1)
        if int(keep.sum()) < 2:
            return 0.0
        D = P[keep] / n[keep]
        G = np.clip(D @ D.T, -1.0, 1.0)
        return float(np.degrees(np.arccos(G.min())))
