"""
Roll-sweep scan self-test: no ZED, no arm, no geometry backend.

Drives the real ``RollSweepScan`` against a synthetic arm (a roll joint that
tracks its command with lag) and a synthetic camera (renders the visible wall
arc of a cylinder into the camera frame). What is under test is the part that
replaced the VIO path:

  * FK poses actually place the camera on the arm and rotate with roll
  * angle-triggered capture yields the planned angular coverage
  * transform-and-merge fusion reconstructs the cylinder from FK poses alone
  * injected FK error inflates the fused wall thickness -- the misregistration
    signal the FK-as-metrology ablation reads

Plane removal is monkeypatched to identity so the test does not need
``zed_cylinder_bench``; the synthetic scene has no table to remove.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engine.vision.scan import fusion as fusion_mod
from engine.vision.scan.fk_pose import FkPoseProvider
from engine.vision.scan.fusion import fuse, transform_points, voxel_downsample
from engine.vision.scan.plan import RollScanConfig, build_plan
from engine.vision.scan.roll_sweep import RollSweepScan

R_TRUE = 0.045
CYL_CENTER = np.array([0.55, 0.0, 0.30])
CYL_AXIS = np.array([0.0, 0.0, 1.0])


def _identity_plane_removal(pts, min_fraction=0.20):  # noqa: ARG001
    return pts


class FakeArm:
    """Roll joint that chases its command at a finite rate; others stay put."""

    def __init__(self, rate_deg_s: float = 60.0) -> None:
        self.q = [0.10, 0.0, 0.20, -0.20]
        self._target = 0.0
        self._rate = math.radians(float(rate_deg_s))
        self._t_last = time.time()

    def command_roll_deg(self, deg: float) -> None:
        self._target = math.radians(float(deg))

    def read(self):
        # rate-limited tracking, so a continuous traverse cannot teleport and the
        # settle detector in stop-and-go mode has something real to wait for
        now = time.time()
        dt = max(0.0, now - self._t_last)
        self._t_last = now
        err = self._target - self.q[1]
        step = self._rate * dt
        self.q[1] += math.copysign(min(abs(err), step), err) if step > 0 else 0.0
        return tuple(self.q), 0.0


class FakeCylinderCamera:
    """Renders the front-facing wall arc of a cylinder into the camera frame."""

    def __init__(self, pose_provider, arm, rng, noise_m: float = 0.0008, n: int = 900) -> None:
        self._pose = pose_provider
        self._arm = arm
        self._rng = rng
        self._noise = float(noise_m)
        self._n = int(n)

    def grab_points(self):
        T = self._pose.transform(self._arm.q)
        R, t = T[:3, :3], T[:3, 3]
        to_cam = t - CYL_CENTER
        phi0 = math.atan2(to_cam[1], to_cam[0])
        phi = phi0 + self._rng.uniform(-1.0, 1.0, self._n)
        z = self._rng.uniform(-0.05, 0.05, self._n)
        pw = (
            CYL_CENTER.reshape(1, 3)
            + np.stack([R_TRUE * np.cos(phi), R_TRUE * np.sin(phi), z], axis=1)
            + self._rng.normal(0.0, self._noise, (self._n, 3))
        )
        return (pw - t.reshape(1, 3)) @ R  # world -> camera


def _fit_radius(pts: np.ndarray) -> tuple[float, float]:
    """Least-squares circle in the XY plane; returns (radius, residual rms)."""
    xy = np.asarray(pts, dtype=float)[:, :2]
    A = np.c_[2 * xy, np.ones(len(xy))]
    sol, *_ = np.linalg.lstsq(A, (xy ** 2).sum(1), rcond=None)
    cx, cy = sol[0], sol[1]
    r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))
    res = np.abs(np.linalg.norm(xy - [cx, cy], axis=1) - r)
    return r, float(np.sqrt((res ** 2).mean()))


def _ik_context():
    from engine.robot.arm import ik as ik_pipeline

    _bundle, ctx = ik_pipeline.load_solver_context("config.ini")
    return dict(ctx or {})


def _pose_provider(ctx):
    return FkPoseProvider(
        ik_context=ctx,
        hand_eye_path="model_presets/visual_servoing/hand_eye.camera.json",
    )


def check_plan_coverage() -> list[str]:
    fails = []
    # span_deg=0 -> plan the full joint range, which is what this check is about
    cfg = RollScanConfig(step_deg=5.0, sweeps=2, margin_deg=3.0, span_deg=0.0)
    p = build_plan(cfg)
    lo, hi = cfg.limit_span_deg()
    if abs(lo + 87.0) > 1e-9 or abs(hi - 87.0) > 1e-9:
        fails.append(f"span {lo}..{hi} != -87..87")
    if p.angles_deg[0] != lo or abs(max(p.angles_deg) - hi) > 1e-9:
        fails.append("plan does not reach both limits")
    turns = [
        i for i in range(1, len(p.angles_deg))
        if abs(p.angles_deg[i] - p.angles_deg[i - 1]) < 1e-9
    ]
    if turns:
        fails.append(f"plan repeats an angle back to back at {turns}")
    steps = np.diff(p.angles_deg)
    if np.abs(np.abs(steps) - 5.0).max() > 2.01:  # the endpoint stub is shorter
        fails.append(f"irregular steps: max deviation {np.abs(np.abs(steps)-5.0).max():.2f}")
    if p.sweeps != 2 or p.n_stops < 60:
        fails.append(f"unexpected plan size {p.n_stops}")
    centred = build_plan(RollScanConfig(step_deg=5.0, sweeps=1, span_deg=90.0,
                                       home_roll_deg=0.0))
    if abs(centred.lo_deg + 45.0) > 1e-6 or abs(centred.hi_deg - 45.0) > 1e-6:
        fails.append(f"centred plan is {centred.lo_deg:+.1f}..{centred.hi_deg:+.1f}, want -45..+45")
    print(f"  {'ok ' if not fails else 'BAD'} plan: {p.n_stops} stops over "
          f"{lo:+.0f}..{hi:+.0f} deg full range, no duplicate turn angle; "
          f"span_deg=90 plans {centred.lo_deg:+.0f}..{centred.hi_deg:+.0f}")
    return fails


def check_fk_pose_moves_with_roll() -> list[str]:
    fails = []
    ctx = _ik_context()
    pose = _pose_provider(ctx)
    samples = [
        pose.sample((0.10, math.radians(d), 0.20, -0.20))
        for d in np.linspace(-87.0, 87.0, 25)
    ]
    looks = np.array([s.look for s in samples])
    if np.abs(np.linalg.norm(looks, axis=1) - 1.0).max() > 1e-9:
        fails.append("optical axis is not unit length")

    axis_span = pose.view_span_deg(samples)
    obs_span = pose.observation_span_deg(samples, CYL_CENTER)
    travel = pose.baseline_span_m(samples) * 1000.0

    # this arm's roll is a BASE roll about X (j_housing_wedge), so with the arm
    # nearly straight the optical axis lies along the roll axis and barely
    # re-aims; the sweep instead swings the camera on a wide arc. What must be
    # large is the angle subtended at the object, which is what fusion consumes.
    if obs_span < 45.0:
        fails.append(f"roll sweep gives only {obs_span:.1f} deg of object coverage")
    if travel < 50.0:
        fails.append(f"camera barely moves: {travel:.1f} mm")
    if not (0.0 <= axis_span <= 180.0):
        fails.append(f"axis span out of range: {axis_span}")
    print(f"  {'ok ' if not fails else 'BAD'} FK pose: object-view span {obs_span:.1f} deg, "
          f"optical-axis span {axis_span:.1f} deg, camera travel {travel:.1f} mm")
    return fails


def check_scan_reconstructs_cylinder(monkeypatched: bool = True) -> list[str]:
    fails = []
    ctx = _ik_context()
    pose = _pose_provider(ctx)
    arm = FakeArm()
    cam = FakeCylinderCamera(pose, arm, np.random.default_rng(3))
    cfg = RollScanConfig(step_deg=6.0, sweeps=1, continuous=False, settle_s=0.0,
                         step_timeout_s=0.5, box_half=0.20, min_points_per_frame=100,
                         span_deg=0.0)   # full range: this check is about reconstruction

    scan = RollSweepScan(
        cfg=cfg,
        pose_provider=pose,
        read_q4=arm.read,
        command_roll=arm.command_roll_deg,
        grab_points=cam.grab_points,
    )
    scan.start()
    if not scan.wait(timeout_s=120.0):
        fails.append("scan thread did not finish")
        return fails
    res = scan.result()
    prog = scan.progress()
    if res is None:
        fails.append(f"no result: {prog.phase} / {prog.msg}")
        return fails

    r_fit, rms = _fit_radius(res.fused)
    err_mm = (r_fit - R_TRUE) * 2000.0
    if abs(err_mm) > 3.0:
        fails.append(f"diameter error {err_mm:+.2f} mm")
    if rms * 1000.0 > 3.0:
        fails.append(f"wall rms {rms*1000:.2f} mm too thick for clean FK poses")
    if res.n_frames < 20:
        fails.append(f"only {res.n_frames} frames kept")
    if res.roll_span_deg < 150.0:
        fails.append(f"roll span only {res.roll_span_deg:.0f} deg")
    if res.observation_span_deg < 45.0:
        fails.append(f"object-view span only {res.observation_span_deg:.0f} deg")
    print(f"  {'ok ' if not fails else 'BAD'} scan: {res.n_frames} frames, "
          f"roll span {res.roll_span_deg:.0f} deg, object-view span "
          f"{res.observation_span_deg:.0f} deg, camera travel "
          f"{res.baseline_span_m*1000:.0f} mm, d={2*r_fit*1000:.1f} mm "
          f"(err {err_mm:+.2f} mm), wall rms {rms*1000:.2f} mm")
    return fails, rms


def check_pose_error_inflates_wall(clean_rms: float) -> list[str]:
    """The metric the ablation reads: bad poses must show up as a thicker wall."""
    fails = []
    ctx = _ik_context()
    pose = _pose_provider(ctx)
    rng = np.random.default_rng(11)
    frames = []
    for deg in np.arange(-87.0, 88.0, 6.0):
        q = (0.10, math.radians(float(deg)), 0.20, -0.20)
        T = pose.transform(q)
        R, t = T[:3, :3], T[:3, 3]
        arm = FakeArm()
        arm.q = list(q)
        cam = FakeCylinderCamera(pose, arm, rng)
        pts_cam = cam.grab_points()
        # 1 deg of joint-angle error + 5 mm of link error, i.e. what a miscalibrated
        # FK chain would contribute
        ang = math.radians(1.0)
        Rz = np.array([[math.cos(ang), -math.sin(ang), 0], [math.sin(ang), math.cos(ang), 0], [0, 0, 1]])
        frames.append(transform_points(pts_cam, R @ Rz, t + rng.normal(0.0, 0.005, 3)))
    fused = fuse(frames, 0.002)
    _r, rms = _fit_radius(fused)
    ratio = rms / max(clean_rms, 1e-9)
    if ratio < 2.0:
        fails.append(f"pose error barely visible: {ratio:.1f}x clean wall rms")
    print(f"  {'ok ' if not fails else 'BAD'} pose-error sensitivity: wall rms "
          f"{rms*1000:.2f} mm = {ratio:.1f}x the clean scan -- misregistration is measurable")
    return fails


def check_voxel_downsample_no_key_collision() -> list[str]:
    """Coordinates far from the origin must not alias into the same voxel."""
    fails = []
    pts = np.array([[0.0, 0.0, 0.0], [12.0, -9.0, 4.0], [-31.0, 27.5, -18.25]])
    out = voxel_downsample(pts, 0.002)
    if len(out) != 3:
        fails.append(f"3 distant points collapsed to {len(out)}")
    dense = np.repeat(np.array([[5.0, 5.0, 5.0]]), 50, axis=0)
    if len(voxel_downsample(dense, 0.01)) != 1:
        fails.append("co-located points did not collapse to one voxel")
    print(f"  {'ok ' if not fails else 'BAD'} voxel downsample: distant keys distinct, "
          f"duplicates merged")
    return fails


def check_continuous_sweep() -> list[str]:
    """
    Continuous mode must cover the same roll range as stop-and-go, without
    stopping -- that is the whole reason it exists (~0.3 s per stop saved).
    """
    fails: list[str] = []
    ctx = _ik_context()
    pose = _pose_provider(ctx)
    arm = FakeArm(rate_deg_s=90.0)
    cam = FakeCylinderCamera(pose, arm, np.random.default_rng(9), n=500)
    cfg = RollScanConfig(
        step_deg=8.0, sweeps=1, continuous=True, sweep_rate_deg_s=180.0,
        settle_s=0.0, step_timeout_s=0.5, box_half=0.20,
        min_points_per_frame=50, max_pose_straddle_deg=90.0,
        span_deg=0.0,   # full joint range, so the span assertion below is about
                        # the traverse itself and not about span_deg
    )
    scan = RollSweepScan(
        cfg=cfg, pose_provider=pose, read_q4=arm.read,
        command_roll=arm.command_roll_deg, grab_points=cam.grab_points,
    )
    t0 = time.time()
    scan.start()
    if not scan.wait(timeout_s=180.0):
        return ["continuous scan did not finish"]
    elapsed = time.time() - t0
    res = scan.result()
    if res is None:
        return [f"continuous scan produced no result: {scan.progress().msg}"]
    rolls = scan.roll_angles_deg()
    if res.n_frames < 8:
        fails.append(f"only {res.n_frames} frames kept in continuous mode")
    lo, hi = cfg.limit_span_deg()
    want = 0.7 * (hi - lo)
    if res.roll_span_deg < want:
        fails.append(
            f"continuous roll span {res.roll_span_deg:.0f} deg covers less than "
            f"70% of the {hi - lo:.0f} deg range"
        )
    gaps = np.abs(np.diff(sorted(rolls))) if len(rolls) > 1 else np.array([0.0])
    if gaps.max() > 6 * cfg.step_deg:
        fails.append(f"coverage gap of {gaps.max():.1f} deg (step is {cfg.step_deg})")
    r_fit, rms = _fit_radius(res.fused)
    err_mm = (r_fit - R_TRUE) * 2000.0
    if abs(err_mm) > 6.0:
        fails.append(f"continuous diameter error {err_mm:+.2f} mm")
    print(f"  {'ok ' if not fails else 'BAD'} continuous sweep: {res.n_frames} frames in "
          f"{elapsed:.1f} s, roll span {res.roll_span_deg:.0f} deg, max gap "
          f"{gaps.max():.1f} deg, d={2*r_fit*1000:.1f} mm (err {err_mm:+.2f} mm)")
    return fails


def check_span_is_centred_on_anchor() -> list[str]:
    """
    span_deg must centre the traverse on where the anchor was taken.

    The live failure was a full-range sweep that began at a joint limit -- 90 deg
    of q away from centre, where the object was not in view -- and kept exactly
    one frame. Centring is what makes the traverse spend its travel where the
    object actually is.
    """
    fails: list[str] = []
    cfg = RollScanConfig(span_deg=40.0, margin_deg=3.0)
    for centre in (0.0, -60.0, 60.0, -200.0):
        lo, hi = cfg.centred_span_deg(centre)
        lim_lo, lim_hi = cfg.limit_span_deg()
        if abs((hi - lo) - 40.0) > 1e-6:
            fails.append(f"centre {centre}: span {hi - lo:.1f} != 40")
        if lo < lim_lo - 1e-9 or hi > lim_hi + 1e-9:
            fails.append(f"centre {centre}: {lo:.1f}..{hi:.1f} escapes the joint limits")
        want = min(max(centre, lim_lo + 20.0), lim_hi - 20.0)
        if abs(0.5 * (lo + hi) - want) > 1e-6:
            fails.append(f"centre {centre}: midpoint {0.5*(lo+hi):.1f} != {want:.1f}")
    full = RollScanConfig(span_deg=0.0)
    if full.centred_span_deg(30.0) != full.limit_span_deg():
        fails.append("span_deg=0 should mean the full joint range")
    print(f"  {'ok ' if not fails else 'BAD'} span centring: 40 deg window tracks the "
          f"anchor and clamps at the limits; span_deg=0 keeps the full range")
    return fails


def check_parks_at_centre() -> list[str]:
    """
    The scan must park roll at centre before sweeping and return there after --
    including when it fails, so a bad run never leaves the joint at an extreme.
    """
    fails: list[str] = []
    ctx = _ik_context()
    pose = _pose_provider(ctx)

    # start the arm far from centre so a missing pre-scan homing move is visible
    arm = FakeArm(rate_deg_s=400.0)
    arm.q[1] = math.radians(-80.0)
    cam = FakeCylinderCamera(pose, arm, np.random.default_rng(4), n=400)
    cfg = RollScanConfig(
        step_deg=10.0, sweeps=1, continuous=True, sweep_rate_deg_s=300.0,
        span_deg=40.0, home_roll_deg=0.0, return_home=True, home_timeout_s=4.0,
        settle_s=0.0, step_timeout_s=0.4, box_half=0.20,
        min_points_per_frame=50, max_pose_straddle_deg=90.0,
    )
    seen_phases: list[str] = []
    scan = RollSweepScan(
        cfg=cfg, pose_provider=pose, read_q4=arm.read,
        command_roll=arm.command_roll_deg, grab_points=cam.grab_points,
        on_progress=lambda p: seen_phases.append(p.phase),
    )
    scan.start()
    if not scan.wait(timeout_s=180.0):
        return ["park test: scan did not finish"]
    end_deg = math.degrees(arm.q[1])
    if "homing" not in seen_phases:
        fails.append("no homing phase was reported before the sweep")
    if abs(end_deg) > cfg.settle_tol_deg * 3:
        fails.append(f"roll left at {end_deg:+.1f} deg instead of centre")
    rolls = scan.roll_angles_deg()
    if rolls and (min(rolls) < -25.0 or max(rolls) > 25.0):
        fails.append(f"sweep left the +-20 deg window: {min(rolls):+.1f}..{max(rolls):+.1f}")

    # and on failure: no camera at all, so the sweep cannot keep a single frame
    arm2 = FakeArm(rate_deg_s=400.0)
    arm2.q[1] = math.radians(70.0)
    scan2 = RollSweepScan(
        cfg=cfg, pose_provider=pose, read_q4=arm2.read,
        command_roll=arm2.command_roll_deg, grab_points=lambda: None,
    )
    scan2.start()
    scan2.wait(timeout_s=180.0)
    end2 = math.degrees(arm2.q[1])
    if scan2.progress().phase != "failed":
        fails.append(f"expected a failed scan, got {scan2.progress().phase}")
    if abs(end2) > cfg.settle_tol_deg * 3:
        fails.append(f"failed scan left roll at {end2:+.1f} deg instead of centre")
    print(f"  {'ok ' if not fails else 'BAD'} parks at centre: started -80 deg -> ended "
          f"{end_deg:+.2f} deg; failed run started +70 deg -> ended {end2:+.2f} deg")
    return fails


def check_end_to_end_with_real_fitter() -> list[str]:
    """
    Full pipeline against the real fitting backend: cylinder standing on a table,
    swept by roll, fused from FK poses, then fitted and reported.

    This is the only check that exercises ``remove_dominant_plane`` for real, so
    it is also the one that proves per-frame plane removal keeps the table out of
    the fused wall.
    """
    from engine.vision.scan import geometry
    from engine.vision.scan.service import fit_and_report

    fails: list[str] = []
    if not geometry.available():
        print(f"  --  end-to-end skipped: {geometry.status()}")
        return fails

    # real plane removal for this check
    fusion_mod.remove_dominant_plane = geometry.remove_dominant_plane

    ctx = _ik_context()
    pose = _pose_provider(ctx)
    arm = FakeArm()
    rng = np.random.default_rng(5)

    class TableCylinderCamera(FakeCylinderCamera):
        """Cylinder wall plus a table plane through its base."""

        def grab_points(self):
            obj = super().grab_points()
            T = self._pose.transform(self._arm.q)
            R, t = T[:3, :3], T[:3, 3]
            # table: a patch of the z = base plane, in world, then to camera
            n = 1400
            xy = self._rng.uniform(-0.13, 0.13, (n, 2))
            table_w = np.stack(
                [CYL_CENTER[0] + xy[:, 0], CYL_CENTER[1] + xy[:, 1],
                 np.full(n, CYL_CENTER[2] - 0.05)], axis=1
            ) + self._rng.normal(0.0, 0.0006, (n, 3))
            table_c = (table_w - t.reshape(1, 3)) @ R
            return np.vstack([obj, table_c])

    cam = TableCylinderCamera(pose, arm, rng)
    cfg = RollScanConfig(
        step_deg=6.0, sweeps=1, continuous=False, settle_s=0.0, span_deg=0.0,
        step_timeout_s=0.5, box_half=0.20, min_points_per_frame=100, inlier_tol=0.004,
        label="selftest_e2e",
        outdir=str(Path(__file__).resolve().parents[1] / "engine" / "logs" / "roll_scan_selftest"),
        save_frames_npz=False,
    )
    scan = RollSweepScan(
        cfg=cfg, pose_provider=pose, read_q4=arm.read,
        command_roll=arm.command_roll_deg, grab_points=cam.grab_points,
    )
    scan.start()
    if not scan.wait(timeout_s=300.0):
        return ["end-to-end scan did not finish"]
    res = scan.result()
    if res is None:
        return [f"end-to-end scan produced no result: {scan.progress().msg}"]

    try:
        report = fit_and_report(
            res, cfg, frames=scan.frames(), cam_positions=scan.cam_positions(),
            gt_diameter_m=2 * R_TRUE, log=lambda _m: None,
        )
    except Exception as exc:  # noqa: BLE001
        return [f"fit_and_report raised: {exc}"]

    fused = report["fused"]
    err = report.get("fused_err_mm")
    if err is None or abs(float(err)) > 5.0:
        fails.append(f"fused diameter error {err} mm")
    if fused.get("surface") != "exterior":
        fails.append(f"surface classified as {fused.get('surface')}, expected exterior")
    if report["pose_source"] != "fk":
        fails.append("report does not declare the FK pose source")
    for key in ("fused_ply", "report_json"):
        pth = (report.get("outputs") or {}).get(key)
        if not pth or not Path(pth).is_file():
            fails.append(f"missing output {key}: {pth}")
    print(f"  {'ok ' if not fails else 'BAD'} end-to-end (real fitter): "
          f"d={fused.get('diameter_mm')} mm (true {2*R_TRUE*1000:.1f}, err {err} mm), "
          f"arc {fused.get('arc_span_deg')} deg, rms {fused.get('residual_rms_mm')} mm, "
          f"surface={fused.get('surface')}, single-view "
          f"d={(report.get('single_view_best') or {}).get('diameter_mm')} mm")

    # restore the identity stub for any later check
    fusion_mod.remove_dominant_plane = _identity_plane_removal
    return fails


def test_roll_scan_selftest() -> None:
    """Single pytest entry point; the staged checks run in order inside main()."""
    assert main() == 0


def main() -> int:
    # the synthetic scene has no table; skip the bench-backed plane removal so the
    # test runs without zed_cylinder_bench
    fusion_mod.remove_dominant_plane = _identity_plane_removal

    print("roll-sweep scan self-test (synthetic arm + camera, FK poses)")
    failures: list[str] = []
    failures += check_plan_coverage()
    failures += check_voxel_downsample_no_key_collision()
    failures += check_fk_pose_moves_with_roll()
    scan_fails, clean_rms = check_scan_reconstructs_cylinder()
    failures += scan_fails
    if not scan_fails:
        failures += check_pose_error_inflates_wall(clean_rms)
    failures += check_span_is_centred_on_anchor()
    failures += check_continuous_sweep()
    failures += check_parks_at_centre()
    failures += check_end_to_end_with_real_fitter()

    print()
    if failures:
        for f in failures:
            print(f"  FAIL: {f}")
        print(f"self-test: {len(failures)} FAILURE(S)")
        return 1
    print("self-test: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
