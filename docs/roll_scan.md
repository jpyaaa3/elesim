# Roll-sweep geometry scan

Measures a cylindrical object's cross-section by sweeping the arm's roll joint,
fusing the ZED's point clouds across the sweep, and fitting a cylinder to the
result.

The distinguishing feature is where the camera poses come from: **forward
kinematics**, not the ZED's visual-inertial odometry. Everything else follows
from that choice.

---

## 1. Why this exists, and why FK

Fusing depth frames from several viewpoints needs to know where the camera was
for each one. There are two ways to get that:

- **VIO** (the ZED's own tracking). Works for a hand-held scan, because the
  camera translates through the scene and stays observable.
- **FK** — read the joint angles, run forward kinematics to the `node9` link,
  compose with the hand-eye extrinsics.

A wrist-mounted camera swept by a single joint barely translates, so VIO
degenerates exactly where this scan operates. FK also has a property VIO cannot
offer: given the joint angles, the poses are **reproducible offline**, so a
saved capture can be re-fused later and get bit-identical registration.

The interesting consequence: if FK is used as the pose source, then **the
quality of the fused cloud is a measurement of how good FK is.** That is the
point of the exercise, and it dictates one hard rule:

> **No ICP.** Registration is transform-and-merge only. Any alignment
> refinement would silently correct FK error and hide the quantity being
> measured.

The metric that reads FK quality is the **fused wall thickness**
(`residual_rms_mm`). A perfect pose source gives a wall as thin as the sensor
noise; pose error smears the same physical surface across several positions and
thickens it. In the self-test, injecting 1° of joint error plus 5 mm of link
error inflates the wall **4.9×**.

---

## 2. The roll unit convention — read this first

This trips everyone once. Roll has **two scales**, and they differ by a factor
of 2 *and* a sign.

| | one end | centre | other end |
|---|---|---|---|
| **u** (motor / control unit) | `0` | `180` | `360` |
| **q** (joint radians/degrees) | `+90°` | `0°` | `−90°` |

Set by `roll_u_min=0, roll_u_max=360` against
`roll_q_min_rad=−π/2, roll_q_max_rad=+π/2` in
[engine/core/protocol.py](../engine/core/protocol.py), with the sign flipped by
`command_direction`'s roll entry (`-1`).

**Every angle in `[roll_scan]` is in q degrees**, where `q = 0` is the middle of
the range. So:

- `step_deg = 4.0` means 4° of q, which is **8 u**
- `span_deg = 90.0` means q −45…+45, which is **u 90…270**
- `home_roll_deg = 0.0` is the centre, i.e. **u 180**

The UI plan line prints both (`step 4 deg q (8 u)`) so this cannot be misread
at the point of use.

---

## 3. What one run does

```
park roll at centre  →  anchor  →  traverse while capturing  →  fuse  →  fit  →  park at centre
```

1. **Park at centre** (`home_roll_deg`). A run always starts from a known,
   object-facing pose rather than wherever the last action left the joint.
2. **Anchor.** One frame is taken from that pose and run through the bench's
   `auto_roi` (nearest significant non-planar cluster) and `extract_points`; the
   median of the resulting object points becomes the centre of a `box_half` cube.
   *Every later frame is cropped to that box*, so the anchor decides what the
   scan is even looking at. If no object cluster is found it falls back to a
   whole-frame median and **says so** — that fallback anchors on background.
3. **Traverse.** Roll ramps across `span_deg` centred on the anchor angle, at
   `sweep_rate_deg_s`. Frames are grabbed continuously; one is **kept** each
   time the measured roll has advanced `step_deg` since the last kept frame.
4. **Fuse.** Kept frames are stacked and voxel-downsampled. No ICP.
5. **Fit.** The fused cloud goes to `zed_cylinder_bench.fit_cylinder`.
6. **Park at centre again** — inside a `finally`, so a failed, crashed or
   stopped run still ends at centre.

### Why capture is gated on angle, not time

The keep condition is *measured roll advanced by `step_deg`*, so angular
coverage is a property of the plan. A time-gated sweep bunches frames wherever
the joint happened to slow down.

### Why it captures while moving

The pose comes from the measured joint state either way, so a frame taken
mid-traverse registers just as well as one taken at rest — and stopping cost
about **0.3 s per stop in settle alone**. `continuous = false` restores
stop-and-go if you want it.

The cost of moving is pose–image time alignment: joint reads are 20 Hz, so at
12°/s each frame carries roughly **0.6° of angular ambiguity**. Frames whose
pose straddles more than `max_pose_straddle_deg` are dropped, because
registering a cloud at an angle the arm was never at is precisely the error the
scan is supposed to measure.

### Why the traverse is centred, not limit-to-limit

The joint reaches 90° of q either side of centre. A 30 cm-distant object spans
only ~385 mm of view (65° HFOV) while a full sweep moves the optical centre
~486 mm, and the object is normally in view only near the middle. A full-range
sweep therefore spends most of its travel looking somewhere else. `span_deg = 0`
restores the full range if you actually want it.

---

## 4. Running it

### From the UI

**Perception panel → Geometry Scan → `Scan Geometry`.**

Point the camera at the object *before* pressing — the anchor is taken from the
starting pose. The panel shows the plan, a progress bar, and on completion the
fitted diameter, arc coverage, wall rms and whether the wall was
exterior/interior.

The scan refuses to move the arm at all if the geometry backend or the camera is
missing, and says which. A sweep that cannot be fitted is ~20 s of joint travel
for nothing.

### Where it runs

Wherever perception runs. With `perception.provider = host` (the PC profile) the
UI sends `roll_scan_start` over ZMQ and the **host** owns the scan; with
`run_local = true` it runs in the ctrl process. The camera and the joint state
have to be in the same process, which is what decides this.

### Prerequisites

| | |
|---|---|
| `zed_cylinder_bench.py` | at the repo root, or pointed to by `ELESIM_CYLINDER_BENCH`. Supplies the fitting layer. |
| ZED on **USB 3.0** | the ZED Mini's video interface needs it. On USB 2.0 only its HID sub-device enumerates, `/dev/video*` never appears, and the SDK reports a bare `CAMERA STREAM FAILED TO START`. `probe_zed()` detects exactly this and names it. |
| `pyzed` | ZED SDK Python bindings. |

Measured on the Jetson: `cam.grab()` with `NEURAL` depth at HD1080 takes **222 ms**, on top of ~295 ms of point processing. `NEURAL_LIGHT` or `HD720` are the levers if a scan needs to be faster.

---

## 5. Configuration

All of `[roll_scan]` in [config.ini](../config.ini). Angles are **q degrees**.

### Geometry of the sweep

| key | default | meaning |
|---|---|---|
| `span_deg` | `90.0` | total q span, centred on the anchor angle. `0` = full joint range. |
| `home_roll_deg` | `0.0` | the angle treated as centre (q 0 = u 180). |
| `return_home` | `true` | park at centre when the run ends, including on failure. |
| `home_timeout_s` | `6.0` | bound on each parking move. |
| `step_deg` | `4.0` | keep a frame each time measured roll advances this much. |
| `sweeps` | `1` | back-and-forth passes. More passes average depth noise without ICP. |
| `margin_deg` | `3.0` | stay this far off the mechanical limit. |
| `roll_min_deg`/`roll_max_deg` | from `[model]` | joint limits; inherited so there is one place to widen them. |

### Motion and timing

| key | default | meaning |
|---|---|---|
| `continuous` | `true` | capture while traversing. `false` = stop at each angle. |
| `sweep_rate_deg_s` | `12.0` | traverse rate. Pick it so frame cadence lands near one frame per `step_deg`. |
| `command_hz` | `25.0` | how often the ramp is re-commanded. The capture loop spins much faster than the servo accepts; commanding every iteration replaced the pending target ~166 times a second and pushed `sync_write` to 650 ms. |
| `max_pose_straddle_deg` | `1.5` | drop a frame whose pose moved more than this during capture. |
| `max_stale_pose_s` | `0.15` | joint state older than this is flagged stale. |
| `settle_s`, `settle_tol_deg`, `step_timeout_s` | `0.12`, `0.35`, `2.0` | stop-and-go mode only. |

### Point processing

| key | default | meaning |
|---|---|---|
| `box_half` | `0.15` | **upper bound** on the crop half-size. The actual box is sized PER AXIS from the object detected at anchor time (an 11 cm object gets a ~15 cm box, not 30 cm). A fixed 0.15 pulled ~19 cm of background in per side. |
| `frame_voxel` / `fuse_voxel` | `0.002` | per-frame and fused voxel size. |
| `inlier_tol` | `0.005` | circle inlier tolerance. FK pose error shows up as wall thickness, so do not tighten this to hide it. |
| `min_depth` / `max_depth` | `0.15` / `1.5` | depth window. |
| `min_points_per_frame` | `200` | below this a frame is dropped. |

### Camera and output

`resolution` (`HD1080`), `depth_mode` (`NEURAL`), `fps` (`15`),
`confidence`/`texture_confidence`, `label`, `outdir`, `save_frames_npz`.

Resolution is the main performance lever: per-frame cost scales with pixel
count, and `HD720` is 2.2× cheaper than `HD1080`.

---

## 6. Reading the results

Outputs land in `outdir` (`engine/logs/roll_scan/`, gitignored):

| file | contents |
|---|---|
| `<label>_fused.ply` | fused world-frame cloud |
| `<label>_scan.json` | report + config + full fit |
| `<label>_cross_section.txt` | polygon string for `newarc.py --vertices` |
| `<label>_frames.npz` | raw per-frame clouds + camera positions, written **before** fitting so a bad fit never costs a re-scan |

Report fields worth understanding:

| field | how to read it |
|---|---|
| `fused.diameter_mm` | the answer. |
| `fused.arc_span_deg` | how much of the circumference the fused cloud covers. A single view cannot exceed ~180° of an exterior wall; the point of fusing is to beat that. |
| `fused.residual_rms_mm` | **the FK quality metric.** Compare against the single-view value: much thicker means pose error, not sensor noise. |
| `fused.surface` | `exterior` / `interior` / `mixed`, from per-point provenance. Grasping wants `exterior`. |
| `single_view_best` | the best single frame fitted the same way, so fused-vs-single is apples to apples. |
| `observation_span_deg` | angle subtended at the object by the camera positions — **the coverage that buys new surface.** |
| `view_span_deg` | spread of the optical axes. Often ~0 on this arm, see below. |
| `camera_travel_mm` | how far the optical centre actually moved. |

### `view_span_deg` is often 0, and that is not a bug

Roll here is a **base** roll (`j_housing_wedge`, axis X). When the arm is nearly
straight the optical axis lies along the roll axis, so rolling does not re-aim
the camera at all — it swings it sideways on a wide arc instead. So the optical
axes stay parallel (`view_span_deg ≈ 0`) while the camera travels ~620 mm and
sees the object from genuinely different positions (`observation_span_deg ≈ 119°`).

Both are reported so the distinction cannot be misread. **`observation_span_deg`
is the one that matters to fusion.**

---

## 7. Module map

```
engine/vision/scan/
  plan.py         RollScanConfig + stop-angle planning. No camera/FK/geometry
                  imports, so the config loader and UI can read it freely.
  fk_pose.py      FkPoseProvider: joint state -> world<-camera pose.
                  Also the coverage metrics and the visible-window calculation.
  zed_capture.py  ZedScanCamera (pyzed XYZ) + probe_zed() pre-flight.
  fusion.py       Pose-agnostic primitives: transform, depth gate, crop, plane
                  removal, voxel downsample, provenance, world-frame surface
                  classification.
  roll_sweep.py   RollSweepScan: the worker thread that owns the sequence.
  geometry.py     Adapter that resolves zed_cylinder_bench lazily.
  service.py      Binds a scan to the control stack; fit + report stage.
```

Collaborators are injected into `RollSweepScan` (pose provider, `read_q4`,
`command_roll`, `grab_points`), so the same object serves the live rig and the
synthetic self-test.

### Why the fitting layer is not implemented here

`geometry.py` resolves `zed_cylinder_bench` and re-exports it. That module was
validated against ground-truth-measured objects, so the scan runs *that* code
rather than a second implementation that could drift from it. Resolution is
lazy, so importing the package has no side effects on a machine without it.

One adaptation is required: `fit_cylinder`'s exterior/interior test assumes the
camera sits at the origin, which is false for a fused world cloud. The scan
therefore passes `prefer_exterior=False` and classifies the wall itself with
`classify_surface_world`, using per-point provenance — each point remembers
which camera position observed it.

### Integration points

| | |
|---|---|
| `ControlService.start_roll_scan` / `stop_roll_scan` | [engine/behaviors/pick/actions.py](../engine/behaviors/pick/actions.py). Delegates to the host when perception is remote, same pattern as gaze. |
| `roll_scan_start` / `roll_scan_stop` | [engine/behaviors/pick/client.py](../engine/behaviors/pick/client.py) → [host.py](../host.py) |
| status → UI | `PanelState.set_roll_scan_status`, broadcast via `pack_state(extra=...)`, mirrored into `HostState.roll_scan` |
| UI | Geometry Scan section in [ui/panels/perception.py](../ui/panels/perception.py) |

**`"roll_scan"` must stay in `host._is_allowed_source()`.** Rejected sources are
dropped on the first line of `_submit_direct_partial_control_u`, before the
arm-latency counters — so an omission there produces a perfect silence: no
motion, `recv=0`, and no log line at all.

---

## 8. Troubleshooting

The host prints phase changes and a 2 s heartbeat with commanded vs actual roll.

| symptom | cause |
|---|---|
| Button does nothing, UI shows nothing | Status precedence: when the scan is delegated the host is the only authority. If the UI is stale it can latch on its own optimistic "requested" flag. |
| `camera unavailable: only the ZED HID sub-device is enumerated` | ZED on USB 2.0. Move it to USB 3.0; `lsusb \| grep 2b03` should show **two** entries (`f680` video + `f681` HID) and `/dev/video*` should exist. |
| `geometry backend: MISSING` | `zed_cylinder_bench.py` not found. Repo root, or set `ELESIM_CYLINDER_BENCH`. |
| Arm does not move, `recv=0`, no `[host] direct target` line | `"roll_scan"` missing from `_is_allowed_source`. |
| `need >=2 usable frames, got N` | Read the `drops:` summary and the `[roll_scan] note:` lines. |

### Reading the drop notes

For the first few frames the scan logs the point count at every stage:

```
[roll_scan] note: anchor: valid=1481122 gated=592328 box=148342 clean=2304
```

- **`box` is small** → the object is not in the crop box. The anchor landed on
  the wrong thing, or `box_half` is too small.
- **the fused cloud fills the crop box exactly** (extent = `2 * box_half` in two
  axes) → the box is slicing a large surface, not enclosing an object. Check the
  `anchor via roi` note: if it says the whole-frame median was used, no object
  cluster was detected and the box is on background. Frames will also share ~0%
  of their points, which is the giveaway.
- **`box` is large but `clean` is tiny** → plane removal ate the object. Its
  wall was locally planar enough to win a plane consensus. Shrink `box_half` so
  the table is not in the box to begin with (the cheapest fix), or raise the
  `min_fraction` that `clean_world_frame` passes to `remove_dominant_plane_fast`
  so a plane needs a larger share of the points before it is removed.

These need opposite fixes, which is why the stages are logged separately.

### Quieting the console

`arm_latency_log_enable = false` in `[hardware]` silences the latency lines
entirely. Idle intervals are already suppressed, and streaming sources are
rate-limited to one `[host] direct target` line per second.

---

## 9. Tests

```bash
python3 tests/test_roll_scan.py       # readable staged output
python3 -m pytest tests/test_roll_scan.py
python3 zed_cylinder_bench.py --selftest   # the fitting layer alone
```

`tests/test_roll_scan.py` drives the **real** `RollSweepScan` against a
synthetic arm (roll tracks its command at a finite rate) and a synthetic camera
(renders the visible wall arc of a cylinder). No hardware. It covers:

- plan geometry, including that `span_deg` centres the plan
- voxel key packing (distant coordinates must not alias)
- FK poses: object-view span, camera travel
- reconstruction accuracy (+0.07 mm on a 90 mm cylinder)
- **pose-error sensitivity** — 1° + 5 mm inflates the wall 4.9×
- continuous sweep: frame count, coverage gaps, accuracy
- parking at centre, including that a *failed* run still parks
- end-to-end through the real fitter, on a cylinder standing on a table (the
  only check that exercises plane removal for real)

---

## 10. Known limitations

- **Pose–image alignment in continuous mode** is bounded by the 20 Hz joint read
  rate: ~0.6° of ambiguity per frame at 12°/s. `max_pose_straddle_deg` rejects
  the worst cases but does not remove the floor. Stop-and-go trades ~0.3 s per
  stop for a tighter bound.
- **The anchor is a single frame.** If the starting view is poor, the crop box is
  wrong for the whole run. There is no re-anchor mid-sweep yet;
  `reanchor_from_view_rays` exists in `fusion.py` for offline salvage but is not
  wired to a CLI.
- **One object per scan.** The crop box assumes a single target.
- **`newarc.py` integration is untested here** — that file is not in this repo.
  The cross-section polygon is exported for it, but the wrap-solve path has not
  been exercised.
- **Coverage depends on arm pose.** How much new surface a roll sweep buys
  depends on where the object sits relative to the roll axis. Check
  `observation_span_deg` in the report rather than assuming.
