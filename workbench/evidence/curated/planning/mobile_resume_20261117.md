# Mobile task: exact continuation checkpoint

Updated: 2026-11-17T13:59:43.981910+09:00
This file records execution state. `docs/status.md` still owns acceptance milestones.

## Resume here

**V31 walking/leveling/arrival correction is pushed and deployed on both installs.
The installed short probe passed. Full run 49 FINISHED with a separate arm-return
timeout; do not restart or repeat the long scan unchanged.**

- Actual runtime source: `9c1d1d73115ab03bc3da69d91cef4e7dc36df995`.
- Operator v31: `fa332f9e510eebbd10a32e02ee582986b27e5a725024914e08d5c15b6bd9e805`.
- Compute v31: `f3c02b2232d4a8cd8055ca00e37c1ab067efc82dd9ed880f1b632706e56fa06b`.
- Actual Pilot/Sim/UI image + package hashes and Enforce/GPU2 matched. Registration
  v31 completed atomically; manager shut down. Scoped test Pilot/Sim were
  stopped after the finished run; GPU2 returned to 0% / 188 MiB. Other work remained running.
- **Installed short arrival passed WITHOUT source override: 7.72 s / 2.26 mm
  settled combined error**, measured stand, zero command, stationary velocities.
  Raw `base-planted-v31.tar.gz`. Controller SHA:
  `44a4e925c4feec3b8905343d2f5f08592fdc7c052956c0170daea7966367a3bd`.

Full run 49 used actual installed UI/Pilot/Sim, DDS and both WebRTC streams.
It cleared the needed floor map (**unknown 424 → 0**, eight bounded views).
Then the certified return to the last target sighting used 3 shortened segments
(from 10), but exceeded the unchanged **180 s** search phase deadline.
Total report time **674.43 s**. It never reached scan/base/hug/hold in this run.
This is not a walking failure or evidence that full mobile grasp succeeded.

At timeout the angular axes were within endpoint tolerance; linear had **9.51 mm**
left. The linear axis still covered **43.85 mm over the last 30 s**, so do not
claim it was stuck or in a local minimum. Inspect the recorded trajectory/timing
before changing rates or tolerances. Next remaining bottleneck is the certified
arm-return duration. Do not rerun floor mapping merely to rediscover this.

Generated evidence root:
`workbench/evidence/generated/mobile-installed-20261115/`.
- `run-49/report.json`, `telemetry.jsonl`, `observer.mp4`, `hand_eye_preview.mp4`.
- Both videos finalized with **5,926 frames each**.
- `run-49-return-timing.json`, `run-49-remote-tail.log`.
- `official-v31-{remote,operator}.json`, `registration-v31.json`, `rotation-v31.json`.
- `operator-source-49-v31.json`, `runtime-down-v31.log`.

Both earlier short probes are FINISHED and fetched; never relaunch them. The v30
installed probe failed because gait restarts displaced the body after a transient
2.34 mm arrival. The planted-leg candidate passed in 7.11 s / 2.24 mm, then the
unchanged installed code passed as above. No precision/hold criterion was relaxed.
See `mobile_walk_level_live_20261117.json` for exact evidence hashes and outcomes.
Final focused Sim checks: **55 passed**. Installed two-process loopback DDS burst
passed **16/16**. Four-process smoke remains canonical-dev-only; Robot is correctly
absent from isolated Sim. Do not add sibling packages or rebuild dev as a detour.

## Objective and fixed scope

- Official installs: `/home/user/ws/newsim` and
  `hckang@100.74.222.24:/home/hckang/ws/newsim`.
- Fixed D100 Sim fixture `(0.7, 0.15, 0.55)`; Pilot uses sensed geometry.
- Required result remains observation → reconstruction → C-band positioning →
  hug/lift → independent same-execution 5 s loaded hold. It is **not achieved**.
- Keep existing 900 s task / 180 s phase / 90 s base deadlines, 0.5 s feedback,
  3 mm combined final arrival, friction and hold conditions. The user corrected
  the intermediate tilt criterion: 0.07 rad is a stationary arrival bound only;
  walking sway is recorded, not a task failure. Immutable old experiment files
  retain their old criteria for historical evidence.
- No training or physical Robot. Never modify host MTU/Tailscale/SSH/firewall or
  stop NewtonGym/unrelated work. Only the owned `test` instance is in scope.

## What already exists; do not rediscover or redo

- Runtime commits pushed: `8b2682b8` swept-grid speed; `55c6b296` bounded DDS
  ready-work drain; `0afd1a02` atomic base-velocity + Home hold.
- Last runtime source published/registered on both installs: **`9c1d1d73115ab03bc3da69d91cef4e7dc36df995`**.
- Operator release: `fa332f9e510eebbd10a32e02ee582986b27e5a725024914e08d5c15b6bd9e805`.
- Compute release: `f3c02b2232d4a8cd8055ca00e37c1ab067efc82dd9ed880f1b632706e56fa06b`.
- Registration/approved SROS2 rotation v31 completed atomically. Normal remote `test`
  runtime was stopped after completed run 49. GPU selection remains host GPU2; do not reapply index
  2 inside its single-visible-GPU container.
- Protocol real-RMW two-process loopback regression: old receive delivered 1/16;
  fixed receive delivered 16/16 in one pump. It used an explicit diagnostic
  source override, not a full installed acceptance result. Source hash:
  `dc231bc53f77c871edee078107c9c0b8e772296dc239322c484da8cb4197ce00`.
- Focused software checks: 370 passed, 5 skipped, 5 subtests passed. Base-probe
  commands separately match the real BaseExecutor in 5 parity cases.
- Full runs 45–48 and their videos/telemetry remain in the generated root below.
  Run 48 reached 2-view scan/AdaPoinTr, C-band plan and Home return, then failed
  feedback freshness before meaningful base motion. No wrap/hold acceptance.

## Historical failure evidence: isolated base plant

- Recorded goal offset: `(0.2249203741, 0.0090866415, ~0)`; same policy Home.
- Installed baseline controller was stopped by the misplaced **body_tilt** task
  guard after **4.168 s**. No fall or controller fault was observed.
  Maximum roll/pitch: `[0.0402549411234283, 0.07019917980996326]` rad.
  Combined final-position/yaw error at failure: **54.63 mm**.
- This isolates the physical Go2 controller from DDS/scan/map. It does not prove
  obstacle admission, network behavior, grasp or hold.
- Maintained runner: `workbench/research/debug/profile_mobile_base.py`, using
  `profile_sim_walking.py` and the production Genesis plant. No sibling role
  import is added to runtime. Its goal command is parity-tested against Pilot.

## Finished prototype: do not relaunch

The detached `elesim-merry-base-goal-prototype` container finished and was removed.
It failed **no_measured_progress** after **13.898 s**. Maximum roll/pitch was
`[0.0397596635, 0.0346419613]` rad; final combined arrival error **188.21 mm**.
All four feet were loaded, but the overly strict 0.02 rad liftoff gate kept
`posture_hold` active and the desired COM velocity zero. It was never deployed.

Remote old source/data:
`/home/hckang/ws/elesim-mobile-diagnostics/base-goal-20261117`.
`output/baseline.*` and `output/prototype.*` are complete old evidence. Their old
runner hash is `a090863433ebd1d7a46f09118907359a82ad4dbd79cd25f1d6bca392e61b76c0`;
prototype hash `d6f32eb40576652167d00355eea500c71eeca57ff97ab56ec6fd624d8656acda`.
The updated local runner/controller must not be confused with these old copies.

## Files that must survive

Generated evidence root:
`workbench/evidence/generated/mobile-installed-20261115/`.

- `base-goal-baseline/`: raw trajectory, profile, summary (fetched locally).
- `base-goal-baseline.log`, `mobile-base-case48.json`, `base-goal-staging.json`.
- `rotation-v29.json`, `registration-v29.json`, `update-*-v29.log`.
- `operator-source-49.json`: prepared manifest, **no run-49/report.json yet**.
- `dds-burst-real-before.log`, `dds-burst-real-after.log`: actual ROS regression.
- `base-goal-prototype/`, `base-goal-prototype.tar.gz`: fetched complete failure.
- Exact failed prototype bytes, patch and hashes are preserved in
  `workbench/archive/mobile-liftoff-prototype-20261117.zip` (ignored) and
  `mobile_liftoff_retired_20261117.json` (tracked inventory). Historical stance
  diagnostic evidence remains intact. No successful 225 mm arrival is claimed.

SSH uses the existing connection-local MSS proxy:
`-o 'ProxyCommand=python3 -m workbench.research.support.ssh_mss_proxy %h %p'`.
This does not change interface MTU. The known current-install dev wrapper exists,
but invoking it attempted a fresh image build; preparation was interrupted before
any tests ran. Canonical required/extended, release build/verify and real-RMW
`smoke_topology.py` remain unrun for this source revision. Host Sim lacks
pinocchio/torch; do not install them into host Python or invent a substitute
container. The focused host checks above are partial software evidence only.
