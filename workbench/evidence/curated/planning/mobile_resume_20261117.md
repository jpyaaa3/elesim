# Mobile task: exact continuation checkpoint

Updated: 2026-11-17T10:27:04.156947+09:00
This file records execution state. `docs/status.md` still owns acceptance milestones.

## Resume here

**The requested source correction is implemented and host-tested, but not committed,
pushed or deployed. Do not relaunch the failed prototype. No experiment is running.**
Run 49 has not started. The user asked for code first, after clarifying that normal
walking sway must not be judged against stationary grasp precision.

Current flow: normal MPC walking → zero command near the goal → planted-foot leg
position control levels the body → measured `stand` + stationary settling → final
position/yaw validation. Any remaining correction gets another swept-path check.
The former 0.07 rad tilt guard applies only to stationary arrival readiness; it is
no longer an unconditional moving-body task failure. The failed 0.02 rad precision
liftoff / recovery-gravity prototype was archived and removed from active source.

Host software evidence: **402 passed, 5 skipped, 5 subtests passed** (Pilot/research
139, focused Sim 51, protocol 212). Source hashes, exact logs and omitted canonical
checks: `mobile_walk_level_20261117.json`. No physical arrival or grasp success is
claimed. Next actual-motion check, when requested, is the short base-goal test;
do not repeat the long scan first. Its maintained runner now uses the actual
Pilot `BaseExecutor` instead of duplicating its control law. Mount maintained
Pilot app source read-only using `--pilot-app`; this is diagnostic, not release
acceptance. No new probe output or server source staging has been made yet.

The runner refuses existing output prefixes and creates an exclusive `.run.json`
marker. Use a new explicit experiment identity for a changed implementation; do
not overwrite old failed results or restart merely because a tool session expired.

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
- Last runtime source published/registered on both installs: **`0afd1a0248357a453ff806d0a6088a28c0e79855`**.
- Operator release: `13cb3ee299d4904e3eb4094395515432deb66379a910e625fcf2246a40d5f993`.
- Compute release: `cdbbde8b6fbaeae57bb2e05749bf5001b763ead50c5086871289967e638390e4`.
- Registration/approved SROS2 rotation v29 completed atomically. Normal `test`
  runtime remains stopped. GPU selection remains host GPU2; do not reapply index
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
