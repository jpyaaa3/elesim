# Mobile task: exact continuation checkpoint

Updated: 2026-11-17T11:05:30.627422+09:00
This file records execution state. `docs/status.md` still owns acceptance milestones.

## Resume here

**Runtime `58c526d8` was pushed and officially updated/registered on both hosts
(v30). The normal test runtime is stopped. Full run 49 has not started.**

The installed 225 mm probe completed and failed after **48.07 s** with
`base made no measured progress`. It walked normally and entered `to_stand` /
`stand` twice; it transiently reached **2.34 mm** combined error, but the moving
stop/stand transition displaced it again. Final error **49.65 mm**. No fall or
walking-tilt guard failure was observed. Raw data is fetched and preserved.

**The planted-body correction candidate finished successfully: 7.11 s, settled
combined error 2.24 mm. Its raw output has been fetched. Do not relaunch it.**
Next: commit/publish/register the candidate, verify actual installed source hashes,
then the installed short probe and full run 49. The old detached container ID was:
`384f1dd8dc2bc71a38c4b0ddae24cd2d6174a4c84d2cfe0fccfed776de689684`.
Remote root: `/home/hckang/ws/elesim-mobile-diagnostics/base-planted-trim-20261117`.
Output: `output/planted-trim.{summary.json,state.jsonl,profile.jsonl,console.log}`.
Controller SHA: `44a4e925c4feec3b8905343d2f5f08592fdc7c052956c0170daea7966367a3bd`.
It is an explicit diagnostic source override, **not deployed code or acceptance**.
It handles slow corrections by moving the supported body through leg IK, retaining
fixed feet; workspace exhaustion returns to walking for a new measured stance.
Host focused candidate regressions: **46 passed**; final focused Sim batch: **55 passed**. Uncommitted files are the controller,
`test_62_walk_start_command.py` and new `test_76_planted_body_trim.py`; preserve them.

Actual installed two-process loopback DDS burst **passed 16/16**. The four-process
smoke cannot run in an isolated Sim image because its Robot application import is
correctly absent; the canonical dev image remains unprepared. Do not add sibling
packages to runtime or start a new environment rebuild as a detour.

See `mobile_walk_level_live_20261117.json` for exact release keys, evidence hashes
and the completed v30 probe. Keep the 3 mm settled arrival and loaded-hold success
criteria. Only after the short arrival succeeds should full run 49 start.

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
- Last runtime source published/registered on both installs: **`58c526d84e3971f86480e2b3abeaf44fbbb659da`**.
- Operator release: `66a9aee8d1f417a1277bf2e3f7d27577185947e100cb114bbd0b7e3ab6eb3487`.
- Compute release: `e0046dfbadc5a8b33850cb22b81e0985ffba916a49d3caf859c922feeee9976b`.
- Registration/approved SROS2 rotation v30 completed atomically. Normal `test`
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
