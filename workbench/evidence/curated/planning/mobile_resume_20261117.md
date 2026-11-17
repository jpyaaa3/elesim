# Mobile task: exact continuation checkpoint

Updated: 2026-11-17T09:41:25+09:00
This file records execution state. `docs/status.md` still owns acceptance milestones.

## Resume here

**Do not restart scan or the full mobile run. First check the exact prototype
container and `output/prototype.summary.json` on the server. If a result exists,
read it; do not rerun. If still running, monitor it. Only if neither exists is
the next action the isolated 225 mm probe below.** Run 49 has **not** started.
The runner refuses any existing summary/state/profile output so an interruption
cannot silently overwrite a finished or partial experiment.
It also creates an exclusive `.run.json` marker before loading the physics stack.
The prototype is launched as a detached server-owned container, with console and
state files bound into remote `output/`; no Codex tool-session ID is needed to
recover it. Never relaunch just because an old session ID is unavailable.

## Objective and fixed scope

- Official installs: `/home/user/ws/newsim` and
  `hckang@100.74.222.24:/home/hckang/ws/newsim`.
- Fixed D100 Sim fixture `(0.7, 0.15, 0.55)`; Pilot uses sensed geometry.
- Required result remains observation → reconstruction → C-band positioning →
  hug/lift → independent same-execution 5 s loaded hold. It is **not achieved**.
- Keep existing 900 s task / 180 s phase / 90 s base deadlines, 0.5 s feedback,
  0.07 rad body tilt, 3 mm combined final arrival, friction and hold conditions.
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

## Current blocking evidence: isolated base plant

- Recorded goal offset: `(0.2249203741, 0.0090866415, ~0)`; same policy Home.
- Installed baseline controller failed **body_tilt** after **4.168 s**.
  Maximum roll/pitch: `[0.0402549411234283, 0.07019917980996326]` rad.
  Combined final-position/yaw error at failure: **54.63 mm**.
- This isolates the physical Go2 controller from DDS/scan/map. It does not prove
  obstacle admission, network behavior, grasp or hold.
- Maintained runner: `workbench/research/debug/profile_mobile_base.py`, using
  `profile_sim_walking.py` and the production Genesis plant. No sibling role
  import is added to runtime. Its goal command is parity-tested against Pilot.

## Server-owned prototype job

Launched detached as `elesim-merry-base-goal-prototype`.
Container ID: `339d60e7a6df135cf5580f1ad967627e92dc6c9c2946414b2a1bab9806f4ac4c`.
Read remote `output/prototype.summary.json` or the live container first.
Runner SHA256: `a090863433ebd1d7a46f09118907359a82ad4dbd79cd25f1d6bca392e61b76c0`.
Console: `output/prototype.console.log`. This job does not depend on a Codex
exec session, and existing output prevents a duplicate run.

## Exact next probe inputs

Remote source root: `/home/hckang/ws/elesim-mobile-diagnostics/base-goal-20261117`.
It contains maintained source copies (read-only mount at `/diagnostic`),
`case.json`, `prototype_controller.py`, and writable `output/`.

Use the official installed Sim service:
`svc-test-sim-readiness-c539c9eacd40eb82be9248b70089b842baf793273edb8366c40f485b9a1e3ed5`.
Launch through `/home/hckang/ws/newsim/bin/elesim-compose`, with
`containers/compose.instances.yaml`, `run --rm --no-deps -d`, an exact distinct
container name, and GPU2 from that service. Source ROS + generated ROSIDL, set
`PYTHONPATH=/diagnostic:$PYTHONPATH`, then run:

```bash
python -m workbench.research.debug.profile_mobile_base   --case /diagnostic/case.json   --config /opt/elesim/config/config.yaml   --model-bundle /opt/elesim/data/models/assemblies/zed-mini   --controller-source /diagnostic/prototype_controller.py   --output /probe-output/prototype
```

Bind remote `output/` at `/probe-output:rw`. The runner enforces the unchanged
90 s wall deadline, 10 s no-progress guard, 0.07 rad tilt and 3 mm/settling gates.
The implementation override is an **unvalidated prototype**, not installed code.
Its current source SHA256 is `d6f32eb40576652167d00355eea500c71eeca57ff97ab56ec6fd624d8656acda`.

If it fails, work on this short reproduction and preserve the raw result.
If it passes, validate/deploy the controller change and verify actual installed
image hashes + Enforce/GPU2. Only then run full fixed-case 49 from Home.

## Files that must survive

Generated evidence root:
`workbench/evidence/generated/mobile-installed-20261115/`.

- `base-goal-baseline/`: raw trajectory, profile, summary (fetched locally).
- `base-goal-baseline.log`, `mobile-base-case48.json`, `base-goal-staging.json`.
- `rotation-v29.json`, `registration-v29.json`, `update-*-v29.log`.
- `operator-source-49.json`: prepared manifest, **no run-49/report.json yet**.
- `dds-burst-real-before.log`, `dds-burst-real-after.log`: actual ROS regression.
- Uncommitted controller prototype, its `test_75_liftoff_posture.py` tests and
  `mobile_stance_transition_20261116.json` must be preserved. Do not silently
  include it in unrelated commits. No successful 225 mm arrival is claimed.

SSH uses the existing connection-local MSS proxy:
`-o 'ProxyCommand=python3 -m workbench.research.support.ssh_mss_proxy %h %p'`.
This does not change interface MTU. Existing canonical dev attachment is absent;
do not install host dependencies or search old containers again.
