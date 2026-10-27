# Workbench routing

This tree is repository-only. Runtime packages must never import it, and release
contexts must never copy it.

## Ownership

- `tools/quality/`: canonical software gates and test UI
- `tools/bench/`: versioned heavy-benchmark catalog, isolated execution and evidence collection
- `tools/release/`: isolated release build and verification
- `tools/setup_preview.py`: side-effect-free setup GUI preview
- `tests/protocol/`: protocol unit, wire-contract and compatibility coverage
- `tests/apps/<role>/`: role-owned unit, contract, scenario and regression coverage
- `tests/setup/`: installer, lifecycle, connection and security coverage
- `tests/system/`: cross-process DDS, RGB-D and WebRTC probes
- `tests/model_builder/`: blueprint, bundle and URDF generation coverage
- `tests/tools/{quality,release}/`: workbench tool self-tests
- `tests/research/{analysis,debug,experiments}/`: research utility self-tests
- `research/experiments/`: repeatable experiment runners
- `research/analysis/`: offline result analysis
- `research/debug/`: manual diagnostics
- `research/support/`: helpers shared only by research commands
- `evidence/curated/`: versioned summaries, metadata and representative figures
- `evidence/generated/`: regenerable output; ignored by Git
- `archive/`: retired tools in ignored local ZIPs; only README and hash inventory are tracked
- `tests/research/support/`: historical calculation fixtures retained for regression assertions, no CLI

## Canonical commands

Run through the setup-generated persistent development attachment:

```bash
elesim-dev python3 workbench/tools/quality/check.py --group required
elesim-dev python3 workbench/tools/quality/check.py --group extended
elesim-dev python3 workbench/tools/release/build.py
elesim-dev python3 workbench/tools/release/verify.py dist/releases
elesim-dev python3 workbench/tests/system/smoke_topology.py
```

Prefer `python -m workbench...` when importing another workbench module. New
raw output goes under `evidence/generated/`; promote only compact, reproducible
evidence to `evidence/curated/`.

Test folders are selected by the matrix in `tools/quality/check.py`. Numeric
scenario filenames encode workflow phase, not pytest ordering or shared state.
Non-test helpers stay beside their owning suite and do not enter runtime wheels.

The optional test GUI uses that same matrix and its workflow selections. Its
development runner takes the absolute current-install `bin/elesim-dev` wrapper;
it does not guess a Compose project or find historical development containers.

Heavy walking, performance, calibration, multiview and completion runs use
`python3 -m workbench.tools.bench list|plan|run|verify|report|pack|collect|index`.
See `docs/benchmarks.md` for recipe inputs and measurement scopes. A completed
run can fail acceptance: `run`/`verify` return 1 for criteria failure and 2 for
incomplete execution. `report` is an explicit reporting operation, not a gate.
Do not restore shell suites that swallow analyzer errors, silently accept
unused flags, or execute code from `evidence/generated/` or `/tmp`.

Preserve raw evidence and existing safety/regression cases. Old generated trees
are indexed as legacy evidence, not assumed passing, and are not executable
source. Move cross-role integration assertions into `tests/system/` with explicit
matrix source roots rather than granting sibling imports to a role/protocol gate.

`workbench/README.md` lists the maintained entrypoints and research workers.
Do not import archived code from runtime, benchmarks or tests. Old experiment
math needed by regression tests belongs in `tests/research/support/`; preserve
the assertions when retiring the CLI. Archive the exact pre-edit file bytes and
record hashes before removing a tracked source. Archive ZIPs are local-only;
the tracked inventory must not require them for normal verification.

Pilot test semantics are encoded in the tree:

- `contracts/vision/` contains small deterministic LJI/visual-servo invariants.
- `regressions/pick/` contains previously observed pick failures, not a workflow.
- `regressions/visual_servo/` preserves feasible-look and visual-servo failures.
- `scenarios/gaze/` follows controller, preview, mode and walking stages.
- `scenarios/pick/` follows `00` setup, `10` Look/Ready, `20` Aim, `30`
  equal-sag, `40` Grasp/LJI and `50+` session stages.
- `scenarios/vision/` follows config, hand-eye, world mapping, remote config,
  lifecycle, detector/tracker and sim-camera stages.
