# Workbench routing

This tree is repository-only. Runtime packages must never import it, and release
contexts must never copy it.

## Ownership

- `tools/quality/`: canonical software gates and test UI
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

Pilot test semantics are encoded in the tree:

- `contracts/vision/` contains small deterministic LJI/visual-servo invariants.
- `regressions/pick/` contains previously observed pick failures, not a workflow.
- `regressions/visual_servo/` preserves feasible-look and visual-servo failures.
- `scenarios/gaze/` follows controller, preview, mode and walking stages.
- `scenarios/pick/` follows `00` setup, `10` Look/Ready, `20` Aim, `30`
  equal-sag, `40` Grasp/LJI and `50+` session stages.
- `scenarios/vision/` follows config, hand-eye, world mapping, remote config,
  lifecycle, detector/tracker and sim-camera stages.
