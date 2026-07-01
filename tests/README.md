# Tests

The test tree is organized for manual debugging first. Start in `scenarios/`
when trying to understand a robot behavior over time. Use `contracts/` when
checking low-level pieces during refactors. Use `regressions/` when a concrete
old failure mode needs to stay pinned.

- `scenarios/pick/`: config -> Look -> Aim -> equal-sag -> Grasp/LJI -> E2E.
- `scenarios/vision/`: perception config -> hand-eye -> detector/tracker ->
  sim-camera contracts.
- `scenarios/go2/`: ROS/env -> lowstate/bridge -> mirror -> locomotion ->
  payload/MPC contracts.
- `contracts/`: subsystem checks that do not form a time-ordered debug story.
- `regressions/`: fixtures derived from real or synthetic failure logs.

Useful commands:

```bash
python3 -m pytest tests
python3 -m pytest tests/scenarios/pick
python3 -m pytest tests/scenarios/pick/test_43_lji.py
python3 -m pytest tests/contracts/vision/test_lji_invariants.py
python3 -m pytest tests/regressions
python3 -m pytest tests/contracts
python3 tools/test_gui.py
```

The GUI `안정` and `전체` groups currently run the full test tree. Use the
scenario buttons when narrowing a failure to one debug phase, `계약` for
subsystem invariants, and `회귀` for previously observed symptoms.
