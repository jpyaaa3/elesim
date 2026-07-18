# Pick Scenario

Read this folder from top to bottom. The numeric prefixes follow the order a
human usually debugs a pick run.

- `00` setup: config, ownership, allowed host command sources.
- `10` Look/Ready: view pose, recovery, ready direction, and pregrasp geometry.
- `20` Aim: aim convergence and UV Jacobian sign checks.
- `30` equal-sag: sag drift and ready pose correction inputs.
- `40` Grasp/LJI: guided grasp, trajectory, and Local Image Jacobian behavior.
- `50+` session checks: extend-ready, convergence, end-to-end, and timing.

`test_43_lji.py` checks whether the grasp stage has enough LJI information to
take a step. Pure LJI math invariants live under `tests/contracts/vision/`, and
known old symptoms live under `tests/regressions/pick/`.
