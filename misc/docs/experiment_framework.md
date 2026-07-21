# Experiment Framework

Experiments are repository tooling, not runtime applications. Canonical runners
live under `misc/tooling/experiments`; analysis-only commands live under
`misc/tooling/analysis`.

## Runtime Prerequisites

Start the router, controller and selected robot or simulator deployment before
running an interactive experiment. Source-workspace runs need these package
roots on `PYTHONPATH`:

```bash
export PYTHONPATH="packages/protocol/src:router/src:controller/src:simulator/src"
python3 -m elesim_router.main
python3 -m elesim_controller.main --config controller/config/default.yaml
python3 -m elesim_simulator.main --config simulator/config/default.yaml
```

Installed release artifacts use the `elesim-router`, `elesim-controller` and
`elesim-simulator` console commands instead.

## Evidence

Experiment outputs remain under `misc/results/` and runtime traces under `logs/`.
These are evidence, not deployment inputs. Preserve a run's effective config
and run ID with its output so later analysis does not depend on current
defaults.

## Test GUI

`python3 misc/tooling/quality/test_gui.py` discovers tests across protocol,
release projects and tooling. Its buttons reflect ownership rather than a single
root `tests/` hierarchy.
