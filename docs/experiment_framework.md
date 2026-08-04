# Experiment Framework

Experiments are repository tooling, not runtime applications. Canonical runners
live under `research/experiments`; analysis-only commands live under
`research/analysis`.

## Runtime Prerequisites

Start Pilot and the selected Robot or Sim deployment before running
an interactive experiment. UI is required only for interactive presentation.
All processes must use the same DDS graph and security profile. Source-workspace
runs first build and source the ROS interfaces:

```bash
colcon build --packages-select elesim_interfaces
source install/setup.bash
export PYTHONPATH="pilot/src:sim/src"
python3 -m elesim_pilot.main --config pilot/config/default.yaml
python3 -m elesim_sim.main --config sim/config/default.yaml
```

Installed release artifacts use the `elesim-pilot` and
`elesim-sim` console commands instead. There is no Router process.

## Evidence

Experiment outputs remain under `research/results/` and runtime traces under `logs/`.
These are evidence, not deployment inputs. Preserve a run's effective config
and run ID with its output so later analysis does not depend on current
defaults.

## Test GUI

`python3 tools/quality/test_gui.py` discovers tests across ROS interfaces,
release projects and tooling. Its buttons reflect ownership rather than a
single root `tests/` hierarchy.
