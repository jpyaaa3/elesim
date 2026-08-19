# Experiment Framework

> Experimental/research documentation. It is not part of the installed runtime
> contract. Current runtime behavior is defined in [`README.md`](README.md) and
> [`architecture.md`](architecture.md).

## Scope

Experiments are repository tooling, not runtime roles. Canonical runners live in
`misc/research/experiments`; analysis-only commands live in
`misc/research/analysis`. They consume an already running Pilot and selected
Robot or Sim deployment. UI is required only for interactive presentation.

Do not add an experiment as a fifth Compose role, Router, DDS broker, or hidden
runtime import. An experiment must use the documented ROS/DDS contracts and the
same system/domain/RMW/security profile as the deployment it observes.

## Environment

Run experiments in the setup-generated persistent Developer container so the
host Python and ROS installation remain untouched.

```bash
elesim-up
elesim-dev
elesim-dev python3 misc/research/experiments/<runner>.py
```

The runner records the effective config, source revision, topology mode, role
endpoints and run ID. Outputs belong under `misc/research/results/`; runtime
snapshots belong under the generated prefix's bounded `logs/runs/`. Neither is a
release input.

## Evidence requirements

An experiment report must state whether it used simulated or physical hardware,
which GPU policy/encoder was active, whether observer/hand-eye media was
involved, and which acceptance gates were not exercised. A headless experiment
does not prove physical convergence, SROS2 enforcement, NAT traversal or display
behavior.
