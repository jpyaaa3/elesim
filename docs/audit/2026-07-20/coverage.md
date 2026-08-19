# Line-Execution Audit — 2026-07-20

> Historical coverage snapshot. It is not a current quality report; see
> [`../../README.md`](../../README.md) and the active quality commands in
> [`../../MILESTONES.md`](../../MILESTONES.md).

This standard-library `trace --missing` report recorded which pre-refactor
control paths were exercised. It was not branch coverage and did not claim
physical or multi-host validation.

## Boundary

The report intentionally left graphical, ROS middleware, hardware, network NAT,
SROS2 and media relay paths incomplete. Those paths remain explicit manual gates
today rather than being inferred from line execution.

## Interpretation

Historical line percentages and role names are comparable only with the exact
revision recorded in `baseline.json`. They must not be used to decide whether a
current release is ready. Use the setup-generated `elesim-dev` environment and
the required/extended quality groups for current software verification.
