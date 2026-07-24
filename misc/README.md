# Miscellaneous Development Assets

This directory contains repository material that is not installed on a
runtime machine:

- `tooling/`: build, release, quality, debug, analysis and experiment tools
- `integration/`: cross-deployment topology tests
- `scripts/`: source-tree development launchers
- `docs/`: architecture, deployment and maintenance documentation
- `model/source/`: editable geometry and blueprint inputs
- `results/`: preserved experiment evidence

Tests remain beside the package or deployment they verify. They are excluded
from application wheels and role-specific release contexts.

Runtime inputs and generated artifacts remain at repository root:

- `controller/`, `ui/`, `robot/`, `simulator/`
- `packages/elesim_interfaces/`, `packages/protocol/`
- `model/bundles/`
- `dist/`
- `logs/`
