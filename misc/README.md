# Repository-only material

`misc/` contains research, system verification and developer/release tooling.
These files support development and acceptance checks but are not runtime
applications and are not copied into role wheels or production release trees.

- `misc/research/`: offline analysis, experiments, debugging and evidence
- `misc/system_tests/`: cross-process DDS/RGBD/WebRTC checks
- `misc/tools/`: quality gates, release checks and developer helpers

Use the generated `elesim-dev` environment for these commands. Runtime users
use the installed role wheels, `elesim-up`, Compose and systemd instead.
