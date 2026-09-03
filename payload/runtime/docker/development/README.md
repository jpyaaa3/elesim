# EleSim development container

This input is copied by the GUI/CLI setup package into a generated Compose
project. It creates one privileged amd64 Ubuntu/ROS2 workspace containing the
complete coding dependency set. It is not a deployable runtime role.

The host checkout is mounted read/write and installed editable when the
container starts. No separate observability service is part of this context.
