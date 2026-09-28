# Pilot IMU model catalog

Each `*.json` file is a versioned calculation program. The basename must match
`id`; Pilot reads this directory on startup and exposes only `id`, `version`,
`label`, and `requires_imu` to UI. Selecting an entry sends its `program` to
Robot once, where C++ validates it and runs it in the local arm loop.

`program.nodes` is a directed acyclic graph in array order, with at most 64
nodes. Each reference `a`/`b` names an earlier node. The four `outputs` name
the corrected q components in canonical q order. Supported nodes are:

- `{"op":"q","index":0..3}`: theoretical q component.
- `{"op":"imu","index":0..2}`: latest IMU roll, pitch, yaw value.
- `{"op":"const","value":number}`: bounded finite constant.
- `add`, `sub`, `mul`, `div`: binary nodes with `a` and `b` indexes.
- `neg`, `sin`, `cos`: unary nodes with an `a` index.

The Robot checks corrected q against its existing limits. Division by zero,
nonfinite results, and stale required IMU samples fail locally. The shipped
`identity.json` ignores IMU and preserves the former motor target mapping.
The Teensy frame format and calibration frame must be defined before adding a
sensor-dependent production model.
