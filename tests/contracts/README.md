# Contracts

Contracts are subsystem checks that are useful during refactors but do not tell a
story by themselves.

- `core/`: protocol, trajectory, runtime URDF selection, debug marker payloads.
- `builder/`: URDF merge/build contracts.
- `arm/`: arm mounting and lightweight alignment helpers.
- `gaze/`: gaze stabilizer sign behavior.
- `vision/`: local image Jacobian and visual-servo math invariants.
- `archive/`: older visual-servo reference tests kept for archaeology.
