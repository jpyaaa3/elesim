# Elesim Maintenance Guide

Read `misc/docs/architecture.md` before changing a cross-cutting behavior.

## Deployable Packages

- `ui`
- `controller`
- `router`
- `robot`
- `simulator`

Each package is independently installable. Implement process behavior inside
the owning deployment and share only protocol contracts through
`packages/protocol`.

## Rules

- A deployment must not import a sibling deployment.
- UI uses operator protocol APIs, never workflow implementation modules.
- Robot owns physical I/O and local safety only.
- Controller owns Pick, Gaze, Vision and IK computation.
- Simulator consumes `model/bundles/default`; it does not rebuild models unless
  `ELESIM_SIM_DEV_REBUILD=1` is explicitly set.
- Protocol changes require a versioned contract and integration test update.
- Do not add root compatibility launchers or legacy re-export modules.
- Preserve unrelated local changes and experiment evidence.

## Verification

Run the per-package matrix in `misc/docs/architecture.md`, then run
`misc/integration/smoke_topology.py`. Build isolated contexts with
`python3 misc/tooling/release/build.py`.
