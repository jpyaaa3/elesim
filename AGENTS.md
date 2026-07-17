# Elesim Maintenance Guide

Read `docs/architecture.md` before changing a cross-cutting behavior.

## Canonical Entry Points

- `python3 ctrl.py`
- `python3 host.py`
- `python3 sim.py`
- `python3 perception_worker.py`

These are compatibility launchers. Implement process behavior in `apps/` and
reusable functionality in `engine/`.

## Rules

- Follow behavior ownership: Pick, Gaze, Vision, Arm, GO2, Simulation.
- Import application configuration only from `engine.config`.
- Import protocol types only from `engine.core.protocol`.
- Do not recreate legacy re-export modules or duplicate a perception pipeline.
- Keep UI dependent on public service APIs, never on workflow implementation
  modules.
- Treat `crafts/` as generated output. Preserve unrelated local changes.

## Verification

```bash
python3 -m pytest tests
python3 tools/quality/test_gui.py
```

In the provided development container, use the repository's Docker Compose
service when local scientific dependencies are unavailable.
