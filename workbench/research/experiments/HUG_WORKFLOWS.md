# Hug workflow ownership (2026-11-10)

New work follows `docs/hug-ga-rl-pipeline-plan.md`. The end-to-end coordinator
is planned, not implemented. Home-to-wrap must be learned by PPO, including the
former manually commanded connection. Reference paths supply reset states only.

## Current entrypoints

- `hug_reverse_ga.experiment`: solver grasp to Home, verified release states.
- `hug_home_rl.experiment`: Home-prefix reverse curriculum and bounded lift
  command-variation cost. Full Home success is not yet established.
- `analysis.export_home_checkpoint`: checkpoint export for the current task.
- `analysis.record_reverse_ga`: current reverse-GA video.
- Mapper support, geometry, approach/collision diagnostics and boundary probes
  remain useful for the planned pipeline and are not retired as failed work.

## Retired training workflows; retained source dependencies

| Package | Superseded workflow | Why source remains |
| --- | --- | --- |
| `wrap_grasp` | Human-waypoint cylinder training | Current tasks use seed/training and environment helpers; regression coverage |
| `hug_interpolation` | Two-band independent training/study | Current tasks use video and other shared helpers; interpolation regression coverage |
| `hug_c` | Direct Home PPO / evolutionary population study | Current environment, solver adapter, policy export, triangle selection and runner |
| `hug_ga` | Forward Home-to-wrap/lift GA and imitation handoff | Reverse GA uses parade helpers and the source provenance digest |
| `hug_reverse_rl` | Original reverse-path PPO with unsuccessful Home transfer | Home PPO inherits reset bank, environment, schedule and plan validation |

These directories are not independent recommended experiments any more. Their
old runbooks are marked historical. Do not import from `workbench/archive`.
We deliberately preserve their Python bytes: removing or moving them would break
imports and the recursive source hashes of already verified plans. Extracting a
shared library requires explicit plan migration and revalidation, not silent
hash bypass. Existing regression assertions and physics safety tests are retained.

## Physically retired files

- `analysis/record_reverse_ppo.py` and `analysis/watch_reverse_ppo.py`: old
  checkpoint/report workflow; current Home task has evaluate/video and export.
- `analysis/record_ga_candidate.py`: forward-GA candidate recorder.
- `support/hug_study_run.py`: historical one-off Pilot diagnostic launcher.
- `hug_c/gpu2-next-study.json`: obsolete direct-PPO/evolution launch preset.

Exact pre-edit bytes of 85 files are in
`workbench/archive/retired-hug-pipelines-20261110.zip`, verified by per-file SHA256.
`workbench/archive/index.json` records removed versus retained status. The ZIP is
ignored and local-only; Git does not distribute these uncommitted historical
sources. Copy it separately for archival backup. Raw results, NPZ, videos and
curated failure evidence are preserved: failed experiments still provide useful
regression and design evidence. No server jobs or server source were changed.
