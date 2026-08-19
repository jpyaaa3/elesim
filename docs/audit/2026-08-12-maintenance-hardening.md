# 2026-08-12 Maintenance Hardening — historical summary

> This is a dated audit record, not a current runbook. The current contracts are
> [`../architecture.md`](../architecture.md), [`../setup.md`](../setup.md), and
> [`../deployment.md`](../deployment.md).

## Scope

The audit reviewed exception boundaries, orphan candidates, generated artifact
ownership, source-only research files, and long maintenance paths. It compared
Python/JavaScript references, console entry points, package data, generated
wrappers, release contexts, wheels and regression tests rather than deleting
files based on names alone.

## Decisions recorded

- DDS and management SSH failures remain separate diagnostics; an SSH success
  cannot claim DDS readiness.
- runtime/start failures are surfaced with their host and role; broad fallback
  does not turn a partial launch into success.
- source-only research/debug material and test fixtures remain in the repository
  but are excluded from curl installation and release contexts.
- exact ownership manifests are required for generated cleanup; legacy paths are
  not silently adopted.
- runtime logs are bounded and archive errors do not prevent a shutdown attempt.
- the Docker backend, Tailscale sidecar, security generation and host lifecycle
  retain explicit ownership boundaries.

## Verification recorded at the time

The then-current focused installer, topology, runtime-status, security and media
tests were run, along with syntax/compile checks and `git diff --check`. The
environment did not provide the full ROS/scientific stack needed for the
canonical quality gate; that gate is now defined in `MILESTONES.md` and must run
inside setup-generated `elesim-dev`.

The detailed command transcript was intentionally removed from the active docs:
it contained temporary host paths, stale source revisions and recovery commands
that are unsafe to copy into a new installation. Git history remains the source
for forensic reconstruction.
