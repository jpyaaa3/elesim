# Runtime and Test Audit — 2026-07-20

> Historical audit snapshot. It describes the repository at the date above and
> is not the current runtime contract or an installation runbook. Start from
> [`../../README.md`](../../README.md) and [`../../architecture.md`](../../architecture.md).

## Purpose

This audit captured the pre-Router-free/refactoring baseline: package counts,
test output, runtime ownership risks and missing live gates. The accompanying
`baseline.json` and `coverage.md` are evidence for that date, not current
version or acceptance status.

## Historical findings

- The old graph mixed application responsibilities and transport assumptions.
- Unit tests did not prove real multi-host DDS, NAT, SROS2 enforcement, GPU/X11,
  WebRTC relay or physical Robot safety.
- Generated artifacts, environment parity and lifecycle ownership required
  stronger boundaries.

Those findings motivated the current four-role direct-DDS architecture,
protocol registry, bounded authority/media paths, generated release contexts,
and ownership-based installer described in the active documentation.

## How to read this directory

Use these files only to compare historical evidence with a newer revision. Do
not copy old role names, transport commands, source paths or test counts into a
new deployment. Current software and manual gates are listed in
[`../../MILESTONES.md`](../../MILESTONES.md) and
[`../../OPEN_ISSUES.md`](../../OPEN_ISSUES.md).
