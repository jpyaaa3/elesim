# Runtime and Test Audit, 2026-07-20

This snapshot is the pre-fix baseline for the deployment refactor. It records
facts that should remain reproducible while implementation changes.

## Baseline

- Branch: `refactoring`
- Revision: `0e631c5`
- Tracked Python: 448 files, approximately 93,606 lines
- Documented package matrix: 399 passed, 1 skipped, 1 warning
- Extended tooling tests: 13 passed
- Combined software baseline: 412 passed, 1 skipped, 1 warning
- Protocol topology smoke test: passed
- Release contexts: wheel creation passed, isolated installation/startup was not
  tested

Run the canonical software gate with:

```bash
python3 tooling/quality/check.py --group required
python3 tooling/quality/check.py --group extended
```

## Confirmed Priority Defects

1. `model/bundles/default` is not self-contained. Its URDFs resolve meshes from
   `model/source`, and the release builder hides the defect by copying the whole
   model tree.
2. Robot safety is not independent of the active controller. Arm stop behavior
   is velocity-mode specific, current checks run only during telemetry, and
   router liveness is not supervised.
3. Physical telemetry has no canonical measured four-DOF `q`, so controller
   state and subsequent partial commands can be based on stale targets.
4. Controller and simulator still wrap legacy local ZMQ protocols behind a v3
   exterior. Startup, reconnect and late-subscriber behavior are therefore not
   governed by one contract.
5. Controller and simulator contain broad copies of each other's implementation
   modules. Import-boundary tests cannot detect copied role violations.
6. Protocol payloads are untyped dictionaries and lifecycle, trace and payload
   validation are incomplete.
7. UI RPC is synchronous and optimistic: local state can change before remote
   acknowledgement, and controller absence can block startup.
8. Existing tests are rich in relocated characterization tests but sparse at
   process boundaries, isolated-wheel startup, safety timing, property checks,
   and headless Look-Aim-Grasp replay.

The detailed remediation order is maintained in `docs/OPEN_ISSUES.md` and its
Korean counterpart. Hardware-in-the-loop remains a manually approved gate;
the automatic matrix is software-only.

## Post-Implementation State

The implementation following this baseline is recorded in `current.json` and
`coverage.md`. The software gate now includes direct protocol-v3 endpoints,
local robot safety, a self-contained hashed model bundle, deterministic
property/headless/replay tests, readability budgets, focused mutations, and
isolated installation of all five release contexts.

This does not close the physical validation gate. A real or Genesis
Look-Aim-Grasp run with camera timing and contact remains required before the
pick pipeline can be called operationally stable.
