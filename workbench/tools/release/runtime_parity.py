#!/usr/bin/env python3
"""Check that the selected runtime roles' dependencies exist in dev.

The developer attachment is intentionally a broad editable environment.  This
small gate prevents that convenience from hiding a missing or incompatible
dependency in a selected deployable role before the isolated release probe is
run.
"""

from __future__ import annotations

import importlib.metadata
import importlib.util
import os
import sys
from pathlib import Path

from packaging.requirements import Requirement


ROOT = Path(__file__).resolve().parents[3]
ROLE_ROOTS = {
    "pilot": ROOT / "payload/runtime/docker/pilot",
    "sim": ROOT / "payload/runtime/docker/sim",
    "ui": ROOT / "payload/runtime/docker/ui",
    "robot": ROOT / "payload/runtime/native/robot",
}


def _selected_roles() -> tuple[str, ...]:
    raw = os.environ.get("ELESIM_RUNTIME_ROLES", "pilot,sim,ui")
    roles = tuple(dict.fromkeys(part.strip() for part in raw.split(",") if part.strip()))
    unknown = sorted(set(roles) - set(ROLE_ROOTS))
    if unknown:
        raise ValueError(f"unknown runtime role(s): {', '.join(unknown)}")
    return roles


def _requirements(path: Path) -> tuple[Requirement, ...]:
    result: list[Requirement] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or line.startswith(("-", "git+")):
            continue
        result.append(Requirement(line))
    return tuple(result)


def check() -> tuple[str, ...]:
    errors: list[str] = []
    try:
        roles = _selected_roles()
    except ValueError as exc:
        return (str(exc),)
    for role in roles:
        lock = ROLE_ROOTS[role] / "requirements.lock"
        if not lock.is_file():
            errors.append(f"{role}: missing requirements.lock: {lock}")
            continue
        for requirement in _requirements(lock):
            try:
                installed = importlib.metadata.version(requirement.name)
            except importlib.metadata.PackageNotFoundError:
                errors.append(f"{role}: missing {requirement}")
                continue
            if requirement.specifier and installed not in requirement.specifier:
                errors.append(
                    f"{role}: {requirement.name} has {installed}, "
                    f"requires {requirement.specifier}"
                )
    mpc_enabled = os.environ.get("ELESIM_GO2_MPC_ENABLED", "1") == "1"
    mpc_present = importlib.util.find_spec("convex_mpc") is not None
    if mpc_enabled and "sim" in roles and not mpc_present:
        errors.append("sim: go2-convex-mpc is enabled but convex_mpc is unavailable")
    if not mpc_enabled and mpc_present:
        errors.append("go2-convex-mpc is present although the runtime switch is disabled")
    return tuple(errors)


def main() -> int:
    errors = check()
    if errors:
        print("runtime dependency parity failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    roles = os.environ.get("ELESIM_RUNTIME_ROLES", "pilot,sim,ui")
    print(f"runtime dependency parity passed: {roles}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
