"""Host-side operation locks used by install-scoped maintenance wrappers.

The lock is deliberately acquired in a short-lived stdlib-only helper and
then carried across an ``exec`` in an inherited descriptor.  A marker in the
argument vector is not sufficient to enter the locked branch: the re-execed
wrapper must also have descriptor 9 pointing at the exact lock file.  This
keeps an ambient environment variable from opting a process out of the
serialization boundary.
"""

from __future__ import annotations

import argparse
import fcntl
import os
import shlex
import stat
import sys
from pathlib import Path


LOCK_FD = 9
LOCK_MARKER = "__elesim_operation_lock_held_v1"


def _safe_lock(path: Path) -> int:
    path = Path(os.path.abspath(os.fspath(path)))
    current = path
    while True:
        if current.is_symlink():
            raise RuntimeError(f"refusing symlink lock path: {path}")
        if current == current.parent:
            break
        current = current.parent
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    if path.parent.is_symlink() or not path.parent.is_dir():
        raise RuntimeError(f"operation lock parent must be a real directory: {path.parent}")
    path.parent.chmod(0o700)
    try:
        fd = os.open(
            path,
            os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o600,
        )
    except OSError as exc:
        raise RuntimeError(f"cannot open operation lock: {path}") from exc
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise RuntimeError(
                f"operation lock must be a singly-linked regular file: {path}"
            )
        fcntl.flock(fd, fcntl.LOCK_EX)
        os.set_inheritable(fd, True)
        return fd
    except BaseException:
        os.close(fd)
        raise


def run_locked(lock: Path, command: list[str], *, fd: int = LOCK_FD) -> int:
    """Acquire ``lock`` and exec ``command`` with the lock descriptor held."""

    if not command:
        raise RuntimeError("a locked operation requires a command")
    if fd < 3 or fd > 255:
        raise RuntimeError("lock descriptor is outside the safe range")
    opened = _safe_lock(lock)
    try:
        if opened != fd:
            os.dup2(opened, fd, inheritable=True)
            os.close(opened)
            opened = fd
        argv = [command[0], LOCK_MARKER, str(fd), *command[1:]]
        os.execvp(argv[0], argv)
    except BaseException:
        try:
            os.close(opened)
        except OSError:
            pass
        raise
    return 127


def _main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Acquire an EleSim operation lock and exec a command")
    parser.add_argument("--lock", required=True, type=Path)
    parser.add_argument("--fd", type=int, default=LOCK_FD)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = list(args.command)
    if command[:1] == ["--"]:
        command = command[1:]
    return run_locked(args.lock, command, fd=args.fd)


if __name__ == "__main__":  # pragma: no cover - exercised by wrapper tests
    try:
        raise SystemExit(_main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"elesim operation lock: {exc}", file=sys.stderr)
        raise SystemExit(73)


def render_lock_preamble(*, lock_path: Path, maintenance_root: Path) -> tuple[str, ...]:
    """Render the shell prologue shared by scoped host wrappers.

    The first invocation re-execs through this module.  The helper keeps the
    lock open on fd 9 while replacing itself with the wrapper, so the wrapper
    can only enter its body when that exact descriptor names the expected
    regular file.  No environment variable is consulted for this decision.
    """

    lock = str(Path(lock_path).absolute())
    bundle = str(Path(maintenance_root).absolute())
    return (
        f"elesim_operation_lock={shlex.quote(lock)}",
        f"elesim_operation_maintenance={shlex.quote(bundle)}",
        f"if [[ ${{1:-}} == {LOCK_MARKER!r} ]]; then",
        f"  if [[ ${{2:-}} != {LOCK_FD} || $# -lt 2 ]]; then",
        "    printf '%s\\n' 'invalid internal operation-lock handoff' >&2",
        "    exit 73",
        "  fi",
        f"  elesim_operation_fd={LOCK_FD}",
        "  shift 2",
        "  if [[ ! -f /proc/$$/fd/$elesim_operation_fd ]] || [[ \"$(readlink -- /proc/$$/fd/$elesim_operation_fd 2>/dev/null || true)\" != \"$elesim_operation_lock\" ]]; then",
        "    printf '%s\\n' 'operation-lock handoff is missing or points at a different file' >&2",
        "    exit 73",
        "  fi",
        "else",
        "  exec env PYTHONPATH=\"$elesim_operation_maintenance\" PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 python3 -B -S -m elesim_setup.operation_lock --lock \"$elesim_operation_lock\" --fd 9 -- \"$0\" \"$@\"",
        "fi",
    )


__all__ = ["LOCK_FD", "LOCK_MARKER", "render_lock_preamble", "run_locked"]
