"""Conservative shell integration owned by the setup wizard."""

from __future__ import annotations

import os
import shlex
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path


_START = "# >>> Elesim managed PATH >>>"
_END = "# <<< Elesim managed PATH <<<"


@dataclass(frozen=True)
class PathRegistration:
    changed: bool
    bashrc: Path
    backup: Path | None


@dataclass(frozen=True)
class PathUnregistration:
    """Result of conservatively removing one exact EleSim PATH block."""

    changed: bool
    matched: bool
    bashrc: Path
    backup: Path | None


def managed_path_block(bin_dir: Path) -> str:
    value_path = _lexical_path(bin_dir.expanduser())
    _reject_symlink_path(value_path, label="PATH directory")
    value = str(value_path)
    if "\n" in value or "\r" in value:
        raise ValueError("PATH directory cannot contain a line break")
    return (
        f"{_START}\n"
        f"export PATH={shlex.quote(value)}:\"$PATH\"\n"
        f"{_END}\n"
    )


def register_bash_path(
    bin_dir: Path,
    *,
    bashrc: Path | None = None,
) -> PathRegistration:
    destination = _bashrc_destination(bashrc)
    block = managed_path_block(bin_dir)
    original = destination.read_text(encoding="utf-8") if destination.exists() else ""
    updated = _replace_block(original, block)
    if updated == original:
        return PathRegistration(False, destination, None)

    destination.parent.mkdir(parents=True, exist_ok=True)
    backup: Path | None = None
    if destination.exists():
        backup = destination.with_name(f"{destination.name}.elesim.bak")
        if not backup.exists():
            shutil.copy2(destination, backup)
    _atomic_write(destination, updated)
    return PathRegistration(True, destination, backup)


def inspect_bash_path(
    bin_dir: Path,
    *,
    bashrc: Path | None = None,
) -> str:
    """Return ``exact``, ``absent`` or ``foreign`` for the managed block.

    An uninstall must never remove a block registered by a newer/different
    installation.  The complete block, including the resolved bin directory,
    therefore has to match byte-for-byte.
    """

    destination = _bashrc_destination(bashrc)
    if not destination.exists():
        return "absent"
    content = destination.read_text(encoding="utf-8")
    spans = _managed_spans(content)
    if not spans:
        return "absent"
    if len(spans) != 1:
        return "foreign"
    start, end = spans[0]
    return "exact" if content[start:end] == managed_path_block(bin_dir) else "foreign"


def unregister_bash_path(
    bin_dir: Path,
    *,
    bashrc: Path | None = None,
) -> PathUnregistration:
    """Remove only the exact block produced for ``bin_dir``.

    Missing or modified blocks are deliberately preserved.  Callers can use
    ``matched`` to distinguish an absent/foreign block from a successful
    removal without guessing from file contents.
    """

    destination = _bashrc_destination(bashrc)
    if not destination.exists():
        return PathUnregistration(False, False, destination, None)
    original = destination.read_text(encoding="utf-8")
    spans = _managed_spans(original)
    if len(spans) != 1:
        return PathUnregistration(False, False, destination, None)
    start, end = spans[0]
    if original[start:end] != managed_path_block(bin_dir):
        return PathUnregistration(False, False, destination, None)
    updated = original[:start] + original[end:]
    backup = destination.with_name(f"{destination.name}.elesim-uninstall.bak")
    if not backup.exists():
        shutil.copy2(destination, backup)
    _atomic_write(destination, updated)
    return PathUnregistration(True, True, destination, backup)


def _replace_block(content: str, block: str) -> str:
    start = content.find(_START)
    end = content.find(_END, start + len(_START)) if start >= 0 else -1
    if start >= 0 and end >= 0:
        end += len(_END)
        if end < len(content) and content[end] == "\n":
            end += 1
        return content[:start] + block + content[end:]
    prefix = content
    if prefix and not prefix.endswith("\n"):
        prefix += "\n"
    if prefix and not prefix.endswith("\n\n"):
        prefix += "\n"
    return prefix + block


def _managed_spans(content: str) -> list[tuple[int, int]]:
    spans: list[tuple[int, int]] = []
    offset = 0
    while True:
        start = content.find(_START, offset)
        if start < 0:
            return spans
        end = content.find(_END, start + len(_START))
        if end < 0:
            # A malformed/partial block is foreign and must be preserved.
            return [*spans, (start, len(content))]
        end += len(_END)
        if end < len(content) and content[end] == "\n":
            end += 1
        spans.append((start, end))
        offset = end


def _atomic_write(path: Path, content: str) -> None:
    _reject_symlink_path(path, label="shell file")
    mode = path.stat().st_mode & 0o777 if path.exists() else 0o644
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as handle:
        handle.write(content)
        temporary = Path(handle.name)
    temporary.chmod(mode)
    os.replace(temporary, path)


def write_executable(path: Path, content: str) -> None:
    """Atomically install one executable text file."""

    path = _lexical_path(path)
    _reject_symlink_path(path, label="executable")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.chmod(0o755)
    temporary.replace(path)


def operator_home() -> Path:
    """Resolve the operator HOME mount without accepting relative paths."""

    configured = os.environ.get("ELESIM_OPERATOR_HOME", "").strip()
    if configured:
        path = Path(configured).expanduser()
        if not path.is_absolute():
            raise ValueError("ELESIM_OPERATOR_HOME must be an absolute path")
        path = _lexical_path(path)
        _reject_symlink_path(path, label="operator HOME")
        return path
    path = _lexical_path(Path.home())
    _reject_symlink_path(path, label="operator HOME")
    return path


def _bashrc_destination(bashrc: Path | None) -> Path:
    destination = Path.home() / ".bashrc" if bashrc is None else bashrc.expanduser()
    destination = _lexical_path(destination)
    _reject_symlink_path(destination, label="bashrc")
    return destination


def _lexical_path(path: Path) -> Path:
    """Make an absolute path without resolving symlink components."""

    return Path(os.path.abspath(os.fspath(path)))


def _reject_symlink_path(path: Path, *, label: str) -> None:
    """Reject final and ancestor symlinks before a host-file write/read."""

    current = _lexical_path(path)
    while True:
        if current.is_symlink():
            raise ValueError(f"{label} path contains a symlink: {current}")
        parent = current.parent
        if parent == current:
            return
        current = parent


__all__ = [
    "PathRegistration",
    "PathUnregistration",
    "inspect_bash_path",
    "managed_path_block",
    "operator_home",
    "register_bash_path",
    "unregister_bash_path",
    "write_executable",
]
