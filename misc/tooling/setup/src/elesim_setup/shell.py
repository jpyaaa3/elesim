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


def managed_path_block(bin_dir: Path) -> str:
    value = str(bin_dir.expanduser().resolve())
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
    destination = (
        Path.home() / ".bashrc"
        if bashrc is None
        else bashrc.expanduser().resolve()
    )
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


def _atomic_write(path: Path, content: str) -> None:
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


__all__ = ["PathRegistration", "managed_path_block", "register_bash_path"]
