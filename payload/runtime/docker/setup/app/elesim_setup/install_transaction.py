"""Serialize file generation and roll back an unfinished fresh installation.

Existing installations keep their ownership boundary. This guard never adopts
or removes a path which existed when a fresh generation started.
"""

from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import os
from pathlib import Path
import shutil
import stat
from typing import Iterable, Iterator


@contextmanager
def installation_lock(prefix: Path, bin_dir: Path) -> Iterator[None]:
    """Serialize installations sharing either output root, before preflight."""
    root = Path('/tmp') / f'elesim-install-locks-{os.getuid()}'
    root.mkdir(mode=0o700, exist_ok=True)
    metadata = root.lstat()
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or metadata.st_uid != os.getuid()
        or stat.S_IMODE(metadata.st_mode) != 0o700
    ):
        raise RuntimeError(f"installer lock directory is not private: {root}")
    descriptors: list[int] = []
    try:
        for path in sorted({str(prefix.resolve()), str(bin_dir.resolve())}):
            digest = hashlib.sha256(os.fsencode(path)).hexdigest()
            fd = os.open(root / digest, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
            metadata = os.fstat(fd)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_uid != os.getuid():
                os.close(fd)
                raise RuntimeError("installer lock must be an owned singly-linked regular file")
            descriptors.append(fd)
            fcntl.flock(fd, fcntl.LOCK_EX)
        yield
    finally:
        for fd in reversed(descriptors):
            os.close(fd)


class FreshInstallRollback:
    """Remove only preflight-approved, initially absent generated paths.

    The caller holds installation_lock and supplies exact output paths. Prefix
    and bin parents are removed only when newly created and empty. Authority
    and logs are similarly empty-only: they may contain independent user data.
    A published ownership manifest is the commit boundary.
    """

    def __init__(
        self, *, prefix: Path, bin_dir: Path, generated_paths: Iterable[Path],
        enabled: bool,
    ) -> None:
        self.enabled = enabled
        self.committed = False
        self.paths: tuple[Path, ...] = ()
        self.empty_directories: tuple[Path, ...] = ()
        if not enabled:
            return
        paths = set(generated_paths)
        for path in paths:
            if path in (prefix, bin_dir) or not any(path.is_relative_to(root) for root in (prefix, bin_dir)):
                raise ValueError(f"fresh installation output is outside its exact roots: {path}")
            if os.path.lexists(path):
                raise FileExistsError(f"fresh installation output already exists: {path}")
        # Keep only outermost generated subtrees, never the enclosing prefix/bin.
        self.paths = tuple(sorted(
            (path for path in paths if not any(parent in paths for parent in path.parents)),
            key=lambda path: (len(path.parts), str(path)), reverse=True,
        ))
        directories = {prefix, bin_dir, prefix / 'authority', prefix / 'logs'}
        self.empty_directories = tuple(sorted(
            (path for path in directories if not os.path.lexists(path)),
            key=lambda path: (len(path.parts), str(path)), reverse=True,
        ))

    def __enter__(self) -> FreshInstallRollback:
        return self

    def commit(self) -> None:
        self.committed = True

    def __exit__(self, exc_type, exc, traceback) -> bool:
        if exc is None or not self.enabled or self.committed:
            return False
        errors = []
        for path in self.paths:
            try:
                # Never follow a replaced ancestor to delete an external tree.
                if any(parent.is_symlink() for parent in path.parents):
                    raise RuntimeError(f"rollback ancestor changed to a symlink: {path}")
                if path.is_symlink() or path.is_file():
                    path.unlink()
                elif path.is_dir():
                    shutil.rmtree(path)
                elif os.path.lexists(path):
                    raise RuntimeError(f"rollback output has an unexpected file type: {path}")
            except (OSError, RuntimeError) as error:
                errors.append(str(error))
        for path in self.empty_directories:
            try:
                if any(parent.is_symlink() for parent in path.parents):
                    errors.append(f"rollback ancestor changed to a symlink: {path}")
                    continue
                if path.is_dir() and not path.is_symlink():
                    path.rmdir()
            except OSError:
                # Parent directories and independent user material are not owned.
                pass
        if errors:
            raise RuntimeError(
                f"installation failed ({exc}); fresh output rollback was incomplete: "
                + '; '.join(errors)
            ) from exc
        return False
