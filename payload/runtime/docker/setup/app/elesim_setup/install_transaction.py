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
import tempfile
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


class RefreshControlRollback:
    """Restore management/configuration files if a refresh fails before commit.

    Generated build contexts may be partially refreshed and are regenerated on
    retry. Runtime caches, release pins, logs and credentials are not snapshots.
    The caller has already validated ownership and holds installation_lock.
    """

    def __init__(self, *, manifest: Path, paths: Iterable[Path]) -> None:
        self.manifest = manifest
        self.manifest_bytes = manifest.read_bytes()
        self.committed = False
        self.files: dict[Path, tuple[bytes, int] | None] = {}
        for path in sorted(set(paths), key=str):
            self._check_path(path)
            if not os.path.lexists(path):
                self.files[path] = None
                continue
            metadata = path.lstat()
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise RuntimeError(f"refresh control file is not singly-linked and regular: {path}")
            self.files[path] = (path.read_bytes(), stat.S_IMODE(metadata.st_mode))

    @staticmethod
    def _check_path(path: Path) -> None:
        if path.is_symlink() or any(parent.is_symlink() for parent in path.parents):
            raise RuntimeError(f"refresh control path contains a symlink: {path}")

    def __enter__(self) -> RefreshControlRollback:
        return self

    def commit(self) -> None:
        self.committed = True

    def __exit__(self, exc_type, exc, traceback) -> bool:
        if exc is None or self.committed:
            return False
        self._check_path(self.manifest)
        if self.manifest.read_bytes() != self.manifest_bytes:
            raise RuntimeError("refresh failed and ownership changed; refusing control-file rollback") from exc
        errors = []
        for path, previous in self.files.items():
            temporary = None
            try:
                self._check_path(path)
                if previous is None:
                    path.unlink(missing_ok=True)
                    continue
                content, mode = previous
                with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", delete=False) as handle:
                    temporary = Path(handle.name)
                    handle.write(content)
                    handle.flush()
                    os.fsync(handle.fileno())
                temporary.chmod(mode)
                os.replace(temporary, path)
            except OSError as error:
                errors.append(f"{path}: {error}")
            except RuntimeError as error:
                errors.append(str(error))
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
        if errors:
            raise RuntimeError(
                f"installation refresh failed ({exc}); control-file rollback was incomplete: "
                + "; ".join(errors)
            ) from exc
        return False
