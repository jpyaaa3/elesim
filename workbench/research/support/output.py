"""Fresh output roots may be pre-created as empty Compose bind mounts."""
from pathlib import Path
from contextlib import contextmanager
import os


def fresh_directory(path):
    path = Path(path)
    if path.is_symlink() or (path.exists() and (not path.is_dir() or any(path.iterdir()))):
        raise FileExistsError(f"output is not an empty directory: {path}")
    path.mkdir(parents=True, exist_ok=True)
    return path


@contextmanager
def in_directory(path):
    previous = Path.cwd()
    try:
        os.chdir(path)
        yield
    finally:
        os.chdir(previous)
