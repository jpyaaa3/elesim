"""Content-addressed run evidence; no deletion or inferred historical passes."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import zipfile


def digest(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def inventory(root: Path) -> dict:
    if root.is_symlink():
        raise ValueError(f"evidence root cannot be a symlink: {root}")
    if root.is_file():
        return {".": {"bytes": root.stat().st_size, "sha256": digest(root)}}
    entries = {}
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"evidence cannot contain symlinks: {path}")
        if path.is_file():
            entries[path.relative_to(root).as_posix()] = {
                "bytes": path.stat().st_size, "sha256": digest(path)}
    return entries


def config_sources(path: Path) -> tuple[Path, ...]:
    """Follow application YAML extends without importing a deployment on the host."""
    import yaml
    sources = []
    current = path.resolve()
    while True:
        if current in sources or len(sources) >= 64:
            raise ValueError("configuration extends cycle or excessive depth")
        sources.append(current)
        document = yaml.safe_load(current.read_text())
        if not isinstance(document, dict):
            raise ValueError("application configuration must be a mapping")
        parent = document.get("extends")
        if parent is None:
            return tuple(sources)
        if not isinstance(parent, str) or not parent.strip():
            raise ValueError("configuration extends must be a path or null")
        current = (current.parent / parent).resolve()


def artifacts(run: Path) -> dict:
    entries = {}
    for name in ("plan.json", "logs", "work", "control"):
        path = run / name
        if not path.exists():
            continue
        for relative, record in inventory(path).items():
            entries[name if relative == "." else f"{name}/{relative}"] = record
    return entries


def check_artifacts(run: Path, manifest: dict) -> None:
    expected = manifest.get("artifacts")
    if not isinstance(expected, dict) or not expected or artifacts(run) != expected:
        raise ValueError("run artifacts changed, are missing, or contain unrecorded files")


def pack(run: Path, output: Path) -> dict:
    from .runner import verify
    # A failed benchmark is valid evidence and can be packed; a changed one cannot.
    verify(run)
    if output.resolve().is_relative_to(run.resolve()):
        raise ValueError("archive must be outside the run directory")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as stream, zipfile.ZipFile(stream, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(run.rglob("*")):
            if path.is_symlink():
                raise ValueError("cannot archive symlinks")
            if path.is_file():
                archive.write(path, path.relative_to(run).as_posix())
    return {"file": str(output), "bytes": output.stat().st_size, "sha256": digest(output)}


def unpack(archive_path: Path, destination: Path, expected_sha256: str) -> dict:
    if digest(archive_path) != expected_sha256:
        raise ValueError("archive SHA256 mismatch")
    if destination.exists():
        raise ValueError("collection destination already exists")
    with zipfile.ZipFile(archive_path) as archive:
        files = archive.infolist()
        if len(files) > 100000 or sum(f.file_size for f in files) > 64 * 1024**3:
            raise ValueError("archive exceeds evidence collection limits")
        names = set()
        for item in files:
            name = PurePosixPath(item.filename)
            if (name.is_absolute() or ".." in name.parts or "\\" in item.filename or
                    name.as_posix() in names or (item.external_attr >> 16) & 0o170000 == 0o120000):
                raise ValueError("unsafe or duplicate archive entry")
            names.add(name.as_posix())
        destination.mkdir(parents=True)
        archive.extractall(destination)
    from .runner import verify
    return verify(destination)


def index(root: Path) -> dict:
    """Index legacy runs and copied code without moving or reclassifying raw files."""
    runs, copies, managed = [], {}, []
    for folder in sorted(root.iterdir()):
        if folder.is_symlink() or not folder.is_dir():
            continue
        count = size = 0
        for base, dirs, files in os.walk(folder, followlinks=False):
            dirs[:] = sorted(d for d in dirs if not (Path(base) / d).is_symlink())
            for name in sorted(files):
                path = Path(base) / name
                if path.is_symlink():
                    continue
                count += 1
                size += path.stat().st_size
                if name == "manifest.json" and path.stat().st_size < 10 * 1024**2:
                    try:
                        data = json.loads(path.read_text())
                        if isinstance(data, dict) and data.get("schema_version") == 1 and "benchmark" in data and "stages" in data:
                            managed.append({"path": path.parent.relative_to(root).as_posix(),
                                "benchmark": data["benchmark"], "declared_complete": data.get("complete"),
                                "declared_accepted": data.get("accepted"), "artifact_verification": "not_run"})
                    except (ValueError, OSError):
                        pass
                if path.suffix in (".py", ".sh"):
                    copies.setdefault(digest(path), []).append(path.relative_to(root).as_posix())
        runs.append({"path": folder.name, "files": count, "bytes": size,
                     "classification": "contains-managed-runs" if any(
                         row["path"] == folder.name or row["path"].startswith(folder.name + "/") for row in managed)
                         else "legacy-unverified"})
    return {"schema_version": 1, "root": str(root.resolve()), "runs": runs,
            "code_files": sum(map(len, copies.values())), "distinct_code_contents": len(copies),
            "duplicate_code": {sha: paths for sha, paths in copies.items() if len(paths) > 1}, "managed_runs": managed,
            "policy": "raw evidence retained; historical directories do not imply acceptance"}
