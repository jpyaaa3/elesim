#!/usr/bin/env python3
"""Download an Elesim source archive and start the terminal setup wizard.

This file intentionally uses only the Python standard library. It can therefore
be piped directly from GitHub before Elesim or its dependencies are installed.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.error
import urllib.request
from pathlib import Path, PurePosixPath
from typing import BinaryIO, Sequence
from urllib.parse import quote


DEFAULT_REPOSITORY = "jpyaaa3/elesim"
DEFAULT_REF = "main"


class BootstrapError(RuntimeError):
    pass


def archive_url(repository: str, ref: str) -> str:
    repo = str(repository).strip().strip("/")
    revision = str(ref).strip()
    if repo.count("/") != 1 or not revision:
        raise BootstrapError("repository는 owner/name, ref는 비어 있지 않은 값이어야 합니다")
    return f"https://codeload.github.com/{repo}/tar.gz/{quote(revision, safe='')}"


def safe_extract_archive(archive: Path, destination: Path) -> Path:
    """Extract regular files/directories only and return the single source root."""

    destination.mkdir(parents=True, exist_ok=True)
    roots: set[str] = set()
    with tarfile.open(archive, mode="r:*") as bundle:
        members = bundle.getmembers()
        for member in members:
            path = PurePosixPath(member.name)
            if path.is_absolute() or ".." in path.parts or not path.parts:
                raise BootstrapError(f"unsafe archive member: {member.name!r}")
            if member.issym() or member.islnk() or member.isdev():
                raise BootstrapError(f"unsupported archive link/device: {member.name!r}")
            roots.add(path.parts[0])
        if len(roots) != 1:
            raise BootstrapError("source archive must contain exactly one top-level directory")

        for member in members:
            relative = PurePosixPath(member.name)
            target = destination.joinpath(*relative.parts)
            resolved = target.resolve()
            try:
                resolved.relative_to(destination.resolve())
            except ValueError as exc:
                raise BootstrapError(f"archive escaped destination: {member.name!r}") from exc
            if member.isdir():
                resolved.mkdir(parents=True, exist_ok=True)
                continue
            if not member.isfile():
                continue
            resolved.parent.mkdir(parents=True, exist_ok=True)
            source = bundle.extractfile(member)
            if source is None:
                raise BootstrapError(f"cannot read archive member: {member.name!r}")
            with source, resolved.open("wb") as handle:
                shutil.copyfileobj(source, handle)
            resolved.chmod(member.mode & 0o777)

    root = destination / next(iter(roots))
    if not (root / "misc/tooling/setup/pyproject.toml").is_file():
        raise BootstrapError("downloaded archive does not contain the Elesim setup package")
    return root


def download_source(
    url: str,
    cache_root: Path,
    *,
    refresh: bool = False,
) -> Path:
    cache_root = cache_root.expanduser().resolve()
    fingerprint = hashlib.sha256(url.encode("utf-8")).hexdigest()[:16]
    destination = cache_root / fingerprint
    marker = destination / ".elesim-source-complete"
    if marker.is_file() and not refresh:
        root_name = marker.read_text(encoding="utf-8").strip()
        root = destination / root_name
        if (root / "misc/tooling/setup/pyproject.toml").is_file():
            return root

    cache_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="elesim-download-", dir=cache_root) as td:
        temporary = Path(td)
        archive = temporary / "source.tar.gz"
        staging = temporary / "extract"
        print(f"[bootstrap] download {url}")
        try:
            with urllib.request.urlopen(url, timeout=60) as response, archive.open("wb") as handle:
                shutil.copyfileobj(response, handle)
        except (OSError, urllib.error.URLError) as exc:
            raise BootstrapError(f"source archive download failed: {exc}") from exc
        source_root = safe_extract_archive(archive, staging)
        root_name = source_root.name
        if destination.exists():
            shutil.rmtree(destination)
        destination.mkdir(parents=True)
        shutil.move(str(source_root), str(destination / root_name))
        marker.write_text(root_name + "\n", encoding="utf-8")
    return destination / root_name


def prepare_bootstrap_venv(source_root: Path, cache_root: Path) -> Path:
    fingerprint = hashlib.sha256(str(source_root).encode("utf-8")).hexdigest()[:16]
    venv = cache_root.expanduser().resolve() / f"venv-{fingerprint}"
    python = venv / "bin/python"
    if not python.is_file():
        print(f"[bootstrap] create venv {venv}")
        subprocess.run((sys.executable, "-m", "venv", str(venv)), check=True)
    commands = (
        (str(python), "-m", "pip", "--disable-pip-version-check", "install", "--upgrade", "pip", "setuptools>=68", "wheel"),
        (str(python), "-m", "pip", "--disable-pip-version-check", "install", "-r", str(source_root / "misc/tooling/setup/requirements.lock")),
        (str(python), "-m", "pip", "--disable-pip-version-check", "install", "--force-reinstall", "--no-deps", str(source_root / "packages/protocol")),
        (str(python), "-m", "pip", "--disable-pip-version-check", "install", "--force-reinstall", "--no-deps", str(source_root / "misc/tooling/setup")),
        (str(python), "-m", "pip", "--disable-pip-version-check", "check"),
    )
    for command in commands:
        subprocess.run(command, check=True)
    return venv / "bin/elesim-setup"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", default=os.environ.get("ELESIM_REPOSITORY", DEFAULT_REPOSITORY))
    parser.add_argument("--ref", default=os.environ.get("ELESIM_REF", DEFAULT_REF))
    parser.add_argument("--archive-url", default=os.environ.get("ELESIM_ARCHIVE_URL", ""))
    parser.add_argument("--cache-dir", default=os.environ.get("ELESIM_CACHE_DIR", "~/.cache/elesim/setup"))
    parser.add_argument("--refresh", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    if sys.version_info < (3, 10):
        print("오류: Python 3.10 이상이 필요합니다.", file=sys.stderr)
        return 2
    parser = _parser()
    args, wizard_args = parser.parse_known_args(argv)
    cache_root = Path(args.cache_dir).expanduser().resolve()
    url = args.archive_url or archive_url(args.repository, args.ref)
    try:
        source_root = download_source(url, cache_root / "sources", refresh=bool(args.refresh))
        executable = prepare_bootstrap_venv(source_root, cache_root)
        command = (str(executable), "--source-root", str(source_root), *wizard_args)
        tty: BinaryIO | None = None
        run_stdin: BinaryIO | int | None = None
        try:
            if not sys.stdin.isatty():
                try:
                    tty = open("/dev/tty", "rb", buffering=0)
                    run_stdin = tty
                except OSError as exc:
                    if "install" not in wizard_args:
                        raise BootstrapError(
                            "대화형 설치에는 controlling terminal이 필요합니다; "
                            "자동화에서는 install subcommand를 지정하십시오"
                        ) from exc
                    run_stdin = subprocess.DEVNULL
            completed = subprocess.run(command, stdin=run_stdin, check=False)
        finally:
            if tty is not None:
                tty.close()
        return int(completed.returncode)
    except (BootstrapError, OSError, subprocess.CalledProcessError, tarfile.TarError) as exc:
        print(f"부트스트랩 오류: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
