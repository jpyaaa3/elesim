from __future__ import annotations

import io
import tarfile
from pathlib import Path

import pytest

from misc.setup.bootstrap import (
    BootstrapError,
    archive_url,
    download_source,
    needs_controlling_terminal,
    safe_extract_archive,
)


def _archive(path: Path, members: dict[str, bytes]) -> None:
    with tarfile.open(path, "w:gz") as bundle:
        for name, payload in members.items():
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            info.mode = 0o644
            bundle.addfile(info, io.BytesIO(payload))


def test_archive_url_uses_configurable_repository_and_ref() -> None:
    assert archive_url("owner/project", "release/v1") == (
        "https://codeload.github.com/owner/project/tar.gz/release%2Fv1"
    )


def test_safe_extract_returns_valid_source_root(tmp_path: Path) -> None:
    archive = tmp_path / "source.tgz"
    _archive(
        archive,
        {
            "elesim-main/misc/tooling/setup/pyproject.toml": b"[project]\n",
            "elesim-main/packages/protocol/pyproject.toml": b"[project]\n",
        },
    )
    root = safe_extract_archive(archive, tmp_path / "out")
    assert root.name == "elesim-main"
    assert (root / "misc/tooling/setup/pyproject.toml").is_file()


def test_safe_extract_rejects_path_traversal(tmp_path: Path) -> None:
    archive = tmp_path / "bad.tgz"
    _archive(archive, {"../escape": b"bad"})
    with pytest.raises(BootstrapError, match="unsafe"):
        safe_extract_archive(archive, tmp_path / "out")
    assert not (tmp_path / "escape").exists()


def test_download_source_reuses_completed_cache(tmp_path: Path) -> None:
    archive = tmp_path / "source.tgz"
    _archive(
        archive,
        {"elesim-main/misc/tooling/setup/pyproject.toml": b"[project]\n"},
    )
    cache = tmp_path / "cache"
    first = download_source(archive.as_uri(), cache)
    archive.unlink()
    second = download_source(archive.as_uri(), cache)
    assert second == first


def test_gui_and_automation_do_not_require_a_controlling_terminal() -> None:
    assert needs_controlling_terminal(("gui",)) is False
    assert needs_controlling_terminal(("install", "--profile", "compute")) is False
    assert needs_controlling_terminal(("status",)) is False
    assert needs_controlling_terminal(("wizard",)) is True


def test_container_bootstrap_preserves_host_python_and_uses_compose_v2() -> None:
    script = (Path(__file__).resolve().parents[3] / "setup/bootstrap.sh").read_text(
        encoding="utf-8"
    )
    assert "python:3.10-slim" in script
    assert '"${docker_cmd[@]}" compose version' in script
    assert "pip install" not in script
    assert 'docker_args+=(--publish "127.0.0.1:${gui_port}:${gui_port}")' in script
    assert '--workdir "$invocation_dir"' in script
    assert '"ELESIM_HOST_ARCH=$host_arch"' in script
    assert '"ELESIM_HOST_WSLG=$host_wslg"' in script
    assert "port_is_in_use" in script
    assert "selected another available port" in script
    assert "gui_arguments=(gui)" in script
    assert "wizard|install|status)" in script
    assert 'if [[ "$argument" != "gui" ]]' in script
    assert 'python /tmp/elesim-bootstrap.py "$@"' in script
