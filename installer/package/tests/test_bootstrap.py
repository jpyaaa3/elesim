from __future__ import annotations

import concurrent.futures
import hashlib
import http.client
import io
import json
import os
import subprocess
import tarfile
import threading
import urllib.error
import urllib.request
from pathlib import Path, PurePosixPath

import pytest

from installer.bootstrap import bootstrap as bootstrap_module
from installer.bootstrap.bootstrap import (
    BootstrapError,
    _atomic_write_json,
    _ensure_bootstrap_pip,
    _snapshot_root,
    archive_url,
    download_source,
    needs_controlling_terminal,
    safe_extract_archive,
    setup_arguments,
    validate_bootstrap_contract,
    validate_bootstrap_generation,
)


def _archive(
    path: Path,
    members: dict[str, bytes],
    *,
    commit: str | None = None,
) -> None:
    options: dict[str, object] = {}
    if commit is not None:
        options = {
            "format": tarfile.PAX_FORMAT,
            "pax_headers": {"comment": commit},
        }
    with tarfile.open(path, "w:gz", **options) as bundle:
        for name, payload in members.items():
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            info.mode = 0o644
            bundle.addfile(info, io.BytesIO(payload))


def _archive_payload(
    tmp_path: Path,
    *,
    root: str = "elesim-main",
    project: bytes = b'[project]\nversion = "0.2.1"\n',
    commit: str | None = None,
    extra_members: dict[str, bytes] | None = None,
) -> bytes:
    archive = tmp_path / f"{root}.tgz"
    members = {
        f"{root}/{relative}": payload
        for relative, payload in _minimal_snapshot_members(project=project).items()
    }
    members.update(
        {f"{root}/{name}": payload for name, payload in (extra_members or {}).items()}
    )
    _archive(
        archive,
        members,
        commit=commit,
    )
    return archive.read_bytes()


def _minimal_snapshot_members(*, project: bytes = b"[project]\n") -> dict[str, bytes]:
    members: dict[str, bytes] = {
        "installer/package/pyproject.toml": project,
        "installer/package/requirements.lock": b"",
        "installer/package/src/elesim_setup/__init__.py": b"",
        "installer/package/src/elesim_setup/cli.py": b"",
        "installer/package/src/elesim_setup/network.py": b"",
        "installer/package/src/elesim_setup/connections.py": b"",
        "installer/package/src/elesim_setup/uninstall.py": b"",
        "installer/package/src/elesim_setup/host_proxy.py": b"",
        "packages/protocol/pyproject.toml": b"[project]\n",
        "packages/protocol/src/elesim_protocol/__init__.py": b"",
        "packages/elesim_interfaces/CMakeLists.txt": (
            b"rosidl_generate_interfaces(${PROJECT_NAME}\n"
            b'  "msg/RgbdFrame.msg"\n'
            b'  "srv/OpenSimulationSession.srv"\n'
            b'  "action/RunOperatorWorkflow.action"\n'
            b")\n"
        ),
        "packages/elesim_interfaces/package.xml": b"",
        "packages/elesim_interfaces/msg/RgbdFrame.msg": b"",
        "packages/elesim_interfaces/srv/OpenSimulationSession.srv": b"",
        "packages/elesim_interfaces/action/RunOperatorWorkflow.action": b"",
        "installer/bootstrap/bootstrap.py": b"# bootstrap\n",
        "installer/bootstrap/install.sh": b"#!/bin/sh\n",
        "installer/bootstrap/bootstrap-contract.json": json.dumps(
            {
                "schema_version": 1,
                "bootstrap_api": 1,
                "required_commands": ["wizard", "gui", "install", "update", "status"],
            }
        ).encode("utf-8"),
        "environment/containers/Dockerfile.app": b"FROM scratch\n",
        "environment/containers/Dockerfile.tools": b"FROM scratch\n",
        "environment/containers/tools-entrypoint": b"#!/bin/sh\n",
        "environment/containers/robotpkg.asc": b"public key\n",
        "environment/development/Dockerfile": b"FROM scratch\n",
        "environment/development/requirements.lock": b"",
        "environment/development/entrypoint.sh": b"#!/bin/sh\n",
        "environment/development/dev-env.sh": b"#!/bin/sh\n",
        "model/bundles/default/bundle.json": b"{}\n",
        "model/bundles/d435/bundle.json": b"{}\n",
    }
    for role in ("pilot", "sim", "ui", "robot"):
        members[f"{role}/pyproject.toml"] = b"[project]\n"
        members[f"{role}/requirements.lock"] = b""
        members[f"{role}/src/elesim_{role}/__init__.py"] = b""
    for relative in bootstrap_module._BOOTSTRAP_SETUP_PYTHON_FILES:
        members.setdefault(relative.as_posix(), b"")
    for relative in bootstrap_module._BOOTSTRAP_PROTOCOL_PYTHON_FILES:
        members.setdefault(relative.as_posix(), b"")
    for relative in bootstrap_module._BOOTSTRAP_ROLE_ENTRYPOINT_FILES:
        members.setdefault(relative.as_posix(), b"")
    for relative in bootstrap_module._BOOTSTRAP_ROLE_CONFIG_FILES:
        members.setdefault(relative.as_posix(), b"{}\n")
    return members


class _Response(io.BytesIO):
    def __init__(
        self,
        payload: bytes = b"",
        *,
        status: int = 200,
        headers: dict[str, str] | None = None,
    ) -> None:
        super().__init__(payload)
        self.status = status
        self.headers = headers or {}

    def getcode(self) -> int:
        return self.status


class _BrokenResponse(_Response):
    def read(self, size: int = -1) -> bytes:
        raise http.client.IncompleteRead(b"partial", 1024)


class _URLSequence:
    def __init__(self, *responses: _Response | BaseException) -> None:
        self.responses = list(responses)
        self.requests: list[urllib.request.Request] = []

    def __call__(
        self,
        request: urllib.request.Request,
        *,
        timeout: int,
    ) -> _Response:
        assert timeout == 60
        self.requests.append(request)
        if not self.responses:
            raise AssertionError("unexpected URL request")
        response = self.responses.pop(0)
        if isinstance(response, BaseException):
            raise response
        return response


def _headers(request: urllib.request.Request) -> dict[str, str]:
    return {key.lower(): value for key, value in request.header_items()}


def _write_valid_snapshot(snapshot: Path, root_name: str = "elesim-main") -> None:
    root = snapshot / root_name
    for relative, payload in _minimal_snapshot_members().items():
        destination = root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(payload)
    (snapshot / ".elesim-source-complete").write_text(root_name + "\n")


def test_source_snapshot_allows_explicitly_excluded_public_examples(
    tmp_path: Path,
) -> None:
    snapshot = tmp_path / "snapshot"
    _write_valid_snapshot(snapshot)
    root = snapshot / "elesim-main"
    for relative in bootstrap_module._BOOTSTRAP_EXCLUDED_CONFIG_FILES:
        destination = root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text("public: true\n", encoding="utf-8")

    bootstrap_module._validate_source_snapshot(root)


@pytest.mark.parametrize(
    "relative",
    (
        "installer/package/src/elesim_setup/cli.py",
        "installer/package/src/elesim_setup/network.py",
        "installer/package/src/elesim_setup/connections.py",
        "installer/package/src/elesim_setup/uninstall.py",
        "installer/package/src/elesim_setup/host_proxy.py",
        "installer/package/src/elesim_setup/ownership.py",
        "installer/package/src/elesim_setup/runtime_status.py",
        "installer/package/src/elesim_setup/shell.py",
    ),
)
def test_source_snapshot_requires_every_setup_console_target(
    tmp_path: Path,
    relative: str,
) -> None:
    snapshot = tmp_path / "snapshot"
    _write_valid_snapshot(snapshot)
    root = snapshot / "elesim-main"
    (root / relative).unlink()

    with pytest.raises(BootstrapError, match=Path(relative).name):
        bootstrap_module._validate_source_snapshot(root)


def test_source_snapshot_rejects_unowned_setup_python_module(
    tmp_path: Path,
) -> None:
    snapshot = tmp_path / "snapshot"
    _write_valid_snapshot(snapshot)
    root = snapshot / "elesim-main"
    (root / "installer/package/src/elesim_setup/dummy.py").write_text(
        "", encoding="utf-8"
    )

    with pytest.raises(BootstrapError, match="unexpected setup Python"):
        bootstrap_module._validate_source_snapshot(root)


def test_source_snapshot_rejects_nested_setup_python_package(
    tmp_path: Path,
) -> None:
    snapshot = tmp_path / "snapshot"
    _write_valid_snapshot(snapshot)
    root = snapshot / "elesim-main"
    rogue = root / "installer/package/src/elesim_setup/rogue"
    rogue.mkdir()
    (rogue / "__init__.py").write_text("", encoding="utf-8")
    (rogue / "payload.py").write_text("", encoding="utf-8")

    with pytest.raises(BootstrapError, match="unexpected setup Python"):
        bootstrap_module._validate_source_snapshot(root)


@pytest.mark.parametrize(
    "relative",
    (
        "pilot/src/elesim_pilot/main.py",
        "robot/src/elesim_robot/go2/unitree_bridge_daemon.py",
        "pilot/config/runtime.yaml",
        "sim/config/runtime.yaml",
    ),
)
def test_source_snapshot_requires_role_entrypoints_and_runtime_configs(
    tmp_path: Path,
    relative: str,
) -> None:
    snapshot = tmp_path / "snapshot"
    _write_valid_snapshot(snapshot)
    root = snapshot / "elesim-main"
    (root / relative).unlink()

    with pytest.raises(BootstrapError, match=Path(relative).name):
        bootstrap_module._validate_source_snapshot(root)


def test_source_snapshot_rejects_unowned_protocol_python_module(
    tmp_path: Path,
) -> None:
    snapshot = tmp_path / "snapshot"
    _write_valid_snapshot(snapshot)
    root = snapshot / "elesim-main"
    (root / "packages/protocol/src/elesim_protocol/dummy.py").write_text(
        "", encoding="utf-8"
    )

    with pytest.raises(BootstrapError, match="unexpected protocol Python"):
        bootstrap_module._validate_source_snapshot(root)


def test_protocol_tracing_module_is_owned_by_bootstrap_manifest() -> None:
    assert (
        PurePosixPath("packages/protocol/src/elesim_protocol/tracing.py")
        in bootstrap_module._BOOTSTRAP_PROTOCOL_PYTHON_FILES
    )


def test_sim_mock_object_is_owned_by_bootstrap_manifest() -> None:
    assert (
        PurePosixPath("sim/config/mock_objects/demo_box.obj")
        in bootstrap_module._BOOTSTRAP_ROLE_CONFIG_FILES
    )


@pytest.mark.parametrize(
    "relative",
    (
        "msg/Extra.msg",
        "srv/Extra.srv",
        "action/Extra.action",
    ),
)
def test_source_snapshot_requires_every_cmake_declared_rosidl_source(
    tmp_path: Path,
    relative: str,
) -> None:
    snapshot = tmp_path / "snapshot"
    _write_valid_snapshot(snapshot)
    root = snapshot / "elesim-main"
    interfaces = root / "packages/elesim_interfaces"
    (interfaces / "CMakeLists.txt").write_text(
        "rosidl_generate_interfaces(${PROJECT_NAME}\n"
        '  "msg/RgbdFrame.msg"\n'
        '  "srv/OpenSimulationSession.srv"\n'
        '  "action/RunOperatorWorkflow.action"\n'
        '  "msg/Extra.msg"\n'
        '  "srv/Extra.srv"\n'
        '  "action/Extra.action"\n'
        ")\n",
        encoding="utf-8",
    )
    for extra in ("msg/Extra.msg", "srv/Extra.srv", "action/Extra.action"):
        source = interfaces / extra
        source.parent.mkdir(exist_ok=True)
        source.write_text("", encoding="utf-8")
    (interfaces / relative).unlink()

    with pytest.raises(BootstrapError, match=Path(relative).name):
        bootstrap_module._validate_source_snapshot(root)


def test_archive_url_uses_configurable_repository_and_ref() -> None:
    assert archive_url("owner/project", "release/v1") == (
        "https://codeload.github.com/owner/project/tar.gz/release%2Fv1"
    )


def test_safe_extract_returns_valid_source_root(tmp_path: Path) -> None:
    archive = tmp_path / "source.tgz"
    _archive(
        archive,
        {
            "elesim-main/installer/package/pyproject.toml": b"[project]\n",
            "elesim-main/packages/protocol/pyproject.toml": b"[project]\n",
        },
    )
    root = safe_extract_archive(archive, tmp_path / "out")
    assert root.name == "elesim-main"
    assert (root / "installer/package/pyproject.toml").is_file()


def test_safe_extract_ignores_links_outside_install_source_boundary(
    tmp_path: Path,
) -> None:
    archive = tmp_path / "source-with-log-link.tgz"
    with tarfile.open(archive, "w:gz") as bundle:
        root = "elesim-main"
        for name, payload in {
            f"{root}/installer/package/pyproject.toml": b"[project]\n",
            f"{root}/packages/protocol/pyproject.toml": b"[project]\n",
        }.items():
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            info.mode = 0o644
            bundle.addfile(info, io.BytesIO(payload))
        link = tarfile.TarInfo(f"{root}/log/latest")
        link.type = tarfile.SYMTYPE
        link.linkname = "latest_version-check"
        bundle.addfile(link)

    extracted = safe_extract_archive(archive, tmp_path / "out")

    assert extracted.name == "elesim-main"
    assert not (extracted / "log").exists()


def test_safe_extract_rejects_links_inside_install_source_boundary(
    tmp_path: Path,
) -> None:
    archive = tmp_path / "source-with-source-link.tgz"
    with tarfile.open(archive, "w:gz") as bundle:
        link = tarfile.TarInfo(
            "elesim-main/installer/package/src/elesim_setup/connections.py"
        )
        link.type = tarfile.SYMTYPE
        link.linkname = "other.py"
        bundle.addfile(link)

    with pytest.raises(BootstrapError, match="unsupported archive link/device"):
        safe_extract_archive(archive, tmp_path / "out")


def test_safe_extract_rejects_path_traversal(tmp_path: Path) -> None:
    archive = tmp_path / "bad.tgz"
    _archive(archive, {"../escape": b"bad"})
    with pytest.raises(BootstrapError, match="unsafe"):
        safe_extract_archive(archive, tmp_path / "out")
    assert not (tmp_path / "escape").exists()


def test_download_source_uses_full_url_hash_and_ignores_legacy_cache(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    url = "https://archives.example/elesim.tar.gz?signature=secret"
    legacy = tmp_path / "sources" / hashlib.sha256(url.encode()).hexdigest()[:16]
    legacy_root = legacy / "elesim-old"
    (legacy_root / "installer/package").mkdir(parents=True)
    (legacy_root / "installer/package/pyproject.toml").write_text("[project]\n")
    (legacy / ".elesim-source-complete").write_text("elesim-old\n")
    opener = _URLSequence(_Response(_archive_payload(tmp_path)))
    monkeypatch.setattr(urllib.request, "urlopen", opener)

    root = download_source(url, tmp_path)

    assert len(opener.requests) == 1
    assert root != legacy_root
    assert root.is_relative_to(
        tmp_path / "sources-v2" / hashlib.sha256(url.encode()).hexdigest()
    )
    assert legacy_root.is_dir()
    assert url not in capsys.readouterr().out
    assert "signature=secret" not in root.parents[2].joinpath("current.json").read_text()


def test_download_source_caches_only_install_source_boundary(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = _archive_payload(
        tmp_path,
        extra_members={
            "environment/development/Dockerfile": b"FROM ubuntu:22.04\n",
            "environment/development/requirements.lock": b"pytest==8.4.2\n",
            "environment/development/entrypoint.sh": b"#!/bin/sh\n",
            "environment/development/dev-env.sh": b"#!/bin/sh\n",
            "environment/containers/robotpkg.asc": b"public key\n",
            "installer/package/src/elesim_setup/__init__.py": b"",
            "packages/protocol/src/elesim_protocol/__init__.py": b"",
            "packages/elesim_interfaces/action/RunOperatorWorkflow.action": b"",
            "pilot/src/elesim_pilot/main.py": b"def main(): pass\n",
            "pilot/config/perception/detector.yolo.example.json": b"{}\n",
            "pilot/config/runtime.public.example.yaml": b"public: true\n",
            "sim/config/runtime.public.example.yaml": b"public: true\n",
            "ui/config/public.example.yaml": b"public: true\n",
            "robot/config/public.example.yaml": b"public: true\n",
            "model/bundles/default/bundle.json": b"{}\n",
            "model/bundles/d435/bundle.json": b"{}\n",
            "pilot/tests/test_dummy.py": b"raise AssertionError\n",
            "installer/package/tests/test_dummy.py": b"raise AssertionError\n",
            "misc/research/dummy.bin": b"research-only\n",
        },
    )
    monkeypatch.setattr(
        urllib.request,
        "urlopen",
        _URLSequence(_Response(payload)),
    )

    root = download_source("https://archives.example/elesim.tar.gz", tmp_path)

    assert (root / "environment/development/Dockerfile").is_file()
    assert (root / "environment/development/requirements.lock").is_file()
    assert (root / "environment/development/entrypoint.sh").is_file()
    assert (root / "environment/development/dev-env.sh").is_file()
    assert (root / "environment/containers/robotpkg.asc").is_file()
    assert (root / "installer/package/src/elesim_setup/__init__.py").is_file()
    assert (root / "packages/protocol/src/elesim_protocol/__init__.py").is_file()
    assert (
        root / "packages/elesim_interfaces/action/RunOperatorWorkflow.action"
    ).is_file()
    assert (root / "pilot/src/elesim_pilot/main.py").is_file()
    assert (root / "pilot/config/perception/detector.yolo.example.json").is_file()
    assert not (root / "pilot/config/runtime.public.example.yaml").exists()
    assert not (root / "sim/config/runtime.public.example.yaml").exists()
    assert not (root / "ui/config/public.example.yaml").exists()
    assert not (root / "robot/config/public.example.yaml").exists()
    assert (root / "model/bundles/default/bundle.json").is_file()
    assert (root / "model/bundles/d435/bundle.json").is_file()
    assert not (root / "pilot/tests").exists()
    assert not (root / "installer/package/tests").exists()
    assert not (root / "misc/research").exists()


def test_download_source_validates_completed_snapshot_with_conditional_get(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = _archive_payload(tmp_path)
    opener = _URLSequence(
        _Response(
            payload,
            headers={
                "ETag": '"revision-1"',
                "Last-Modified": "Fri, 24 Jul 2026 00:00:00 GMT",
            },
        ),
        _Response(status=304),
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://archives.example/elesim.tar.gz"

    first = download_source(url, tmp_path)
    second = download_source(url, tmp_path)

    assert second == first
    assert _headers(opener.requests[0]) == {}
    assert _headers(opener.requests[1]) == {
        "if-none-match": '"revision-1"',
        "if-modified-since": "Fri, 24 Jul 2026 00:00:00 GMT",
    }


def test_download_source_replaces_legacy_snapshot_outside_install_boundary(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = _archive_payload(tmp_path)
    opener = _URLSequence(
        _Response(payload, headers={"ETag": '"revision-1"'}),
        _Response(status=304),
        _Response(payload, headers={"ETag": '"revision-1"'}),
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://archives.example/elesim.tar.gz"
    first = download_source(url, tmp_path)
    legacy = first / "misc/research/legacy.txt"
    legacy.parent.mkdir(parents=True)
    legacy.write_text("source-only\n", encoding="utf-8")
    (first / "tests").mkdir()

    second = download_source(url, tmp_path)

    assert second == first
    assert not (second / "misc").exists()
    assert not (second / "tests").exists()
    assert _headers(opener.requests[1]) == {"if-none-match": '"revision-1"'}
    assert _headers(opener.requests[2]) == {}


def test_download_source_handles_real_urllib_304_and_closes_it(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    url = "https://archives.example/elesim.tar.gz"
    error_body = io.BytesIO()
    not_modified = urllib.error.HTTPError(
        url,
        304,
        "Not Modified",
        {"ETag": '"revision-1"'},
        error_body,
    )
    opener = _URLSequence(
        _Response(_archive_payload(tmp_path), headers={"ETag": '"revision-1"'}),
        not_modified,
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)

    first = download_source(url, tmp_path)
    second = download_source(url, tmp_path)

    assert second == first
    assert error_body.closed


def test_download_source_wraps_interrupted_response_body(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    opener = _URLSequence(_BrokenResponse())
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://archives.example/elesim.tar.gz"

    with pytest.raises(BootstrapError, match="IncompleteRead"):
        download_source(url, tmp_path)

    cache = tmp_path / "sources-v2" / hashlib.sha256(url.encode()).hexdigest()
    assert not (cache / "current.json").exists()


def test_download_source_publishes_changed_codeload_commit(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    first_commit = "1" * 40
    second_commit = "2" * 40
    opener = _URLSequence(
        _Response(
            _archive_payload(
                tmp_path,
                root="elesim-one",
                project=b'[project]\nversion = "0.2.0"\n',
                commit=first_commit,
            ),
            headers={"ETag": '"one"'},
        ),
        _Response(
            _archive_payload(
                tmp_path,
                root="elesim-two",
                project=b'[project]\nversion = "0.2.1"\n',
                commit=second_commit,
            ),
            headers={"ETag": '"two"'},
        ),
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://codeload.github.com/owner/elesim/tar.gz/refactoring"

    first = download_source(url, tmp_path, ref="refactoring")
    second = download_source(url, tmp_path, ref="refactoring")

    assert first.parent.name == f"git-{first_commit}"
    assert second.parent.name == f"git-{second_commit}"
    assert first != second
    assert _headers(opener.requests[1])["if-none-match"] == '"one"'


def test_download_source_rejects_incomplete_same_revision_archive(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    commit = "1" * 40
    valid = _archive_payload(tmp_path, root="elesim-valid", commit=commit)
    invalid_path = tmp_path / "invalid.tgz"
    _archive(
        invalid_path,
        {"elesim-invalid/installer/package/pyproject.toml": b"[project]\n"},
        commit=commit,
    )
    opener = _URLSequence(
        _Response(valid, headers={"ETag": '"one"'}),
        _Response(invalid_path.read_bytes(), headers={"ETag": '"two"'}),
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://codeload.github.com/owner/elesim/tar.gz/refactoring"
    first = download_source(url, tmp_path, ref="refactoring")

    with pytest.raises(BootstrapError, match="missing required setup files"):
        download_source(url, tmp_path, ref="refactoring")

    assert first.is_dir()


def test_download_source_fails_closed_instead_of_using_stale_snapshot(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    opener = _URLSequence(
        _Response(_archive_payload(tmp_path), headers={"ETag": '"one"'}),
        urllib.error.URLError("network unavailable"),
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://archives.example/elesim.tar.gz"
    cached = download_source(url, tmp_path)

    with pytest.raises(BootstrapError, match="network unavailable"):
        download_source(url, tmp_path)

    assert cached.is_dir()


def test_download_source_refresh_uses_unconditional_request(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = _archive_payload(tmp_path)
    opener = _URLSequence(
        _Response(payload, headers={"ETag": '"one"'}),
        _Response(payload, headers={"ETag": '"two"'}),
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://archives.example/elesim.tar.gz"
    download_source(url, tmp_path)

    download_source(url, tmp_path, refresh=True)

    assert _headers(opener.requests[1]) == {}


def test_download_source_refresh_repairs_existing_snapshot(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = _archive_payload(tmp_path)
    opener = _URLSequence(_Response(payload), _Response(payload))
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://archives.example/elesim.tar.gz"
    first = download_source(url, tmp_path)
    contract = first / "installer/bootstrap/bootstrap-contract.json"
    contract.unlink()

    refreshed = download_source(url, tmp_path, refresh=True)

    assert refreshed == first
    assert contract.is_file()


def test_download_source_without_validators_downloads_on_every_run(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = _archive_payload(tmp_path)
    opener = _URLSequence(_Response(payload), _Response(payload))
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://archives.example/elesim.tar.gz"

    first = download_source(url, tmp_path)
    first_inode = first.stat().st_ino
    second = download_source(url, tmp_path)

    assert second == first
    assert second.stat().st_ino == first_inode
    assert len(opener.requests) == 2
    assert _headers(opener.requests[1]) == {}


def test_download_source_reuses_full_commit_without_network(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    commit = "a" * 40
    opener = _URLSequence(
        _Response(
            _archive_payload(tmp_path, commit=commit),
            headers={"ETag": '"one"'},
        )
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = f"https://codeload.github.com/owner/elesim/tar.gz/{commit}"

    first = download_source(url, tmp_path, ref=commit)
    second = download_source(url, tmp_path, ref=commit)

    assert second == first
    assert len(opener.requests) == 1


def test_download_source_rejects_wrong_archive_for_immutable_commit(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    requested = "a" * 40
    returned = "b" * 40
    opener = _URLSequence(
        _Response(_archive_payload(tmp_path, commit=returned))
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = f"https://codeload.github.com/owner/elesim/tar.gz/{requested}"

    with pytest.raises(BootstrapError, match="immutable commit"):
        download_source(url, tmp_path, ref=requested)


def test_download_source_refresh_rejects_unexpected_304(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    opener = _URLSequence(_Response(status=304))
    monkeypatch.setattr(urllib.request, "urlopen", opener)

    with pytest.raises(BootstrapError, match="without a conditional request"):
        download_source(
            "https://archives.example/elesim.tar.gz",
            tmp_path,
            refresh=True,
        )


def test_download_source_retries_unconditionally_after_304_for_incomplete_snapshot(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    first_payload = _archive_payload(tmp_path, root="elesim-one")
    second_payload = _archive_payload(tmp_path, root="elesim-two")
    opener = _URLSequence(
        _Response(first_payload, headers={"ETag": '"one"'}),
        _Response(status=304),
        _Response(second_payload, headers={"ETag": '"two"'}),
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://archives.example/elesim.tar.gz"
    first = download_source(url, tmp_path)
    (first / "installer/package/pyproject.toml").unlink()

    recovered = download_source(url, tmp_path)

    assert recovered.name == "elesim-two"
    assert _headers(opener.requests[1]) == {"if-none-match": '"one"'}
    assert _headers(opener.requests[2]) == {}


def test_download_source_ignores_corrupt_index_and_downloads_unconditionally(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = _archive_payload(tmp_path)
    opener = _URLSequence(
        _Response(payload, headers={"ETag": '"one"'}),
        _Response(payload, headers={"ETag": '"two"'}),
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://archives.example/elesim.tar.gz"
    download_source(url, tmp_path)
    index = (
        tmp_path
        / "sources-v2"
        / hashlib.sha256(url.encode()).hexdigest()
        / "current.json"
    )
    index.write_text("{not-json", encoding="utf-8")

    download_source(url, tmp_path)

    assert _headers(opener.requests[1]) == {}


@pytest.mark.parametrize("root_name", ("..", "/", "."))
def test_snapshot_root_rejects_cache_path_escape(
    tmp_path: Path,
    root_name: str,
) -> None:
    cache = tmp_path / "cache"
    revision = f"git-{'a' * 40}"
    snapshot = cache / "snapshots" / revision
    snapshot.mkdir(parents=True)
    (snapshot / ".elesim-source-complete").write_text(
        root_name + "\n",
        encoding="utf-8",
    )
    index = {
        "schema_version": 1,
        "revision": revision,
        "root_name": root_name,
        "archive_sha256": "0" * 64,
    }

    assert _snapshot_root(cache, index) is None


def test_snapshot_root_rejects_symlinked_revision_directory(tmp_path: Path) -> None:
    cache = tmp_path / "cache"
    revision = f"git-{'a' * 40}"
    root_name = "elesim-main"
    outside_snapshot = tmp_path / "outside"
    _write_valid_snapshot(outside_snapshot, root_name)
    snapshots = cache / "snapshots"
    snapshots.mkdir(parents=True)
    (snapshots / revision).symlink_to(outside_snapshot, target_is_directory=True)
    index = {
        "schema_version": 1,
        "revision": revision,
        "root_name": root_name,
        "archive_sha256": "0" * 64,
    }

    assert _snapshot_root(cache, index) is None


def test_snapshot_root_rejects_symlinked_snapshots_directory(
    tmp_path: Path,
) -> None:
    cache = tmp_path / "cache"
    cache.mkdir()
    revision = f"git-{'a' * 40}"
    root_name = "elesim-main"
    outside_snapshots = tmp_path / "outside-snapshots"
    _write_valid_snapshot(outside_snapshots / revision, root_name)
    (cache / "snapshots").symlink_to(
        outside_snapshots,
        target_is_directory=True,
    )
    index = {
        "schema_version": 1,
        "revision": revision,
        "root_name": root_name,
        "archive_sha256": "0" * 64,
    }

    assert _snapshot_root(cache, index) is None


def test_download_source_rejects_symlinked_snapshots_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    url = "https://archives.example/elesim.tar.gz"
    cache = (
        tmp_path
        / "sources-v2"
        / hashlib.sha256(url.encode()).hexdigest()
    )
    cache.mkdir(parents=True)
    outside_snapshots = tmp_path / "outside-snapshots"
    outside_snapshots.mkdir()
    (cache / "snapshots").symlink_to(
        outside_snapshots,
        target_is_directory=True,
    )
    monkeypatch.setattr(
        urllib.request,
        "urlopen",
        _URLSequence(_Response(_archive_payload(tmp_path))),
    )

    with pytest.raises(BootstrapError, match="must not be a symlink"):
        download_source(url, tmp_path)

    assert not tuple(outside_snapshots.iterdir())


def test_download_source_custom_archive_uses_digest_not_forged_pax_commit(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = _archive_payload(tmp_path, commit="f" * 40)
    opener = _URLSequence(_Response(payload))
    monkeypatch.setattr(urllib.request, "urlopen", opener)

    root = download_source("https://archives.example/custom.tgz", tmp_path)

    assert root.parent.name == f"sha256-{hashlib.sha256(payload).hexdigest()}"


def test_download_source_serializes_same_url_workers(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payloads = (
        _archive_payload(tmp_path, root="elesim-one"),
        _archive_payload(tmp_path, root="elesim-two"),
    )
    first_entered = threading.Event()
    release_first = threading.Event()
    second_entered = threading.Event()
    state_lock = threading.Lock()
    call_count = 0

    def fake_download(
        _url: str,
        archive: Path,
        *,
        validators: dict[str, str],
    ) -> tuple[int, None, None, str]:
        nonlocal call_count
        assert validators == {}
        with state_lock:
            call_count += 1
            current = call_count
        if current == 1:
            first_entered.set()
            assert release_first.wait(timeout=5)
        else:
            second_entered.set()
        payload = payloads[current - 1]
        archive.write_bytes(payload)
        return 200, None, None, hashlib.sha256(payload).hexdigest()

    monkeypatch.setattr(bootstrap_module, "_download_archive", fake_download)
    url = "https://archives.example/elesim.tar.gz"
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        first_future = executor.submit(download_source, url, tmp_path)
        assert first_entered.wait(timeout=5)
        second_future = executor.submit(download_source, url, tmp_path)
        assert not second_entered.wait(timeout=0.1)
        release_first.set()
        first = first_future.result(timeout=5)
        second = second_future.result(timeout=5)

    assert second_entered.is_set()
    assert first.name == "elesim-one"
    assert second.name == "elesim-two"
    assert first != second
    url_cache = (
        tmp_path
        / "sources-v2"
        / hashlib.sha256(url.encode()).hexdigest()
    )
    current = json.loads((url_cache / "current.json").read_text(encoding="utf-8"))
    assert current["root_name"] == "elesim-two"


def test_index_write_failure_preserves_previous_current_snapshot(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    opener = _URLSequence(
        _Response(
            _archive_payload(tmp_path, root="elesim-one"),
            headers={"ETag": '"one"'},
        ),
        _Response(
            _archive_payload(tmp_path, root="elesim-two"),
            headers={"ETag": '"two"'},
        ),
    )
    monkeypatch.setattr(urllib.request, "urlopen", opener)
    url = "https://archives.example/elesim.tar.gz"
    first = download_source(url, tmp_path)
    index_path = (
        tmp_path
        / "sources-v2"
        / hashlib.sha256(url.encode()).hexdigest()
        / "current.json"
    )
    previous_index = index_path.read_bytes()

    def fail_index_write(_path: Path, _value: dict[str, object]) -> None:
        raise OSError("injected index failure")

    monkeypatch.setattr(bootstrap_module, "_atomic_write_json", fail_index_write)
    with pytest.raises(OSError, match="injected index failure"):
        download_source(url, tmp_path)

    assert index_path.read_bytes() == previous_index
    assert first.is_dir()


def test_atomic_index_replace_failure_preserves_previous_file(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    index = tmp_path / "current.json"
    index.write_text('{"old": true}\n', encoding="utf-8")
    original_replace = os.replace

    def fail_target_replace(source: str | os.PathLike[str], destination: str | os.PathLike[str]) -> None:
        if Path(destination) == index:
            raise OSError("injected replace failure")
        original_replace(source, destination)

    monkeypatch.setattr(os, "replace", fail_target_replace)

    with pytest.raises(OSError, match="injected replace failure"):
        _atomic_write_json(index, {"new": True})

    assert index.read_text(encoding="utf-8") == '{"old": true}\n'
    assert not tuple(tmp_path.glob(".current.json.*"))


def test_bootstrap_contract_and_executing_file_must_match(tmp_path: Path) -> None:
    source = tmp_path / "source"
    setup = source / "installer/bootstrap"
    setup.mkdir(parents=True)
    contract = {
        "schema_version": 1,
        "bootstrap_api": 1,
        "required_commands": ["wizard", "gui", "install", "update", "status"],
    }
    (setup / "bootstrap-contract.json").write_text(json.dumps(contract), encoding="utf-8")
    executing = tmp_path / "bootstrap.py"
    executing.write_text("same generation\n", encoding="utf-8")
    (setup / "bootstrap.py").write_text("same generation\n", encoding="utf-8")

    assert validate_bootstrap_contract(source) == contract
    validate_bootstrap_generation(source, executing_file=executing)

    (setup / "bootstrap.py").write_text("moved branch\n", encoding="utf-8")
    with pytest.raises(BootstrapError, match="branch moved"):
        validate_bootstrap_generation(source, executing_file=executing)


def test_bootstrap_generation_auto_check_requires_shell_marker(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    setup = tmp_path / "installer/bootstrap"
    setup.mkdir(parents=True)
    (setup / "bootstrap.py").write_text("different generation\n", encoding="utf-8")

    monkeypatch.delenv("ELESIM_VERIFY_BOOTSTRAP_SOURCE", raising=False)
    validate_bootstrap_generation(tmp_path)
    monkeypatch.setenv("ELESIM_VERIFY_BOOTSTRAP_SOURCE", "1")
    with pytest.raises(BootstrapError, match="branch moved"):
        validate_bootstrap_generation(tmp_path)


def test_bootstrap_repairs_cached_venv_without_pip(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[str, ...]] = []

    def fake_run(command: tuple[str, ...], **_kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(command)
        if "pip" in command and "--version" in command:
            # The first probe sees the incomplete venv; the final probe sees
            # the pip installed by ensurepip.
            return subprocess.CompletedProcess(command, 0 if len(calls) == 3 else 1)
        if "ensurepip" in command:
            return subprocess.CompletedProcess(command, 0)
        raise AssertionError(command)

    monkeypatch.setattr(bootstrap_module.subprocess, "run", fake_run)
    _ensure_bootstrap_pip(tmp_path / "bin/python")

    assert calls == [
        (str(tmp_path / "bin/python"), "-m", "pip", "--version"),
        (str(tmp_path / "bin/python"), "-m", "ensurepip", "--upgrade"),
        (str(tmp_path / "bin/python"), "-m", "pip", "--version"),
    ]


def test_bootstrap_reports_missing_venv_package_when_ensurepip_is_unavailable(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def fake_run(command: tuple[str, ...], **_kwargs: object) -> subprocess.CompletedProcess[str]:
        if "ensurepip" in command:
            return subprocess.CompletedProcess(
                command,
                1,
                stderr="No module named ensurepip",
            )
        return subprocess.CompletedProcess(command, 1, stderr="No module named pip")

    monkeypatch.setattr(bootstrap_module.subprocess, "run", fake_run)
    with pytest.raises(BootstrapError, match="python3-venv|ensurepip"):
        _ensure_bootstrap_pip(tmp_path / "bin/python")

@pytest.mark.parametrize(
    "contract",
    [
        {"schema_version": 2, "bootstrap_api": 1, "required_commands": []},
        {
            "schema_version": 1,
            "bootstrap_api": 2,
            "required_commands": ["wizard", "gui", "install", "update", "status"],
        },
        {
            "schema_version": 1,
            "bootstrap_api": 1,
            "required_commands": ["wizard", "install", "status"],
        },
    ],
)
def test_bootstrap_contract_rejects_incompatible_generation(
    tmp_path: Path,
    contract: dict[str, object],
) -> None:
    setup = tmp_path / "installer/bootstrap"
    setup.mkdir(parents=True)
    (setup / "bootstrap-contract.json").write_text(json.dumps(contract), encoding="utf-8")

    with pytest.raises(BootstrapError, match="contract|generation"):
        validate_bootstrap_contract(tmp_path)


def test_bootstrap_contract_is_required(tmp_path: Path) -> None:
    with pytest.raises(BootstrapError, match="missing bootstrap-contract"):
        validate_bootstrap_contract(tmp_path)


def test_gui_and_automation_do_not_require_a_controlling_terminal() -> None:
    assert needs_controlling_terminal(("gui",)) is False
    assert needs_controlling_terminal(("install", "--profile", "compute")) is False
    assert needs_controlling_terminal(("status",)) is False
    assert needs_controlling_terminal(("wizard",)) is True


def test_gui_setup_arguments_preserve_trusted_repository_and_ref() -> None:
    assert setup_arguments((), repository="owner/fork", ref="feature") == [
        "gui",
        "--repository",
        "owner/fork",
        "--ref",
        "feature",
    ]
    assert setup_arguments(
        ("gui", "--host", "0.0.0.0"),
        repository="owner/fork",
        ref="feature",
    ) == [
        "gui",
        "--host",
        "0.0.0.0",
        "--repository",
        "owner/fork",
        "--ref",
        "feature",
    ]
    assert setup_arguments(
        ("--state", "/tmp/state.json", "gui", "--host", "0.0.0.0"),
        repository="owner/fork",
        ref="feature",
    ) == [
        "--state",
        "/tmp/state.json",
        "gui",
        "--host",
        "0.0.0.0",
        "--repository",
        "owner/fork",
        "--ref",
        "feature",
    ]
    assert setup_arguments(
        ("install", "--dry-run"),
        repository="owner/fork",
        ref="feature",
    ) == ["install", "--dry-run"]


def test_main_forwards_trusted_gui_source_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source_root = tmp_path / "source"
    source_root.mkdir()
    executable = tmp_path / "elesim-setup"
    download_call: dict[str, object] = {}
    commands: list[tuple[str, ...]] = []

    def fake_download(
        url: str,
        cache_root: Path,
        *,
        refresh: bool,
        ref: str | None,
    ) -> Path:
        download_call.update(
            url=url,
            cache_root=cache_root,
            refresh=refresh,
            ref=ref,
        )
        return source_root

    def fake_run(
        command: tuple[str, ...],
        **_kwargs: object,
    ) -> subprocess.CompletedProcess[str]:
        commands.append(tuple(command))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(bootstrap_module, "download_source", fake_download)
    monkeypatch.setattr(bootstrap_module, "validate_bootstrap_contract", lambda _root: {})
    monkeypatch.setattr(bootstrap_module, "validate_bootstrap_generation", lambda _root: None)
    monkeypatch.setattr(
        bootstrap_module,
        "prepare_bootstrap_venv",
        lambda _root, _cache: executable,
    )
    monkeypatch.setattr(bootstrap_module, "preflight_setup", lambda _executable: None)
    monkeypatch.setattr(bootstrap_module.subprocess, "run", fake_run)

    result = bootstrap_module.main(
        (
            "--repository",
            "owner/fork",
            "--ref",
            "feature",
            "--cache-dir",
            str(tmp_path / "cache"),
            "--state",
            str(tmp_path / "state.json"),
            "gui",
            "--host",
            "127.0.0.1",
        )
    )

    assert result == 0
    assert download_call["url"] == (
        "https://codeload.github.com/owner/fork/tar.gz/feature"
    )
    assert download_call["ref"] == "feature"
    assert commands == [
        (
            str(executable),
            "--source-root",
            str(source_root),
            "--state",
            str(tmp_path / "state.json"),
            "gui",
            "--host",
            "127.0.0.1",
            "--repository",
            "owner/fork",
            "--ref",
            "feature",
        )
    ]


def test_failed_shell_download_preserves_previous_bootstrap(tmp_path: Path) -> None:
    script = Path(__file__).resolve().parents[3] / "installer/bootstrap/install.sh"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    docker.chmod(0o755)
    curl = fake_bin / "curl"
    curl.write_text(
        "#!/bin/sh\n"
        "while [ \"$#\" -gt 0 ]; do\n"
        "  if [ \"$1\" = \"-o\" ]; then\n"
        "    shift\n"
        "    printf partial >\"$1\"\n"
        "  fi\n"
        "  shift\n"
        "done\n"
        "exit 22\n",
        encoding="utf-8",
    )
    curl.chmod(0o755)
    home = tmp_path / "home"
    cache = home / ".cache/elesim/setup"
    cache.mkdir(parents=True)
    bootstrap = cache / "bootstrap.py"
    bootstrap.write_text("known good\n", encoding="utf-8")
    environment = os.environ.copy()
    environment.update(
        HOME=str(home),
        PATH=os.pathsep.join((str(fake_bin), environment["PATH"])),
        ELESIM_CACHE_DIR=str(cache),
        ELESIM_NO_OPEN="1",
    )

    completed = subprocess.run(
        ("bash", str(script)),
        cwd=tmp_path,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )

    assert completed.returncode != 0
    assert bootstrap.read_text(encoding="utf-8") == "known good\n"
    assert not tuple(cache.glob(".bootstrap.py.*"))


def test_bootstrap_reports_selected_docker_backend_and_tailscale_interfaces() -> None:
    script = (Path(__file__).resolve().parents[3] / "installer/bootstrap/install.sh").read_text(
        encoding="utf-8"
    )

    # The bootstrap keeps the selected Docker command in an array so a
    # temporary sudo fallback is handled without reparsing shell text.
    assert "info --format '{{.Name}}'" in script
    assert "info --format '{{.ID}}'" in script
    assert "context show" in script
    assert "context inspect" in script
    assert "DOCKER_HOST overrides are not supported" in script
    assert "ssh://*|tcp://*" in script
    assert "docker_backend_kind=\"docker-desktop\"" in script
    assert "^tailscale[0-9]+$" in script
    assert "ELESIM_HOST_TAILSCALE_INTERFACES" in script
    assert "kernel-mode Tailscale runtime sidecar" in script
    assert "ELESIM_HOST_DOCKER_BACKEND" in script
    assert "ELESIM_HOST_DOCKER_CONTEXT" in script
    assert "ELESIM_HOST_DOCKER_ENGINE_ID" in script
    assert "ELESIM_HOST_DOCKER_ENDPOINT" in script
    assert "ELESIM_HOST_DOCKER_HOST_OVERRIDE" in script


def test_shell_bootstrap_rejects_docker_host_override(tmp_path: Path) -> None:
    script = Path(__file__).resolve().parents[3] / "installer/bootstrap/install.sh"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    docker.chmod(0o755)
    home = tmp_path / "home"
    home.mkdir()
    environment = os.environ.copy()
    environment.update(
        HOME=str(home),
        PATH=os.pathsep.join((str(fake_bin), environment["PATH"])),
        DOCKER_HOST="tcp://docker.example:2376",
        ELESIM_CACHE_DIR=str(home / ".cache/elesim/setup"),
        ELESIM_NO_OPEN="1",
    )

    completed = subprocess.run(
        ("bash", str(script)),
        cwd=tmp_path,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )

    assert completed.returncode != 0
    assert "DOCKER_HOST overrides are not supported" in completed.stdout


def test_shell_bootstrap_rejects_remote_docker_context(tmp_path: Path) -> None:
    script = Path(__file__).resolve().parents[3] / "installer/bootstrap/install.sh"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/bin/sh\n"
        "case \"$*\" in\n"
        "  info|'compose version') exit 0 ;;\n"
        "  'info --format {{.Name}}') printf 'remote-engine\\n' ;;\n"
        "  'context show') printf 'research-server\\n' ;;\n"
        "  'info --format {{.ID}}') printf 'remote-id\\n' ;;\n"
        "  context\\ inspect*) printf 'ssh://docker.example/run/docker.sock\\n' ;;\n"
        "esac\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    home = tmp_path / "home"
    home.mkdir()
    environment = os.environ.copy()
    environment.pop("DOCKER_HOST", None)
    environment.update(
        HOME=str(home),
        PATH=os.pathsep.join((str(fake_bin), environment["PATH"])),
        ELESIM_CACHE_DIR=str(home / ".cache/elesim/setup"),
        ELESIM_NO_OPEN="1",
    )

    completed = subprocess.run(
        ("bash", str(script)),
        cwd=tmp_path,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )

    assert completed.returncode != 0
    assert "remote Docker contexts are not supported" in completed.stdout
    assert "ssh://docker.example/run/docker.sock" in completed.stdout


def test_shell_forwards_custom_archive_without_exposing_it_in_docker_argv(
    tmp_path: Path,
) -> None:
    script = Path(__file__).resolve().parents[3] / "installer/bootstrap/install.sh"
    bootstrap_source = Path(__file__).resolve().parents[3] / "installer/bootstrap/bootstrap.py"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    curl = fake_bin / "curl"
    curl.write_text(
        "#!/bin/sh\n"
        "output=\n"
        "while [ \"$#\" -gt 0 ]; do\n"
        "  if [ \"$1\" = \"-o\" ]; then\n"
        "    shift\n"
        "    output=$1\n"
        "  fi\n"
        "  shift\n"
        "done\n"
        "cp \"$FAKE_BOOTSTRAP_SOURCE\" \"$output\"\n",
        encoding="utf-8",
    )
    curl.chmod(0o755)
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/bin/sh\n"
        "if [ \"$1\" = context ] && [ \"$2\" = show ]; then\n"
        "  printf 'default\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [ \"$1\" = context ] && [ \"$2\" = inspect ]; then\n"
        "  printf 'unix:///var/run/docker.sock\\n'\n"
        "  exit 0\n"
        "fi\n"
        "case \"$1\" in\n"
        "  info|compose) exit 0 ;;\n"
        "esac\n"
        "printf '%s\\n' \"$@\" >\"$DOCKER_LOG.args\"\n"
        "while [ \"$#\" -gt 0 ]; do\n"
        "  if [ \"$1\" = \"--env-file\" ]; then\n"
        "    shift\n"
        "    cp \"$1\" \"$DOCKER_LOG.env\"\n"
        "  fi\n"
        "  shift\n"
        "done\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    home = tmp_path / "home"
    home.mkdir()
    docker_log = tmp_path / "docker"
    archive_url = "https://archives.example/elesim.tgz?token=secret"
    environment = os.environ.copy()
    environment.pop("DOCKER_HOST", None)
    environment.update(
        HOME=str(home),
        PATH=os.pathsep.join((str(fake_bin), environment["PATH"])),
        ELESIM_CACHE_DIR=str(home / ".cache/elesim/setup"),
        ELESIM_NO_OPEN="1",
        ELESIM_ARCHIVE_URL=archive_url,
        ELESIM_HOST_USER="not a linux username",
        USER="not a linux username",
        LOGNAME="not a linux username",
        FAKE_BOOTSTRAP_SOURCE=str(bootstrap_source),
        DOCKER_LOG=str(docker_log),
    )

    completed = subprocess.run(
        ("bash", str(script)),
        cwd=tmp_path,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout
    assert archive_url not in (tmp_path / "docker.args").read_text(encoding="utf-8")
    docker_args = (tmp_path / "docker.args").read_text(encoding="utf-8").splitlines()
    assert "USER=dev" in docker_args
    assert "LOGNAME=dev" in docker_args
    assert "USERNAME=dev" in docker_args
    assert "ELESIM_HOST_USER=dev" in docker_args
    assert (tmp_path / "docker.env").read_text(encoding="utf-8") == (
        f"ELESIM_ARCHIVE_URL={archive_url}\n"
    )


def test_container_bootstrap_preserves_host_python_and_uses_compose_v2() -> None:
    script = (Path(__file__).resolve().parents[3] / "installer/bootstrap/install.sh").read_text(
        encoding="utf-8"
    )
    assert "python:3.10-slim" in script
    assert '"${docker_cmd[@]}" compose version' in script
    assert "pip install" not in script
    assert 'docker_args+=(--publish "127.0.0.1:${gui_port}:${gui_port}")' in script
    assert '--workdir "$invocation_dir"' in script
    assert '"ELESIM_HOST_ARCH=$host_arch"' in script
    assert '"ELESIM_HOST_WSLG=$host_wslg"' in script
    assert 'host_user="${ELESIM_HOST_USER:-${USER:-${LOGNAME:-dev}}}"' in script
    assert 'if [[ ! "$host_user" =~ ^[a-z_][a-z0-9_-]{0,31}$ ]]; then' in script
    assert '--env "USER=$host_user"' in script
    assert '--env "LOGNAME=$host_user"' in script
    assert '--env "USERNAME=$host_user"' in script
    assert '"USER=$host_user"' in script
    assert '"LOGNAME=$host_user"' in script
    assert '"USERNAME=$host_user"' in script
    assert '"ELESIM_HOST_USER=$host_user"' in script
    assert '"ELESIM_VERIFY_BOOTSTRAP_SOURCE=1"' in script
    assert "port_is_in_use" in script
    assert "selected another available port" in script
    assert "gui_arguments=(gui)" in script
    assert "wizard|install|update|status)" in script
    assert 'if [[ "$argument" != "gui" ]]' in script
    assert 'bootstrap_tmp="$(mktemp "$cache_dir/.bootstrap.py.XXXXXX")"' in script
    assert 'curl -fsSL "$raw_url" -o "$bootstrap_tmp"' in script
    assert 'mv -f -- "$bootstrap_tmp" "$bootstrap_file"' in script
    assert 'docker_args+=(--env-file "$archive_env_file")' in script
    assert 'python /tmp/elesim-bootstrap.py "$@"' in script
    assert '"PYTHONNOUSERSITE=1"' in script


def test_bootstrap_venv_pins_ros_build_python_metadata_dependencies() -> None:
    script = Path(__file__).resolve().parents[3] / "installer/bootstrap/bootstrap.py"
    text = script.read_text(encoding="utf-8")

    assert '"setuptools>=68,<80"' in text
    assert '"packaging>=24.2,<26"' in text


def test_jetson_bootstrap_uses_host_ros_without_exposing_gui() -> None:
    script = (Path(__file__).resolve().parents[3] / "installer/bootstrap/install.sh").read_text(
        encoding="utf-8"
    )

    assert "host_bootstrap=0" in script
    assert "[[ -r /opt/ros/humble/setup.bash ]]" in script
    assert 'command -v colcon >/dev/null 2>&1' in script
    assert '"$host_python" -m venv --help' in script
    assert 'env "${host_bootstrap_env[@]}" "$host_python" "$bootstrap_file"' in script
    assert "--host 127.0.0.1" in script
    assert "EleSim 전용 host venv" in script
