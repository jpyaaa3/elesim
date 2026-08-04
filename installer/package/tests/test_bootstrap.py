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
from pathlib import Path

import pytest

from installer.bootstrap import bootstrap as bootstrap_module
from installer.bootstrap.bootstrap import (
    BootstrapError,
    _atomic_write_json,
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
) -> bytes:
    archive = tmp_path / f"{root}.tgz"
    _archive(
        archive,
        {
            f"{root}/installer/package/pyproject.toml": project,
            f"{root}/installer/package/requirements.lock": b"",
            f"{root}/packages/protocol/pyproject.toml": b"[project]\n",
            f"{root}/installer/bootstrap/bootstrap.py": b"# bootstrap\n",
            f"{root}/installer/bootstrap/bootstrap-contract.json": json.dumps(
                {
                    "schema_version": 1,
                    "bootstrap_api": 1,
                    "required_commands": ["wizard", "gui", "install", "status"],
                }
            ).encode("utf-8"),
        },
        commit=commit,
    )
    return archive.read_bytes()


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
    (root / "installer/package").mkdir(parents=True)
    (root / "installer/package/pyproject.toml").write_text("[project]\n")
    (root / "installer/package/requirements.lock").write_text("")
    (root / "packages/protocol").mkdir(parents=True)
    (root / "packages/protocol/pyproject.toml").write_text("[project]\n")
    (root / "installer/bootstrap").mkdir(parents=True)
    (root / "installer/bootstrap/bootstrap.py").write_text("# bootstrap\n")
    (root / "installer/bootstrap/bootstrap-contract.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "bootstrap_api": 1,
                "required_commands": ["wizard", "gui", "install", "status"],
            }
        ),
        encoding="utf-8",
    )
    (snapshot / ".elesim-source-complete").write_text(root_name + "\n")


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
        "required_commands": ["wizard", "gui", "install", "status"],
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


@pytest.mark.parametrize(
    "contract",
    [
        {"schema_version": 2, "bootstrap_api": 1, "required_commands": []},
        {
            "schema_version": 1,
            "bootstrap_api": 2,
            "required_commands": ["wizard", "gui", "install", "status"],
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
    script = Path(__file__).resolve().parents[3] / "installer/bootstrap/bootstrap.sh"
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


def test_shell_forwards_custom_archive_without_exposing_it_in_docker_argv(
    tmp_path: Path,
) -> None:
    script = Path(__file__).resolve().parents[3] / "installer/bootstrap/bootstrap.sh"
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
    environment.update(
        HOME=str(home),
        PATH=os.pathsep.join((str(fake_bin), environment["PATH"])),
        ELESIM_CACHE_DIR=str(home / ".cache/elesim/setup"),
        ELESIM_NO_OPEN="1",
        ELESIM_ARCHIVE_URL=archive_url,
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
    assert (tmp_path / "docker.env").read_text(encoding="utf-8") == (
        f"ELESIM_ARCHIVE_URL={archive_url}\n"
    )


def test_container_bootstrap_preserves_host_python_and_uses_compose_v2() -> None:
    script = (Path(__file__).resolve().parents[3] / "installer/bootstrap/bootstrap.sh").read_text(
        encoding="utf-8"
    )
    assert "python:3.10-slim" in script
    assert '"${docker_cmd[@]}" compose version' in script
    assert "pip install" not in script
    assert 'docker_args+=(--publish "127.0.0.1:${gui_port}:${gui_port}")' in script
    assert '--workdir "$invocation_dir"' in script
    assert '"ELESIM_HOST_ARCH=$host_arch"' in script
    assert '"ELESIM_HOST_WSLG=$host_wslg"' in script
    assert '"ELESIM_VERIFY_BOOTSTRAP_SOURCE=1"' in script
    assert "port_is_in_use" in script
    assert "selected another available port" in script
    assert "gui_arguments=(gui)" in script
    assert "wizard|install|status)" in script
    assert 'if [[ "$argument" != "gui" ]]' in script
    assert 'bootstrap_tmp="$(mktemp "$cache_dir/.bootstrap.py.XXXXXX")"' in script
    assert 'curl -fsSL "$raw_url" -o "$bootstrap_tmp"' in script
    assert 'mv -f -- "$bootstrap_tmp" "$bootstrap_file"' in script
    assert 'docker_args+=(--env-file "$archive_env_file")' in script
    assert 'python /tmp/elesim-bootstrap.py "$@"' in script
