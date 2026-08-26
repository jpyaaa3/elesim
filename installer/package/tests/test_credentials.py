from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from elesim_setup.credentials import (
    install_staged_credentials,
    probe_ssh_fingerprint,
    proxy_failure_detail,
    validate_external_turn_credentials,
)


def test_staged_sros2_files_are_installed_with_private_modes(tmp_path: Path) -> None:
    staged = tmp_path / "staged"
    destination = tmp_path / "destination"
    private = staged / "keystore/enclaves/elesim/key.pem"
    public = staged / "keystore/enclaves/elesim/cert.pem"
    private.parent.mkdir(parents=True)
    private.write_text("private", encoding="utf-8")
    public.write_text("public", encoding="utf-8")

    installed = install_staged_credentials(staged, destination)

    assert len(installed) == 2
    assert (destination / private.relative_to(staged)).stat().st_mode & 0o777 == 0o600
    assert (destination / public.relative_to(staged)).stat().st_mode & 0o777 == 0o644


def test_staged_files_never_overwrite_different_material(tmp_path: Path) -> None:
    staged = tmp_path / "staged"
    destination = tmp_path / "destination"
    source = staged / "turn.secret"
    target = destination / "turn.secret"
    source.parent.mkdir()
    target.parent.mkdir()
    source.write_text("new", encoding="utf-8")
    target.write_text("old", encoding="utf-8")

    with pytest.raises(FileExistsError, match="덮어"):
        install_staged_credentials(staged, destination)


def test_identical_existing_files_are_idempotent(tmp_path: Path) -> None:
    staged = tmp_path / "staged"
    destination = tmp_path / "destination"
    source = staged / "cert.pem"
    target = destination / "cert.pem"
    source.parent.mkdir()
    target.parent.mkdir()
    source.write_text("same", encoding="utf-8")
    target.write_text("same", encoding="utf-8")

    assert install_staged_credentials(staged, destination) == ()


def test_ssh_fingerprint_uses_the_supplied_non_default_port(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[tuple[str, int], float]] = []

    class Connection:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return None

    class Key:
        @staticmethod
        def asbytes() -> bytes:
            return b"server-key"

    class Transport:
        def __init__(self, _connection):
            pass

        def start_client(self, *, timeout: float) -> None:
            assert timeout == 3.0

        @staticmethod
        def get_remote_server_key() -> Key:
            return Key()

        def close(self) -> None:
            pass

    monkeypatch.setattr(
        "elesim_setup.credentials.socket.create_connection",
        lambda address, timeout: (
            calls.append((address, timeout)) or Connection()
        ),
    )
    monkeypatch.setitem(sys.modules, "paramiko", SimpleNamespace(Transport=Transport))

    fingerprint = probe_ssh_fingerprint("server.example", 2222, timeout_s=3.0)

    assert calls == [(("server.example", 2222), 3.0)]
    assert fingerprint.startswith("SHA256:")


def test_ssh_fingerprint_uses_host_tailscale_proxy_for_cgnat_address(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[str] = []

    class Proxy:
        def __init__(self, command: str) -> None:
            calls.append(command)

        def close(self) -> None:
            pass

    class Connection:
        def close(self) -> None:
            raise AssertionError("the direct socket path must not be used")

    class Key:
        @staticmethod
        def asbytes() -> bytes:
            return b"server-key"

    class Transport:
        def __init__(self, _connection):
            pass

        def start_client(self, *, timeout: float) -> None:
            assert timeout == 3.0

        @staticmethod
        def get_remote_server_key() -> Key:
            return Key()

        def close(self) -> None:
            pass

    monkeypatch.setenv("ELESIM_TAILSCALE_PROXY", "1")
    monkeypatch.setenv("ELESIM_TAILSCALE_PROXY_BIN", "/usr/local/bin/tailscale")
    monkeypatch.setenv(
        "ELESIM_TAILSCALE_PROXY_SOCKET", "/var/run/tailscale/tailscaled.sock"
    )
    monkeypatch.setitem(
        sys.modules,
        "paramiko",
        SimpleNamespace(
            Transport=Transport,
            proxy=SimpleNamespace(ProxyCommand=Proxy),
        ),
    )
    monkeypatch.setitem(sys.modules, "paramiko.proxy", SimpleNamespace(ProxyCommand=Proxy))
    monkeypatch.setattr(
        "elesim_setup.credentials.socket.create_connection",
        lambda *_args, **_kwargs: Connection(),
    )

    fingerprint = probe_ssh_fingerprint("100.74.222.24", 22, timeout_s=3.0)

    assert fingerprint.startswith("SHA256:")
    assert calls == [
        "/usr/local/bin/tailscale --socket=/var/run/tailscale/tailscaled.sock nc 100.74.222.24 22"
    ]


def test_ssh_fingerprint_timeout_explains_container_and_tailscale_path(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("ELESIM_CONNECTION_PUBLISHED", "1")
    monkeypatch.setattr(
        "elesim_setup.credentials.socket.create_connection",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(TimeoutError()),
    )
    monkeypatch.setitem(sys.modules, "paramiko", SimpleNamespace(Transport=object))

    with pytest.raises(RuntimeError, match="Docker 컨테이너") as error:
        probe_ssh_fingerprint("100.74.222.24", 22)
    assert "tailscale" in str(error.value).lower()


def test_proxy_failure_detail_prefers_host_helper_diagnostic() -> None:
    process = SimpleNamespace(
        poll=lambda: 2,
        stderr=SimpleNamespace(
            read=lambda _limit: b"elesim-host-proxy: peer offline; SSH disabled\n"
        ),
    )

    assert proxy_failure_detail(
        SimpleNamespace(process=process), BrokenPipeError("Broken pipe")
    ) == "elesim-host-proxy: peer offline; SSH disabled"


def test_external_turn_credentials_use_strict_bounded_json(tmp_path: Path) -> None:
    credentials = tmp_path / "turn.credentials.json"
    credentials.write_text(
        '{"username":"lab-user","credential":"lab-password"}\n',
        encoding="utf-8",
    )

    validate_external_turn_credentials(
        credentials,
        urls=("turn:relay.example.com:3478?transport=udp",),
    )

    credentials.write_text(
        '{"username":"lab-user","credential":"lab-password","secret":"bad"}\n',
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unknown fields"):
        validate_external_turn_credentials(
            credentials,
            urls=("turn:relay.example.com:3478?transport=udp",),
        )


def test_external_turn_credentials_fail_when_expired(tmp_path: Path) -> None:
    credentials = tmp_path / "turn.credentials.json"
    credentials.write_text(
        '{"username":"lab-user","credential":"lab-password","expires_at":1}\n',
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="expired"):
        validate_external_turn_credentials(
            credentials,
            urls=("turn:relay.example.com:3478?transport=udp",),
        )


def test_external_turn_credentials_reject_symlinked_path(tmp_path: Path) -> None:
    target = tmp_path / "credentials.json"
    target.write_text(
        '{"username":"lab-user","credential":"lab-password"}\n',
        encoding="utf-8",
    )
    link = tmp_path / "link.json"
    link.symlink_to(target)

    with pytest.raises(ValueError, match="symlinked path components"):
        validate_external_turn_credentials(
            link,
            urls=("turn:relay.example.com:3478?transport=udp",),
        )
