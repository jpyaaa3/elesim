from __future__ import annotations

from pathlib import Path

import pytest
import zmq
from zmq.auth import create_certificates

from elesim_protocol import (
    CurveClientConfig,
    CurveServerConfig,
    TransportSecurityError,
    configure_curve_client,
    configure_curve_server,
    endpoint_is_loopback,
    require_curve_server_auth,
    require_secure_remote,
)


def test_loopback_detection_distinguishes_public_tcp_addresses() -> None:
    assert endpoint_is_loopback("tcp://127.0.0.1:5558") is True
    assert endpoint_is_loopback("tcp://localhost:5558") is True
    assert endpoint_is_loopback("tcp://[::1]:5558") is True
    assert endpoint_is_loopback("ipc:///tmp/elesim.sock") is True
    assert endpoint_is_loopback("tcp://192.0.2.10:5558") is False
    assert endpoint_is_loopback("tcp://0.0.0.0:5558") is False


def test_plaintext_remote_transport_requires_explicit_development_override() -> None:
    with pytest.raises(TransportSecurityError, match="CURVE"):
        require_secure_remote("tcp://192.0.2.10:5558", curve_enabled=False)
    require_secure_remote(
        "tcp://192.0.2.10:5558",
        curve_enabled=False,
        allow_insecure_remote=True,
    )


def test_remote_curve_server_requires_a_client_key_allowlist() -> None:
    with pytest.raises(TransportSecurityError, match="authorized client key"):
        require_curve_server_auth(
            "tcp://0.0.0.0:5568",
            curve_enabled=True,
            authorized_clients=False,
        )

    require_curve_server_auth(
        "tcp://0.0.0.0:5568",
        curve_enabled=True,
        authorized_clients=True,
    )


def test_curve_certificates_load_and_configure_sockets(tmp_path: Path) -> None:
    create_certificates(str(tmp_path), "server")
    create_certificates(str(tmp_path), "client")
    client = CurveClientConfig.from_files(
        client_secret_file=tmp_path / "client.key_secret",
        server_public_file=tmp_path / "server.key",
    )
    server = CurveServerConfig.from_file(tmp_path / "server.key_secret")
    context = zmq.Context()
    dealer = context.socket(zmq.DEALER)
    router = context.socket(zmq.ROUTER)
    try:
        configure_curve_client(dealer, client)
        configure_curve_server(router, server)
        assert client.server_key == server.public_key
        assert bool(router.curve_server) is True
    finally:
        dealer.close(0)
        router.close(0)
        context.term()


def test_curve_client_uses_a_media_server_key_from_stream_metadata(tmp_path: Path) -> None:
    create_certificates(str(tmp_path), "media-client")
    server_public, _server_secret = zmq.curve_keypair()

    client = CurveClientConfig.from_client_file(
        client_secret_file=tmp_path / "media-client.key_secret",
        server_key=server_public,
    )

    assert client.server_key == server_public
