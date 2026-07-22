from __future__ import annotations

import shutil
import threading
import time
from pathlib import Path

import zmq
from zmq.auth import create_certificates, load_certificate

from elesim_protocol import (
    CurveClientConfig,
    CurveServerConfig,
    EndpointClient,
    EndpointDescriptor,
)
from elesim_router.main import RoutingServer
from elesim_router.security import EndpointIdentityRegistry


def _certificate(directory: Path, name: str) -> tuple[Path, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    create_certificates(str(directory), name)
    return directory / f"{name}.key", directory / f"{name}.key_secret"


def _wait_for_registration(client: EndpointClient, *, timeout_s: float = 3.0):
    deadline = time.monotonic() + timeout_s
    messages = []
    while time.monotonic() < deadline:
        client.heartbeat()
        messages.extend(client.receive(timeout_ms=50))
        if client.registered:
            return messages
    return messages


def test_curve_router_authenticates_wire_key_before_endpoint_registration(
    tmp_path: Path,
) -> None:
    server_public, server_secret = _certificate(tmp_path / "server", "router")
    client_public, client_secret = _certificate(tmp_path / "clients", "controller")
    authorized = tmp_path / "authorized"
    authorized.mkdir()
    shutil.copy2(client_public, authorized / client_public.name)
    public_key, _ = load_certificate(str(client_public))
    registry = EndpointIdentityRegistry(
        {public_key.decode("ascii"): ("controller-main", "controller")}
    )
    server = RoutingServer(
        "tcp://127.0.0.1:*",
        curve=CurveServerConfig.from_file(server_secret),
        curve_public_keys_dir=authorized,
        endpoint_registry=registry,
    )
    endpoint = server.socket.getsockopt(zmq.LAST_ENDPOINT).decode("ascii")
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    client = EndpointClient(
        endpoint,
        EndpointDescriptor("controller-main", "controller", ("operator_control",)),
        curve=CurveClientConfig.from_files(
            client_secret_file=client_secret,
            server_public_file=server_public,
        ),
        registration_retry_s=0.1,
    )

    try:
        messages = _wait_for_registration(client)
        assert client.registered
        assert any(message.message_type == "registered" for message in messages)
    finally:
        client.close()
        server.close()
        thread.join(timeout=2.0)


def test_curve_key_cannot_register_an_unlisted_endpoint_identity(tmp_path: Path) -> None:
    server_public, server_secret = _certificate(tmp_path / "server", "router")
    client_public, client_secret = _certificate(tmp_path / "clients", "controller")
    authorized = tmp_path / "authorized"
    authorized.mkdir()
    shutil.copy2(client_public, authorized / client_public.name)
    public_key, _ = load_certificate(str(client_public))
    registry = EndpointIdentityRegistry(
        {public_key.decode("ascii"): ("controller-main", "controller")}
    )
    server = RoutingServer(
        "tcp://127.0.0.1:*",
        curve=CurveServerConfig.from_file(server_secret),
        curve_public_keys_dir=authorized,
        endpoint_registry=registry,
    )
    endpoint = server.socket.getsockopt(zmq.LAST_ENDPOINT).decode("ascii")
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    client = EndpointClient(
        endpoint,
        EndpointDescriptor("controller-impostor", "controller", ("operator_control",)),
        curve=CurveClientConfig.from_files(
            client_secret_file=client_secret,
            server_public_file=server_public,
        ),
        registration_retry_s=0.1,
    )

    try:
        messages = _wait_for_registration(client, timeout_s=0.7)
        assert not client.registered
        assert any(
            message.message_type == "error"
            and "not authorized" in str(message.payload.get("reason", ""))
            for message in messages
        )
    finally:
        client.close()
        server.close()
        thread.join(timeout=2.0)
