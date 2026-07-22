#!/usr/bin/env python3
"""ZMQ process boundary for the Elesim protocol router."""

from __future__ import annotations

import argparse
import logging
import threading
from collections.abc import Sequence
from dataclasses import replace
from pathlib import Path
from typing import Optional

import zmq
from zmq.auth.thread import ThreadAuthenticator

from elesim_protocol import (
    CurveServerConfig,
    Envelope,
    ProtocolError,
    dumps_envelope,
    loads_envelope,
    make_envelope,
    configure_curve_server,
    require_secure_remote,
)
from elesim_router.config import load_config
from elesim_router.core import RoutedMessage, RouterCore
from elesim_router.security import EndpointIdentityRegistry
from elesim_router.simulation_sessions import TurnCredentialIssuer


LOG = logging.getLogger("elesim.router")
_ROOT = Path(__file__).resolve().parents[2]


def decode_router_frames(frames: Sequence[bytes]) -> tuple[bytes, Envelope]:
    if len(frames) != 2:
        raise ProtocolError(f"ROUTER message must contain exactly two frames, got {len(frames)}")
    identity, payload = frames
    if not isinstance(identity, bytes) or not identity:
        raise ProtocolError("ROUTER identity frame must be non-empty bytes")
    if not isinstance(payload, bytes):
        raise ProtocolError("ROUTER payload frame must be bytes")
    return identity, loads_envelope(payload)


class RoutingServer:
    def __init__(
        self,
        bind_endpoint: str,
        *,
        heartbeat_timeout_s: float = 3.5,
        curve: Optional[CurveServerConfig] = None,
        curve_public_keys_dir: str | Path | None = None,
        endpoint_registry: Optional[EndpointIdentityRegistry] = None,
        turn_issuer: Optional[TurnCredentialIssuer] = None,
        allow_insecure_remote: bool = False,
    ) -> None:
        self.bind_endpoint = str(bind_endpoint)
        require_secure_remote(
            self.bind_endpoint,
            curve_enabled=curve is not None,
            allow_insecure_remote=bool(allow_insecure_remote),
        )
        if curve is not None and (curve_public_keys_dir is None or endpoint_registry is None):
            raise ValueError("CURVE router requires authorized keys and endpoint registry")
        self.core = RouterCore(
            heartbeat_timeout_s=heartbeat_timeout_s,
            turn_issuer=turn_issuer,
            endpoint_authorizer=None if endpoint_registry is None else endpoint_registry.authorize,
        )
        self.context = zmq.Context.instance()
        self.authenticator: Optional[ThreadAuthenticator] = None
        if curve is not None:
            self.authenticator = ThreadAuthenticator(self.context)
            self.authenticator.start()
            self.authenticator.configure_curve(
                domain="*",
                location=str(Path(curve_public_keys_dir).expanduser().resolve()),
            )
        self.socket = self.context.socket(zmq.ROUTER)
        self.socket.setsockopt(zmq.LINGER, 0)
        if curve is not None:
            configure_curve_server(self.socket, curve)
        self.socket.bind(self.bind_endpoint)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.stop_event = threading.Event()

    def run(self) -> None:
        LOG.info("routing endpoint bound %s", self.bind_endpoint)
        try:
            while not self.stop_event.is_set():
                events = dict(self.poller.poll(100))
                if self.socket in events:
                    self._receive_and_route()
                self._send(self.core.expire())
        except KeyboardInterrupt:
            pass
        finally:
            self.socket.close(0)
            if self.authenticator is not None:
                self.authenticator.stop()

    def close(self) -> None:
        self.stop_event.set()

    def _receive_and_route(self) -> None:
        received = self.socket.recv_multipart(copy=False)
        authenticated_user_id = self._authenticated_user_id(received)
        frames = [bytes(frame) for frame in received]
        identity = frames[0] if frames else b""
        try:
            identity, request = decode_router_frames(frames)
            routed = self.core.handle(
                identity,
                request,
                authenticated_user_id=authenticated_user_id,
            )
        except ProtocolError as exc:
            routed = self._wire_error(identity, str(exc)) if identity else []
        except Exception as exc:  # keep one malformed request from killing the router
            LOG.exception("unexpected router request failure")
            routed = self._wire_error(identity, f"internal router error: {type(exc).__name__}") if identity else []
        self._send(routed)

    @staticmethod
    def _authenticated_user_id(frames: Sequence[object]) -> str:
        if not frames:
            return ""
        frame = frames[-1]
        try:
            value = frame.get("User-Id")  # type: ignore[attr-defined]
        except (KeyError, AttributeError, zmq.ZMQError):
            return ""
        if isinstance(value, bytes):
            return value.decode("ascii", errors="strict")
        return str(value)

    @staticmethod
    def _wire_error(identity: bytes, reason: str) -> list[RoutedMessage]:
        return [
            RoutedMessage(
                identity,
                make_envelope(
                    "error",
                    "server",
                    target_id="unknown",
                    payload={"ok": False, "reason": str(reason)},
                    seq=1,
                ),
            )
        ]

    def _send(self, routed: list[RoutedMessage]) -> None:
        for item in routed:
            self.socket.send_multipart([item.identity, dumps_envelope(item.envelope)])


def _run() -> None:
    parser = argparse.ArgumentParser(description="Elesim distributed routing server")
    parser.add_argument("--config", default=str(_ROOT / "config/default.yaml"))
    parser.add_argument("--bind", default="")
    parser.add_argument("--heartbeat-timeout", type=float, default=None)
    parser.add_argument("--allow-insecure-remote", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config)
    if args.bind:
        config = replace(config, bind_endpoint=str(args.bind))
    if args.heartbeat_timeout is not None:
        config = replace(config, heartbeat_timeout_s=float(args.heartbeat_timeout))
    if args.allow_insecure_remote:
        config = replace(config, allow_insecure_remote=True)
    config.validate()

    curve = (
        None
        if config.curve_server_secret_file is None
        else CurveServerConfig.from_file(config.curve_server_secret_file)
    )
    registry = (
        None
        if config.endpoint_registry_file is None
        else EndpointIdentityRegistry.from_file(config.endpoint_registry_file)
    )
    turn_issuer = None
    if config.turn_static_auth_secret_file is not None:
        secret = config.turn_static_auth_secret_file.read_bytes().strip()
        turn_issuer = TurnCredentialIssuer(
            urls=config.turn_urls,
            static_auth_secret=secret,
            ttl_s=config.turn_credential_ttl_s,
            refresh_before_s=config.turn_refresh_before_s,
        )
    RoutingServer(
        config.bind_endpoint,
        heartbeat_timeout_s=config.heartbeat_timeout_s,
        curve=curve,
        curve_public_keys_dir=config.curve_public_keys_dir,
        endpoint_registry=registry,
        turn_issuer=turn_issuer,
        allow_insecure_remote=config.allow_insecure_remote,
    ).run()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    _run()


if __name__ == "__main__":
    main()
