from __future__ import annotations

from pathlib import Path

import pytest

from elesim_protocol import EndpointDescriptor, make_envelope
from elesim_router.config import RouterConfig, load_config
from elesim_router.core import RouterCore
from elesim_router.security import EndpointIdentityRegistry


PUBLIC_KEY = "A" * 40


def test_endpoint_identity_registry_binds_key_id_and_role(tmp_path: Path) -> None:
    registry_file = tmp_path / "endpoints.yaml"
    registry_file.write_text(
        "schema_version: 1\n"
        "clients:\n"
        f"  - public_key: {PUBLIC_KEY}\n"
        "    endpoint_id: ui-main\n"
        "    role: ui\n",
        encoding="utf-8",
    )
    registry = EndpointIdentityRegistry.from_file(registry_file)

    assert registry.authorize(PUBLIC_KEY, "ui-main", "ui") is True
    assert registry.authorize(PUBLIC_KEY, "controller-main", "controller") is False
    assert registry.authorize("B" * 40, "ui-main", "ui") is False


def test_one_ui_key_can_authorize_its_operator_and_simulator_session_endpoints(
    tmp_path: Path,
) -> None:
    registry_file = tmp_path / "endpoints.yaml"
    registry_file.write_text(
        "schema_version: 1\n"
        "clients:\n"
        f"  - public_key: {PUBLIC_KEY}\n"
        "    endpoint_id: ui-main\n"
        "    role: ui\n"
        f"  - public_key: {PUBLIC_KEY}\n"
        "    endpoint_id: ui-main-simulator\n"
        "    role: ui\n",
        encoding="utf-8",
    )

    registry = EndpointIdentityRegistry.from_file(registry_file)

    assert registry.authorize(PUBLIC_KEY, "ui-main", "ui") is True
    assert registry.authorize(PUBLIC_KEY, "ui-main-simulator", "ui") is True
    assert registry.authorize(PUBLIC_KEY, "ui-other", "ui") is False


def test_router_core_rejects_registration_that_does_not_match_zap_identity() -> None:
    registry = EndpointIdentityRegistry(
        {PUBLIC_KEY: ("ui-main", "ui")}
    )
    core = RouterCore(endpoint_authorizer=registry.authorize)
    descriptor = EndpointDescriptor("ui-main", "ui", instance_id="instance-a")
    rejected = core.handle(
        b"ui",
        make_envelope(
            "register",
            "ui-main",
            payload={"endpoint": descriptor.to_dict()},
            seq=1,
        ),
        now=1.0,
        authenticated_user_id="B" * 40,
    )
    accepted = core.handle(
        b"ui",
        make_envelope(
            "register",
            "ui-main",
            payload={"endpoint": descriptor.to_dict()},
            seq=1,
        ),
        now=1.0,
        authenticated_user_id=PUBLIC_KEY,
    )

    assert rejected[0].envelope.message_type == "error"
    assert accepted[0].envelope.message_type == "registered"


def test_router_config_resolves_security_and_turn_secret_paths(tmp_path: Path) -> None:
    config_path = tmp_path / "public.yaml"
    config_path.write_text(
        "schema_version: 2\n"
        "router:\n"
        "  bind_endpoint: tcp://0.0.0.0:5558\n"
        "  heartbeat_timeout_s: 4.0\n"
        "security:\n"
        "  curve_server_secret_file: secrets/router.key_secret\n"
        "  curve_public_keys_dir: secrets/authorized\n"
        "  endpoint_registry_file: secrets/endpoints.yaml\n"
        "  allow_insecure_remote: false\n"
        "turn:\n"
        "  urls: ['turn:relay.example:3478?transport=udp']\n"
        "  static_auth_secret_file: secrets/turn.secret\n"
        "  credential_ttl_s: 3600\n"
        "  refresh_before_s: 600\n",
        encoding="utf-8",
    )
    config = load_config(config_path)

    assert isinstance(config, RouterConfig)
    assert config.bind_endpoint == "tcp://0.0.0.0:5558"
    assert config.curve_server_secret_file == tmp_path / "secrets/router.key_secret"
    assert config.turn_static_auth_secret_file == tmp_path / "secrets/turn.secret"


def test_public_bind_requires_curve_or_explicit_development_override() -> None:
    with pytest.raises(ValueError, match="CURVE"):
        RouterConfig(bind_endpoint="tcp://0.0.0.0:5558").validate()
