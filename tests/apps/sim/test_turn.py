from __future__ import annotations

import base64
import hashlib
import hmac

from elesim_sim.config.distributed import TurnConfig
from elesim_sim.turn import (
    StaticTurnCredentialProvider,
    TurnCredentialIssuer,
    load_turn_credential_provider,
)


def test_turn_rest_credentials_are_expiring_hmac_values() -> None:
    issuer = TurnCredentialIssuer(
        urls=("turn:relay.example:3478",),
        realm="relay.example",
        static_auth_secret=b"shared-secret",
    )

    credentials = issuer("ui-a", "4:opaque-token", 1000.25)

    assert credentials.expires_at == 4600.0
    assert credentials.username == "4600:ui-a:4:opaque-token"
    expected = hmac.new(
        b"shared-secret",
        credentials.username.encode("utf-8"),
        hashlib.sha1,
    ).digest()
    assert base64.b64decode(credentials.credential) == expected


def test_sim_loads_only_its_configured_turn_secret(tmp_path) -> None:
    secret = tmp_path / "turn.secret"
    secret.write_text("managed-secret\n", encoding="utf-8")

    issuer = load_turn_credential_provider(
        TurnConfig(
            urls=("turn:relay.example:3478",),
            realm="relay.example",
            static_auth_secret_file=secret,
        )
    )

    assert issuer is not None
    assert issuer.static_auth_secret == b"managed-secret"


def test_external_turn_without_local_hmac_secret_has_no_issuer() -> None:
    config = TurnConfig(urls=("turn:external.example:3478",))

    try:
        load_turn_credential_provider(config)
    except ValueError as exc:
        assert "credential source" in str(exc)
    else:
        raise AssertionError("TURN URLs without credentials must fail closed")


def test_external_turn_credentials_are_loaded_from_bounded_json(tmp_path) -> None:
    path = tmp_path / "turn.credentials.json"
    path.write_text(
        '{"username":"lab-user","credential":"lab-password",'
        '"expires_at":4102444800}\n',
        encoding="utf-8",
    )

    provider = load_turn_credential_provider(
        TurnConfig(
            urls=("turn:external.example:3478?transport=udp",),
            credential_file=path,
        )
    )

    assert isinstance(provider, StaticTurnCredentialProvider)
    credentials = provider("ui-main", "session-1", 1000.0)
    assert credentials.urls == ("turn:external.example:3478?transport=udp",)
    assert credentials.username == "lab-user"
    assert credentials.credential == "lab-password"
    assert credentials.expires_at == 4102444800.0


def test_external_turn_credentials_reject_unknown_json_fields(tmp_path) -> None:
    path = tmp_path / "turn.credentials.json"
    path.write_text(
        '{"username":"lab-user","credential":"lab-password","secret":"no"}\n',
        encoding="utf-8",
    )

    try:
        load_turn_credential_provider(
            TurnConfig(
                urls=("turn:external.example:3478",),
                credential_file=path,
            )
        )
    except ValueError as exc:
        assert "unknown fields" in str(exc)
    else:
        raise AssertionError("unknown credential fields must be rejected")
