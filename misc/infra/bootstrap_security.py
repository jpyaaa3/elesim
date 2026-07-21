#!/usr/bin/env python3
"""Generate local Elesim CURVE identities and a shared Coturn HMAC secret."""

from __future__ import annotations

import argparse
import secrets
import shutil
import stat
from pathlib import Path

import yaml
from zmq.auth import create_certificates, load_certificate


INFRA_ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT = INFRA_ROOT / "generated"
DEFAULT_COTURN_ENV = INFRA_ROOT / "coturn/.env"


def _secret(path: Path) -> None:
    path.chmod(stat.S_IRUSR | stat.S_IWUSR)


def _public_key(path: Path) -> str:
    public, _secret_key = load_certificate(str(path))
    return public.decode("ascii")


def _certificate(directory: Path, name: str) -> tuple[Path, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    create_certificates(str(directory), name)
    public = directory / f"{name}.key"
    private = directory / f"{name}.key_secret"
    _secret(private)
    return public, private


def generate(
    output: Path,
    coturn_env: Path,
    *,
    turn_public_ip: str,
    turn_realm: str,
    force: bool,
) -> None:
    turn_public_ip = str(turn_public_ip).strip()
    turn_realm = str(turn_realm).strip()
    if not turn_public_ip:
        raise ValueError("TURN public IP or hostname must not be empty")
    if not turn_realm:
        raise ValueError("TURN realm must not be empty")
    if output.exists():
        if not force:
            raise FileExistsError(f"{output} already exists; pass --force to replace it")
        shutil.rmtree(output)
    output.mkdir(parents=True)

    router_public, _router_private = _certificate(output / "curve/router", "router")
    authorized_dir = output / "curve/authorized"
    authorized_dir.mkdir(parents=True)
    media_authorized_dir = output / "curve/media-authorized"
    media_authorized_dir.mkdir(parents=True)

    clients = (
        ("controller-main", "controller"),
        ("ui-main", "ui"),
        ("doctor-main", "ui"),
        ("sim-default", "simulator"),
        ("robot-go2", "robot"),
    )
    registry: list[dict[str, str]] = []
    for endpoint_id, role in clients:
        public, _private = _certificate(output / "curve/clients", endpoint_id)
        shutil.copy2(public, authorized_dir / public.name)
        if endpoint_id == "controller-main":
            shutil.copy2(public, media_authorized_dir / public.name)
        key = _public_key(public)
        registry.append(
            {"public_key": key, "endpoint_id": endpoint_id, "role": role}
        )
        if endpoint_id == "ui-main":
            registry.append(
                {
                    "public_key": key,
                    "endpoint_id": "ui-main-simulator",
                    "role": "ui",
                }
            )

    _certificate(output / "curve/media", "simulator-media")
    _certificate(output / "curve/media", "robot-media")
    (output / "curve/endpoints.yaml").write_text(
        yaml.safe_dump(
            {"schema_version": 1, "clients": registry},
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    turn_secret = secrets.token_urlsafe(48)
    turn_secret_path = output / "turn.secret"
    turn_secret_path.write_text(turn_secret + "\n", encoding="utf-8")
    _secret(turn_secret_path)
    coturn_env.parent.mkdir(parents=True, exist_ok=True)
    coturn_env.write_text(
        f"TURN_PUBLIC_IP={turn_public_ip}\n"
        f"TURN_REALM={turn_realm}\n"
        f"TURN_STATIC_AUTH_SECRET={turn_secret}\n",
        encoding="utf-8",
    )
    _secret(coturn_env)

    print(f"generated credentials: {output}")
    print(f"router public certificate: {router_public}")
    print(f"coturn environment: {coturn_env}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--coturn-env", type=Path, default=DEFAULT_COTURN_ENV)
    parser.add_argument("--turn-public-ip", required=True)
    parser.add_argument("--turn-realm", required=True)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    generate(
        args.output.resolve(),
        args.coturn_env.resolve(),
        turn_public_ip=str(args.turn_public_ip),
        turn_realm=str(args.turn_realm),
        force=bool(args.force),
    )


if __name__ == "__main__":
    main()
