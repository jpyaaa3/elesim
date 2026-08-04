#!/usr/bin/env python3
"""Generate only the shared HMAC secret used by managed Coturn."""

from __future__ import annotations

import argparse
import os
import secrets
import stat
from pathlib import Path


DEFAULT_OUTPUT = Path(
    os.environ.get(
        "ELESIM_COTURN_STATE",
        str(Path.home() / ".local/share/elesim/coturn"),
    )
)
DEFAULT_COTURN_ENV = DEFAULT_OUTPUT / ".env"


def _private_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    path.chmod(stat.S_IRUSR | stat.S_IWUSR)


def generate(
    output: Path,
    coturn_env: Path,
    *,
    turn_public_ip: str,
    turn_realm: str,
    force: bool,
) -> None:
    public_ip = str(turn_public_ip).strip()
    realm = str(turn_realm).strip()
    if not public_ip:
        raise ValueError("TURN public IP or hostname must not be empty")
    if not realm:
        raise ValueError("TURN realm must not be empty")
    secret_path = output / "turn.secret"
    existing = [path for path in (secret_path, coturn_env) if path.exists()]
    if existing and not force:
        rendered = ", ".join(str(path) for path in existing)
        raise FileExistsError(
            f"TURN output already exists ({rendered}); pass --force to replace it"
        )

    secret = secrets.token_urlsafe(48)
    _private_text(secret_path, secret + "\n")
    _private_text(
        coturn_env,
        f"TURN_PUBLIC_IP={public_ip}\n"
        f"TURN_REALM={realm}\n"
        f"TURN_STATIC_AUTH_SECRET={secret}\n",
    )
    print(f"TURN secret: {secret_path}")
    print(f"Coturn environment: {coturn_env}")


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
