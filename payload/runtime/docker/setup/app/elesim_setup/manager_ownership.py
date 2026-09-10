"""Host-only ownership registration for scoped connection-manager runs."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .ownership import OwnershipError, append_manager_docker_ownership


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="elesim-manager-ownership",
        description="Register one exact scoped manager container before creation.",
    )
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--install-uuid", required=True)
    parser.add_argument("--project", required=True)
    parser.add_argument("--system-id", required=True)
    parser.add_argument("--docker-context", required=True)
    parser.add_argument("--docker-engine-id", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        append_manager_docker_ownership(
            manifest_path=args.manifest,
            install_uuid=args.install_uuid,
            project=args.project,
            docker_context=args.docker_context,
            docker_engine_id=args.docker_engine_id,
            system_id=args.system_id,
        )
    except (OwnershipError, OSError, ValueError) as exc:
        print(f"연결관리자 소유권 등록을 거부했습니다: {exc}", file=sys.stderr)
        return 78
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised by wrapper tests
    raise SystemExit(main())
