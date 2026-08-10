"""Render the host-side self-update wrapper for an owned Elesim install."""

from __future__ import annotations

import os
import shlex
from pathlib import Path
from typing import Sequence


def render_update_wrapper(
    *,
    edition: str,
    prefix: Path,
    state_path: Path,
    compose: Path | None = None,
    compose_wrapper: Path | None = None,
    build_services: Sequence[str] = (),
    pull_services: Sequence[str] = (),
    preamble: str = "",
) -> str:
    if edition not in {"general", "developer"}:
        raise ValueError(f"unsupported update edition: {edition!r}")
    repository = os.environ.get("ELESIM_REPOSITORY", "jpyaaa3/elesim").strip()
    ref = os.environ.get("ELESIM_REF", "main").strip()
    if not repository or not ref or any(
        "\n" in value or "\r" in value for value in (repository, ref)
    ):
        raise ValueError("update repository/ref must be non-empty single-line values")

    lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        preamble.rstrip("\n"),
        f"repository={shlex.quote(repository)}",
        f"ref={shlex.quote(ref)}",
    ]
    if edition == "developer":
        workspace = shlex.quote(str(prefix))
        lines.extend(
            (
                f"if ! git -C {workspace} diff --quiet || "
                f"! git -C {workspace} diff --cached --quiet; then",
                "  printf '%s\\n' 'Elesim update refused: tracked workspace changes must be committed or stashed.' >&2",
                "  exit 73",
                "fi",
                f"git -C {workspace} fetch --prune origin \"$ref\"",
                f"git -C {workspace} merge --ff-only FETCH_HEAD",
            )
        )
    raw_url = (
        '"https://raw.githubusercontent.com/${repository}/${ref}/'
        'installer/bootstrap/bootstrap.sh"'
    )
    lines.extend(
        (
            f"curl -fsSL {raw_url} | "
            f"ELESIM_REPOSITORY=\"$repository\" ELESIM_REF=\"$ref\" "
            f"ELESIM_INVOCATION_DIR={shlex.quote(str(prefix))} "
            "bash -s -- "
            f"--state {shlex.quote(str(state_path))} update --edition {edition}",
        )
    )
    if compose is not None:
        compose_command = (
            shlex.quote(str(compose_wrapper))
            if compose_wrapper is not None
            else "docker compose"
        )
        if pull_services:
            pulls = " ".join(shlex.quote(value) for value in pull_services)
            lines.append(
                f"{compose_command} -f {shlex.quote(str(compose))} pull {pulls}"
            )
        services = " ".join(shlex.quote(value) for value in build_services)
        suffix = f" {services}" if services else ""
        lines.append(
            f"{compose_command} --progress plain "
            f"-f {shlex.quote(str(compose))} build{suffix}"
        )
    if compose is None:
        lines.append("printf '%s\\n' '[elesim-update] update completed.'")
    else:
        lines.extend(
            (
                "printf '%s\\n' '[elesim-update] update and incremental image build completed.'",
                "printf '%s\\n' '[elesim-update] running containers were not replaced; run elesim-up to apply.'",
            )
        )
        if "tailscale" in pull_services:
            lines.append(
                "printf '%s\\n' '[elesim-update] before elesim-up, prepare the runtime network in elesim-connections or run elesim-tailscale login.'"
            )
    lines.append("")
    return "\n".join(lines)


__all__ = ["render_update_wrapper"]
