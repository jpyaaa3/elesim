"""Render the host-side self-update wrapper for an owned EleSim install."""

from __future__ import annotations

import os
import re
import shlex
from pathlib import Path
from typing import Sequence

from .state import DEFAULT_SOURCE_REF, DEFAULT_SOURCE_REPOSITORY


_OWNED_IMAGE = re.compile(r"^elesim/[a-z0-9][a-z0-9_.-]{0,127}:local$")


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
    repository: str | None = None,
    ref: str | None = None,
    runtime_uid: int | None = None,
    install_uuid: str | None = None,
    owned_images: Sequence[str] = (),
) -> str:
    if edition not in {"general", "developer"}:
        raise ValueError(f"unsupported update edition: {edition!r}")
    if runtime_uid is not None and (
        isinstance(runtime_uid, bool) or not isinstance(runtime_uid, int) or runtime_uid < 0
    ):
        raise ValueError("runtime_uid must be a non-negative integer")
    if install_uuid is not None:
        install_uuid = str(install_uuid).strip()
        if not install_uuid or any(
            ch.isspace() or ch in {"'", '"', "\\", "\x00"}
            for ch in install_uuid
        ):
            raise ValueError("install_uuid must be a non-empty shell-safe value")
    normalized_owned_images = tuple(str(value).strip() for value in owned_images)
    if len(set(normalized_owned_images)) != len(normalized_owned_images):
        raise ValueError("owned_images must not contain duplicates")
    if any(
        not _OWNED_IMAGE.fullmatch(value)
        for value in normalized_owned_images
    ):
        raise ValueError("owned_images must contain exact elesim/*:local names")
    if normalized_owned_images and install_uuid is None:
        raise ValueError("owned_images requires install_uuid")
    recorded_repository = (
        os.environ.get("ELESIM_REPOSITORY", DEFAULT_SOURCE_REPOSITORY)
        if repository is None
        else repository
    ).strip()
    recorded_ref = (
        os.environ.get("ELESIM_REF", DEFAULT_SOURCE_REF) if ref is None else ref
    ).strip()
    if not recorded_repository or not recorded_ref or any(
        "\n" in value or "\r" in value or any(ch.isspace() for ch in value)
        for value in (recorded_repository, recorded_ref)
    ):
        raise ValueError("update repository/ref must be non-empty single-line values")

    lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        preamble.rstrip("\n"),
        *(
            [
                f"expected_update_uid={shlex.quote(str(runtime_uid))}",
                'actual_update_uid="$(id -u)"',
                'if [[ "$actual_update_uid" != "$expected_update_uid" ]]; then',
                "  printf '%s\\n' 'EleSim update must run as the user that owns this installation.' >&2",
                '  printf \'  expected UID: %s; current UID: %s\\n\' "$expected_update_uid" "$actual_update_uid" >&2',
                "  exit 77",
                "fi",
            ]
            if runtime_uid is not None
            else []
        ),
        f"recorded_repository={shlex.quote(recorded_repository)}",
        f"recorded_ref={shlex.quote(recorded_ref)}",
        'repository="${ELESIM_REPOSITORY:-$recorded_repository}"',
        'ref="${ELESIM_REF:-$recorded_ref}"',
        'if [[ -z "$repository" || -z "$ref" || "$repository" == *[[:space:]]* || "$ref" == *[[:space:]]* ]]; then',
        '  printf \'%s\\n\' \'EleSim update refused: repository/ref must be non-empty single-line values.\' >&2',
        "  exit 2",
        "fi",
        'printf \'[elesim-update] source=%s@%s\\n\' "$repository" "$ref"',
    ]
    if edition == "developer":
        workspace = shlex.quote(str(prefix))
        lines.extend(
            (
                f"if ! git -C {workspace} diff --quiet || "
                f"! git -C {workspace} diff --cached --quiet; then",
                "  printf '%s\\n' 'EleSim update refused: tracked workspace changes must be committed or stashed.' >&2",
                "  exit 73",
                "fi",
                f"git -C {workspace} fetch --prune origin \"$ref\"",
                f"git -C {workspace} merge --ff-only FETCH_HEAD",
            )
        )
    raw_url = (
        '"https://raw.githubusercontent.com/${repository}/${ref}/'
        'installer/bootstrap/install.sh"'
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
        build_line = (
            f"{compose_command} --progress plain "
            f"-f {shlex.quote(str(compose))} build{suffix}"
        )
        if normalized_owned_images:
            # Compose retags a rebuilt service image and leaves the previous
            # image ID dangling.  Capture only the exact tagged IDs that
            # existed before this update, then remove an old ID only when it
            # carries this install's label, has no remaining repository tag,
            # and no container (running or stopped) still references it.
            # This deliberately avoids image-prune and cannot touch foreign
            # projects or untracked build layers.
            rendered_images = " ".join(
                shlex.quote(value) for value in normalized_owned_images
            )
            lines.extend(
                (
                    f"elesim_owned_images=({rendered_images})",
                    f"elesim_expected_install_uuid={shlex.quote(install_uuid or '')}",
                    "elesim_previous_image_names=()",
                    "elesim_previous_image_ids=()",
                    "for elesim_image_name in \"${elesim_owned_images[@]}\"; do",
                    "  elesim_image_id=\"$(docker image inspect \"$elesim_image_name\" --format '{{.Id}}' 2>/dev/null || true)\"",
                    "  if [[ -n \"$elesim_image_id\" ]]; then",
                    "    elesim_previous_image_names+=(\"$elesim_image_name\")",
                    "    elesim_previous_image_ids+=(\"$elesim_image_id\")",
                    "  fi",
                    "done",
                    "elesim_cleanup_owned_dangling_image() {",
                    "  local elesim_image_id=\"$1\"",
                    "  local elesim_image_tags",
                    "  local elesim_real_image_tags",
                    "  local elesim_image_install_uuid",
                    "  if [[ -z \"$elesim_image_id\" ]]; then",
                    "    return",
                    "  fi",
                    "  elesim_image_tags=\"$(docker image inspect \"$elesim_image_id\" --format '{{range .RepoTags}}{{println .}}{{end}}' 2>/dev/null || true)\"",
                    "  elesim_real_image_tags=\"${elesim_image_tags//<none>:<none>/}\"",
                    "  if [[ \"$elesim_real_image_tags\" =~ [^[:space:]] ]]; then",
                    "    return",
                    "  fi",
                    "  elesim_image_install_uuid=\"$(docker image inspect \"$elesim_image_id\" --format '{{if .Config.Labels}}{{index .Config.Labels \"io.elesim.install_uuid\"}}{{end}}' 2>/dev/null || true)\"",
                    "  if [[ \"$elesim_image_install_uuid\" != \"$elesim_expected_install_uuid\" ]]; then",
                    "    return",
                    "  fi",
                    "  if [[ -n \"$(docker ps -aq --filter \"ancestor=$elesim_image_id\" 2>/dev/null || true)\" ]]; then",
                    "    printf '[elesim-update] 이전 이미지 보존: %s (기존 컨테이너가 참조 중)\\n' \"$elesim_image_id\" >&2",
                    "    return",
                    "  fi",
                    "  if ! docker image rm \"$elesim_image_id\" >/dev/null; then",
                    "    printf '[elesim-update] 이전 이미지 정리 실패: %s\\n' \"$elesim_image_id\" >&2",
                    "  fi",
                    "}",
                )
            )
        lines.append(build_line)
        if normalized_owned_images:
            lines.extend(
                (
                    "for elesim_image_index in \"${!elesim_previous_image_names[@]}\"; do",
                    "  elesim_image_name=\"${elesim_previous_image_names[$elesim_image_index]}\"",
                    "  elesim_old_image_id=\"${elesim_previous_image_ids[$elesim_image_index]}\"",
                    "  elesim_new_image_id=\"$(docker image inspect \"$elesim_image_name\" --format '{{.Id}}' 2>/dev/null || true)\"",
                    "  if [[ -z \"$elesim_new_image_id\" || \"$elesim_new_image_id\" == \"$elesim_old_image_id\" ]]; then",
                    "    continue",
                    "  fi",
                    "  elesim_cleanup_owned_dangling_image \"$elesim_old_image_id\"",
                    "done",
                    "if elesim_owned_dangling_ids=\"$(docker image ls --all --no-trunc --filter \"dangling=true\" --filter \"label=io.elesim.install_uuid=$elesim_expected_install_uuid\" --format '{{.ID}}' 2>/dev/null)\"; then",
                    "  while IFS= read -r elesim_dangling_id; do",
                    "    [[ -n \"$elesim_dangling_id\" ]] || continue",
                    "    elesim_cleanup_owned_dangling_image \"$elesim_dangling_id\"",
                    "  done <<< \"$elesim_owned_dangling_ids\"",
                    "fi",
                )
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
