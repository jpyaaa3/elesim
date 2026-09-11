"""Render the host-side self-update wrapper for an owned EleSim install."""

from __future__ import annotations

import os
import re
import shlex
from pathlib import Path
from typing import Mapping, Sequence

from .operation_lock import render_lock_preamble
from .state import DEFAULT_SOURCE_REF, DEFAULT_SOURCE_REPOSITORY


_LOCAL_IMAGE = re.compile(r"^elesim/[a-z0-9][a-z0-9_.-]{0,127}:local$")
_INSTALL_IMAGE = re.compile(
    r"^elesim/[a-z0-9][a-z0-9_.-]{0,127}:([0-9a-f]{32})-([0-9a-f]{64})$"
)
_SOURCE_REVISION = re.compile(r"(?:git-[0-9a-f]{40}|sha256-[0-9a-f]{64})$")


def _image_belongs_to_install(image: str, install_uuid: str) -> bool:
    if _LOCAL_IMAGE.fullmatch(image):
        return True
    match = _INSTALL_IMAGE.fullmatch(image)
    return match is not None and match.group(1) == install_uuid.replace("-", "")


def render_update_wrapper(
    *,
    prefix: Path,
    state_path: Path,
    compose: Path | None = None,
    compose_wrapper: Path | None = None,
    build_services: Sequence[str] = (),
    preamble: str = "",
    repository: str | None = None,
    ref: str | None = None,
    runtime_uid: int | None = None,
    install_uuid: str | None = None,
    owned_images: Sequence[str] = (),
    runtime_snapshot: Path | None = None,
    source_revision: str | None = None,
    publish_roles: Sequence[str] = (),
    fetch_source: bool = True,
) -> str:
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
    if install_uuid is not None and any(
        not _image_belongs_to_install(value, install_uuid)
        for value in normalized_owned_images
    ):
        raise ValueError(
            "owned_images must contain legacy :local or current-install immutable names"
        )
    if normalized_owned_images and install_uuid is None:
        raise ValueError("owned_images requires install_uuid")
    normalized_publish_roles = tuple(str(role).strip() for role in publish_roles)
    if runtime_snapshot is not None:
        if install_uuid is None:
            raise ValueError("runtime_snapshot requires install_uuid")
        if not normalized_publish_roles:
            raise ValueError("runtime_snapshot requires publish_roles")
        if len(set(normalized_publish_roles)) != len(normalized_publish_roles):
            raise ValueError("publish_roles must not contain duplicates")
        if any(role not in {"pilot", "sim", "ui"} for role in normalized_publish_roles):
            raise ValueError("publish_roles contains an unsupported runtime role")
        if source_revision is not None and not _SOURCE_REVISION.fullmatch(str(source_revision)):
            raise ValueError(
                "scoped release publication requires canonical ELESIM_SOURCE_REVISION"
            )
        if not fetch_source and source_revision is None:
            raise ValueError(
                "a release wrapper must embed its authenticated source revision"
            )
        if normalized_owned_images:
            raise ValueError(
                "immutable release updates must preserve historical image IDs"
            )
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
        *(
            render_lock_preamble(
                lock_path=prefix / "instances" / ".locks" / "install.lock",
                maintenance_root=prefix / "maintenance",
            )
            if runtime_snapshot is not None
            else ()
        ),
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
    ]
    revision_file = prefix / "maintenance" / ".bootstrap-source-revision"
    if runtime_snapshot is not None and fetch_source:
        # The bootstrap runs in a short-lived child process.  Its authenticated
        # revision therefore cannot be returned through the child's
        # environment.  Remove any previous handoff before fetching so a
        # failed/partial bootstrap can never make the following publication
        # reuse an old revision.
        lines.extend(
            (
                "release_revision_file=" + shlex.quote(str(revision_file)),
                'rm -f -- "$release_revision_file"',
            )
        )
    if fetch_source:
        lines.extend(
            (
                f"recorded_repository={shlex.quote(recorded_repository)}",
                f"recorded_ref={shlex.quote(recorded_ref)}",
                'repository="${ELESIM_REPOSITORY:-$recorded_repository}"',
                'ref="${ELESIM_REF:-$recorded_ref}"',
                'if [[ -z "$repository" || -z "$ref" || "$repository" == *[[:space:]]* || "$ref" == *[[:space:]]* ]]; then',
                '  printf \'%s\\n\' \'EleSim update refused: repository/ref must be non-empty single-line values.\' >&2',
                "  exit 2",
                "fi",
                'printf \'[elesim-update] source=%s@%s\\n\' "$repository" "$ref"',
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
                + (
                    'ELESIM_SCOPED_UPDATE=1 '
                    'ELESIM_SOURCE_REVISION_FILE="$release_revision_file" '
                    if runtime_snapshot is not None
                    else ""
                )
                + "bash -s -- "
                f"--state {shlex.quote(str(state_path))} update",
            )
        )
    if compose is not None:
        compose_command = (
            shlex.quote(str(compose_wrapper))
            if compose_wrapper is not None
            else "docker compose"
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
                    "    printf '[elesim-update] preserving previous image: %s (still referenced by an existing container)\\n' \"$elesim_image_id\" >&2",
                    "    return",
                    "  fi",
                    "  if ! docker image rm \"$elesim_image_id\" >/dev/null; then",
                    "    printf '[elesim-update] failed to remove previous image: %s\\n' \"$elesim_image_id\" >&2",
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
    elif runtime_snapshot is None:
        lines.extend(
            (
                "printf '%s\\n' '[elesim-update] update and incremental image build completed.'",
                "printf '%s\\n' '[elesim-update] running containers were not replaced; run elesim-up to apply.'",
            )
        )
    if runtime_snapshot is not None:
        lines.extend(
            _render_release_publish_lines(
                compose=compose,
                compose_wrapper=compose_wrapper,
                state_path=state_path,
                runtime_snapshot=runtime_snapshot,
                source_revision=(None if source_revision is None else str(source_revision)),
                source_revision_file=(revision_file if fetch_source else None),
                install_uuid=str(install_uuid),
                roles=normalized_publish_roles,
                prefix=prefix,
            )
        )
        lines.extend(
            (
                "printf '%s\\n' '[elesim-update] immutable release published.'",
                "printf '%s\\n' '[elesim-update] registered instances remain pinned; replace a selected system explicitly to adopt it.'",
            )
        )
    lines.append("")
    return "\n".join(lines)


def _render_release_publish_lines(
    *,
    compose: Path,
    compose_wrapper: Path | None,
    state_path: Path,
    runtime_snapshot: Path,
    source_revision: str | None,
    source_revision_file: Path | None,
    install_uuid: str,
    roles: Sequence[str],
    prefix: Path,
) -> tuple[str, ...]:
    """Render the bounded host-to-tools release publication handoff.

    Docker is queried on the host because the tools service intentionally has
    no Docker socket.  Only IDs and the three ownership labels are serialized;
    the tools-side publisher validates the complete evidence before writing a
    release.  The temporary file is created below the installation prefix and
    is private even when a caller has an unusual umask.
    """

    compose_command = (
        shlex.quote(str(compose_wrapper))
        if compose_wrapper is not None
        else "docker compose"
    )
    compose_prefix = f"{compose_command} -f {shlex.quote(str(compose))}"
    specs = " ".join(shlex.quote(role) for role in roles)
    project = "elesim-runtime-" + install_uuid.replace("-", "")
    revision_lines: tuple[str, ...]
    if source_revision is not None:
        revision_lines = ("release_source_revision=" + shlex.quote(source_revision),)
    elif source_revision_file is not None:
        revision_lines = (
            "release_revision_file=" + shlex.quote(str(source_revision_file)),
            'if [[ -L "$release_revision_file" || ! -f "$release_revision_file" ]]; then printf \'%s\\n\' \'authenticated source revision handoff is missing\' >&2; exit 64; fi',
            'release_source_revision="$(<"$release_revision_file")"',
            'if [[ ! $release_source_revision =~ ^(git-[0-9a-f]{40}|sha256-[0-9a-f]{64})$ ]]; then printf \'%s\\n\' \'authenticated source revision handoff is invalid\' >&2; exit 64; fi',
        )
    else:
        raise ValueError("release publication requires an authenticated source revision")
    return (
        *revision_lines,
        "release_snapshot=" + shlex.quote(str(runtime_snapshot)),
        "release_state=" + shlex.quote(str(state_path)),
        "release_install_uuid=" + shlex.quote(install_uuid),
        "release_project=" + shlex.quote(project),
        "release_platform=",
        "case $(uname -m) in x86_64|amd64) release_platform=linux/amd64 ;; aarch64|arm64) release_platform=linux/arm64 ;; *) printf '%s\\n' 'unsupported host architecture for release publication' >&2; exit 64 ;; esac",
        "release_evidence=",
        "umask 077",
        "release_evidence=\"$(mktemp " + shlex.quote(str(prefix / 'maintenance/.release-evidence.XXXXXX')) + ")\"",
        "release_evidence_cleanup() { rm -f -- \"$release_evidence\"; }",
        "trap release_evidence_cleanup EXIT",
        "printf '%s' '{\"schema_version\":1,\"install_uuid\":\"'\"$release_install_uuid\"'\",\"project\":\"'\"$release_project\"'\",\"platform\":\"'\"$release_platform\"'\",\"roles\":{' >\"$release_evidence\"",
        "release_evidence_first=1",
        f"release_compose_images=\"$({compose_prefix} config --images 2>/dev/null || true)\"",
        "for release_role in " + specs + "; do",
        "  release_image=",
        "  while IFS= read -r release_candidate; do",
        "    if [[ $release_candidate == \"elesim/$release_role:\"* ]]; then release_image=$release_candidate; break; fi",
        "  done <<< \"$release_compose_images\"",
        "  if [[ ! $release_image =~ ^elesim/(pilot|sim|ui):[0-9a-f]{32}-[0-9a-f]{64}$ || $release_image != \"elesim/$release_role:\"* ]]; then printf 'invalid scoped image for role %s\\n' \"$release_role\" >&2; exit 70; fi",
        "  release_entry=\"$(docker image inspect --format '{\"image_reference\":\"'\"$release_image\"'\",\"image_id\":{{json .Id}},\"install_uuid\":{{json (index .Config.Labels \"io.elesim.install_uuid\")}},\"build_fingerprint\":{{json (index .Config.Labels \"io.elesim.build_fingerprint\")}},\"project\":{{json (index .Config.Labels \"com.docker.compose.project\")}}}' \"$release_image\")\"",
        "  [[ $release_entry == *'\\n'* ]] && { printf '%s\\n' 'invalid Docker evidence' >&2; exit 70; }",
        "  if (( release_evidence_first )); then release_evidence_first=0; else printf '%s' ',' >>\"$release_evidence\"; fi",
        "  printf '\"%s\":%s' \"$release_role\" \"$release_entry\" >>\"$release_evidence\"",
        "done",
        "printf '%s\\n' '}}' >>\"$release_evidence\"",
        f"{compose_prefix} run --rm --no-deps --no-build tools elesim-setup --state \"$release_state\" release publish --source-revision \"$release_source_revision\" --snapshot \"$release_snapshot\" --evidence \"$release_evidence\"",
        "trap - EXIT",
        "release_evidence_cleanup",
        *(
            ('rm -f -- "$release_revision_file"',)
            if source_revision_file is not None
            else ()
        ),
    )


def render_release_wrapper(
    *,
    prefix: Path,
    state_path: Path,
    compose: Path,
    compose_wrapper: Path | None,
    build_services: Sequence[str],
    preamble: str = "",
    source_revision: str | None,
    runtime_snapshot: Path,
    install_uuid: str,
    release_images: Sequence[str],
    runtime_uid: int | None = None,
) -> str:
    """Render the explicit first-release command for a scoped installation."""

    roles = tuple(
        image.split("/", 1)[1].split(":", 1)[0] for image in release_images
    )
    return render_update_wrapper(
        prefix=prefix,
        state_path=state_path,
        compose=compose,
        compose_wrapper=compose_wrapper,
        build_services=build_services,
        preamble=preamble,
        runtime_uid=runtime_uid,
        install_uuid=install_uuid,
        runtime_snapshot=runtime_snapshot,
        source_revision=source_revision,
        publish_roles=roles,
        fetch_source=False,
    )


__all__ = [
    "render_release_wrapper",
    "render_update_wrapper",
]
