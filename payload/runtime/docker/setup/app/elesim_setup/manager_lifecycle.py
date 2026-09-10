"""Generate the host-side lifecycle guard for the transient manager container."""

from __future__ import annotations

import re
import shlex
from pathlib import Path


_DOCKER_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


def compose_owner_guard(
    compose: Path,
    *,
    project: str,
    containers: tuple[str, ...],
    alternate_composes: tuple[Path, ...] = (),
) -> str:
    """Reject fixed-name containers owned by another Compose installation."""

    rendered_containers = " ".join(shlex.quote(name) for name in containers)
    accepted_composes = " ".join(
        shlex.quote(str(path)) for path in (compose, *alternate_composes)
    )
    return (
        f"expected_compose={shlex.quote(str(compose))}\n"
        f"expected_composes=({accepted_composes})\n"
        f"expected_project={shlex.quote(project)}\n"
        f"for container in {rendered_containers}; do\n"
        "  if ! docker container inspect \"$container\" >/dev/null 2>&1; then\n"
        "    continue\n"
        "  fi\n"
        "  metadata=\"$(docker container inspect --format "
        "'{{ index .Config.Labels \"com.docker.compose.project\" }}|"
        "{{ index .Config.Labels \"com.docker.compose.project.config_files\" }}' "
        "\"$container\")\"\n"
        "  actual_project=\"${metadata%%|*}\"\n"
        "  actual_compose=\"${metadata#*|}\"\n"
        "  compose_match=0\n"
        "  IFS=',' read -r -a compose_files <<<\"$actual_compose\"\n"
        "  for compose_file in \"${compose_files[@]}\"; do\n"
        "    for expected_compose in \"${expected_composes[@]}\"; do\n"
        "      if [[ \"$compose_file\" == \"$expected_compose\" ]]; then\n"
        "        compose_match=1\n"
        "        break 2\n"
        "      fi\n"
        "    done\n"
        "  done\n"
        "  if [[ \"$actual_project\" != \"$expected_project\" || $compose_match != 1 ]]; then\n"
        "    printf 'EleSim 고정 컨테이너 이름 충돌: %s\\n' \"$container\" >&2\n"
        "    printf '  기존 소유자: project=%s compose=%s\\n' "
        "\"$actual_project\" \"$actual_compose\" >&2\n"
        "    printf '  현재 설치: project=%s compose=%s\\n' "
        "\"$expected_project\" \"$expected_compose\" >&2\n"
        "    printf '기존 설치의 elesim-down으로 종료·제거한 뒤 다시 실행하십시오.\\n' >&2\n"
        "    exit 73\n"
        "  fi\n"
        "done\n"
    )


def manager_lifecycle_fragment(
    install_uuid: str,
    *,
    container_name: str = "elesim-manager",
    container_name_variable: str | None = None,
) -> str:
    """Return shell code that protects and cleans ``elesim-manager``.

    A manager from another invocation or installation is never removed
    automatically. A stopped manager may be removed at startup only when its
    install UUID matches this installation. Once this wrapper starts its own
    manager, EXIT cleanup is restricted to the current install UUID and
    force-removes that owned container.
    """

    if container_name_variable is not None:
        if not re.fullmatch(r"[a-zA-Z_][a-zA-Z0-9_]*", container_name_variable):
            raise ValueError(f"invalid manager container variable: {container_name_variable!r}")
        # The variable is assigned only after strict system-id validation in
        # the generated wrapper.  It therefore cannot inject shell syntax or
        # a Docker filter expression.
        container = f'"${container_name_variable}"'
        filter_name = f'"name=^/${{{container_name_variable}}}$"'
        display_name = f'"${container_name_variable}"'
    else:
        if not _DOCKER_NAME.fullmatch(str(container_name)):
            raise ValueError(f"invalid manager container name: {container_name!r}")
        container = shlex.quote(str(container_name))
        filter_name = shlex.quote(f"name=^/{re.escape(container_name)}$")
        display_name = container
    quoted_uuid = shlex.quote(install_uuid)
    inspect_format = "'{{.Id}}|{{.State.Running}}|{{index .Config.Labels \"io.elesim.install_uuid\"}}|{{index .Config.Labels \"io.elesim.manager_invocation\"}}'"
    return (
        "manager_started=0\n"
        "manager_invocation_token=\"$(date -u +%s%N)-$$-$RANDOM-$RANDOM\"\n"
        "manager_cleanup() {\n"
        "  local metadata manager_id manager_state manager_owner manager_invocation\n"
        f"  metadata=\"$(docker inspect -f {inspect_format} {container} 2>/dev/null || true)\"\n"
        "  IFS='|' read -r manager_id manager_state manager_owner manager_invocation <<<\"$metadata\"\n"
        "  if [[ $manager_started == 1 && $manager_owner == "
        + quoted_uuid
        + " && $manager_invocation == \"$manager_invocation_token\" && -n $manager_id ]]; then\n"
        "    docker rm -f \"$manager_id\" >/dev/null 2>&1 || true\n"
        "  fi\n"
        "}\n"
        "trap manager_cleanup EXIT\n"
        f"existing_manager=\"$(docker ps -aq --filter {filter_name})\"\n"
        "if [[ -n $existing_manager ]]; then\n"
        f"  metadata=\"$(docker inspect -f {inspect_format} \"$existing_manager\")\"\n"
        "  IFS='|' read -r manager_id manager_state manager_owner manager_invocation <<<\"$metadata\"\n"
        "  if [[ $manager_state == true ]]; then\n"
        f"    printf '%s가 이미 실행 중입니다. 기존 연결관리자를 종료하거나 다른 터미널을 사용하십시오.\\n' {display_name} >&2\n"
        "    exit 73\n"
        "  fi\n"
        "  if [[ $manager_owner != "
        + quoted_uuid
        + " || -z $manager_invocation || -z $manager_id ]]; then\n"
        f"    printf '기존 %s는 다른 설치 또는 invocation 소유입니다. 기존 연결관리자를 종료하거나 다른 터미널을 사용하십시오.\\n' {display_name} >&2\n"
        "    exit 73\n"
        "  fi\n"
        "  docker rm \"$manager_id\" >/dev/null\n"
        "fi\n"
    )


def host_helper_fragment(
    *,
    maintenance_root: Path,
    compose_argument: str,
    bin_dir_argument: str,
    project: str,
    instance_system_argument: str = "",
) -> str:
    """Start a private host broker and mount only its Unix socket."""

    return (
        "host_helper_dir=\"$(mktemp -d \"${TMPDIR:-/tmp}/elesim-host-helper.XXXXXX\")\"\n"
        "chmod 0700 \"$host_helper_dir\"\n"
        "host_helper_socket=\"$host_helper_dir/helper.sock\"\n"
        "host_helper_pid=\n"
        "host_helper_cleanup() {\n"
        "  if [[ -n $host_helper_pid ]]; then\n"
        "    kill \"$host_helper_pid\" >/dev/null 2>&1 || true\n"
        "    wait \"$host_helper_pid\" >/dev/null 2>&1 || true\n"
        "  fi\n"
        "  rm -rf -- \"$host_helper_dir\"\n"
        "}\n"
        "trap 'host_helper_cleanup; manager_cleanup' EXIT\n"
        "tailscale_bin=\"$(command -v tailscale 2>/dev/null || true)\"\n"
        "host_helper_args=(--socket \"$host_helper_socket\" --compose "
        + compose_argument
        + " --bin-dir "
        + bin_dir_argument
        + " --project "
        + shlex.quote(project)
        + ")\n"
        + (
            "host_helper_args+=(--instance-system "
            + instance_system_argument
            + ")\n"
            if instance_system_argument
            else ""
        )
        + "if [[ -n $tailscale_bin ]]; then\n"
        "  host_helper_args+=(--tailscale-bin \"$tailscale_bin\")\n"
        "fi\n"
        "PYTHONPATH="
        + shlex.quote(str(maintenance_root))
        + " PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 "
        "python3 -B -S -m elesim_setup.host_helper \"${host_helper_args[@]}\" &\n"
        "host_helper_pid=$!\n"
        "for _helper_attempt in {1..100}; do\n"
        "  [[ -S $host_helper_socket ]] && break\n"
        "  if ! kill -0 \"$host_helper_pid\" 2>/dev/null; then\n"
        "    printf 'EleSim host helper가 시작 전에 종료되었습니다.\\n' >&2\n"
        "    exit 2\n"
        "  fi\n"
        "  sleep 0.05\n"
        "done\n"
        "if [[ ! -S $host_helper_socket ]]; then\n"
        "  printf 'EleSim host helper socket 준비가 시간 초과되었습니다.\\n' >&2\n"
        "  exit 2\n"
        "fi\n"
        "manager_options+=(\n"
        "  -e ELESIM_HOST_HELPER_SOCKET=/run/elesim-host-helper/helper.sock\n"
        "  -v \"$host_helper_dir:/run/elesim-host-helper:rw\"\n"
        ")\n"
        "if [[ -n $tailscale_bin ]]; then\n"
        "  manager_options+=(\n"
        "    -e ELESIM_TAILSCALE_PROXY=1\n"
        "    -e ELESIM_TAILSCALE_PROXY_BIN=/usr/local/bin/elesim-host-proxy\n"
        "    -e ELESIM_TAILSCALE_PROXY_SOCKET=/run/elesim-host-helper/helper.sock\n"
        "  )\n"
        "fi\n"
    )


__all__ = [
    "compose_owner_guard",
    "host_helper_fragment",
    "manager_lifecycle_fragment",
]
