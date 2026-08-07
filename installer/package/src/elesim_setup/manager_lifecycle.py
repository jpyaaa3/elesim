"""Generate the host-side lifecycle guard for the transient manager container."""

from __future__ import annotations

import shlex
from pathlib import Path


def manager_lifecycle_fragment(install_uuid: str) -> str:
    """Return shell code that protects and cleans ``elesim-manager``.

    A running manager from another invocation is never removed automatically.
    A stopped manager is a disposable one-shot object, so it may be removed
    even when it belongs to a previous installation.  Once this wrapper starts
    its own manager, the EXIT cleanup is restricted to the current install UUID
    and force-removes that owned container so an interrupted GUI cannot leave a
    fixed-name container blocking the next invocation.
    """

    quoted_uuid = shlex.quote(install_uuid)
    return (
        "manager_started=0\n"
        "manager_cleanup() {\n"
        "  local state owner\n"
        "  state=\"$(docker inspect -f '{{.State.Running}}' elesim-manager "
        "2>/dev/null || true)\"\n"
        "  owner=\"$(docker inspect -f '{{index .Config.Labels \"io.elesim.install_uuid\"}}' "
        "elesim-manager 2>/dev/null || true)\"\n"
        "  if [[ $manager_started == 1 && $owner == "
        + quoted_uuid
        + " ]]; then\n"
        "    docker rm -f elesim-manager >/dev/null 2>&1 || true\n"
        "  elif [[ $state == false && $manager_started == 0 ]]; then\n"
        "    docker rm elesim-manager >/dev/null 2>&1 || true\n"
        "  fi\n"
        "}\n"
        "trap manager_cleanup EXIT\n"
        "existing_manager=\"$(docker ps -aq --filter 'name=^/elesim-manager$')\"\n"
        "if [[ -n $existing_manager ]]; then\n"
        "  manager_running=\"$(docker inspect -f '{{.State.Running}}' \"$existing_manager\")\"\n"
        "  if [[ $manager_running == true ]]; then\n"
        "    printf 'elesim-manager가 이미 실행 중입니다. 기존 연결관리자를 종료하거나 다른 터미널을 사용하십시오.\\n' >&2\n"
        "    exit 73\n"
        "  fi\n"
        "  docker rm \"$existing_manager\" >/dev/null\n"
        "fi\n"
    )


def host_helper_fragment(
    *,
    maintenance_root: Path,
    compose_argument: str,
    bin_dir_argument: str,
    project: str,
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
        "if [[ -n $tailscale_bin ]]; then\n"
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
        "    printf 'Elesim host helper가 시작 전에 종료되었습니다.\\n' >&2\n"
        "    exit 2\n"
        "  fi\n"
        "  sleep 0.05\n"
        "done\n"
        "if [[ ! -S $host_helper_socket ]]; then\n"
        "  printf 'Elesim host helper socket 준비가 시간 초과되었습니다.\\n' >&2\n"
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


__all__ = ["host_helper_fragment", "manager_lifecycle_fragment"]
