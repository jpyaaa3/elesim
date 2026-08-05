"""Generate the host-side lifecycle guard for the transient manager container."""

from __future__ import annotations

import shlex


def manager_lifecycle_fragment(install_uuid: str) -> str:
    """Return shell code that protects and cleans ``elesim-manager``.

    A running manager is never removed automatically.  A stopped manager is a
    disposable one-shot object, so it may be removed even when it belongs to a
    previous installation.  Once this wrapper starts its own manager, the EXIT
    cleanup is restricted to the current install UUID.
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
        "  if [[ $state == false && ( $manager_started == 0 || $owner == "
        + quoted_uuid
        + " ) ]]; then\n"
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


__all__ = ["manager_lifecycle_fragment"]
