"""Render host-local runtime status wrappers.

The lifecycle commands intentionally operate on one installation and one
Docker daemon.  ``elesim-status`` follows the same boundary: it reports the
current host, the container/runtime address, resource counters, GPU policy and
the media facts that Sim has actually logged.  It does not pretend to probe a
remote host or to infer WebRTC state from DDS discovery.
"""

from __future__ import annotations

import shlex
from pathlib import Path
from typing import Iterable


def _quoted(value: object) -> str:
    return shlex.quote(str(value))


def render_compose_status_wrapper(
    *,
    compose: Path,
    project: str,
    edition: str,
    services: Iterable[tuple[str, str]],
    guard: str = "",
    sim_container: str | None = None,
) -> str:
    """Render a read-only status command for a Compose installation.

    ``services`` contains ``(display-name, fixed-container-name)`` pairs.
    Fixed names are used deliberately: generated Compose files own those
    names, while the owner guard prevents a different installation from being
    inspected or modified accidentally.
    """

    service_calls: list[str] = []
    sim_rendered = False
    for label, container in services:
        service_calls.append(
            "status_container " + _quoted(label) + " " + _quoted(container)
        )
        if str(label).strip().lower() == "ui":
            service_calls.append("status_ui_media " + _quoted(container))
        if sim_container is not None and str(container) == str(sim_container):
            service_calls.append("status_sim_media " + _quoted(sim_container))
            sim_rendered = True
    rendered_calls = "\n".join(service_calls)
    sim_call = (
        "status_sim_media " + _quoted(sim_container) + "\n"
        if sim_container is not None and not sim_rendered
        else ""
    )
    command = "docker compose -f " + _quoted(compose)
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "umask 077\n"
        + guard
        + "if (( $# == 1 )) && [[ $1 == --gpu-devices ]]; then\n"
        + "  if command -v nvidia-smi >/dev/null 2>&1; then\n"
        + "    nvidia-smi --query-gpu=index,uuid --format=csv,noheader,nounits 2>/dev/null || true\n"
        + "  fi\n"
        + "  exit 0\n"
        + "fi\n"
        + "host_name=\"$(hostname -f 2>/dev/null || hostname 2>/dev/null || printf unknown)\"\n"
        "host_ips=\"$(hostname -I 2>/dev/null | tr '\\n' ' ' | xargs 2>/dev/null || true)\"\n"
        "[[ -n $host_ips ]] || host_ips=unknown\n"
        "printf 'EleSim status (current-host only)\\n'\n"
        "printf 'edition=%s\\n' "
        + _quoted(edition)
        + "\n"
        "printf 'compose_project=%s\\n' "
        + _quoted(project)
        + "\n"
        "printf 'compose_file=%s\\n' "
        + _quoted(compose)
        + "\n"
        "printf 'host=%s\\n' \"$host_name\"\n"
        "printf 'host_ips=%s\\n' \"$host_ips\"\n"
        "printf 'scope=run this command on each host; remote topology is managed by elesim-connections\\n'\n"
        "if (( $# != 0 )); then\n"
        "  printf '사용법: elesim-status [--gpu-devices]\\n' >&2\n"
        "  exit 64\n"
        "fi\n"
        "printf '%s\\n' '--- services ---'\n"
        "status_container() {\n"
        "  local label=$1 container=$2\n"
        "  local status state exit_code oom_killed pid restart_count image network_mode runtime_ip stats environment cvd nvd device_requests domain_id rmw dds_interface security_profile\n"
        "  if ! docker container inspect \"$container\" >/dev/null 2>&1; then\n"
        "    printf '[%s] container=%s state=absent\\n' \"$label\" \"$container\"\n"
        "    return 0\n"
        "  fi\n"
        "  status=\"$(docker inspect -f '{{.State.Status}}|{{.State.ExitCode}}|{{.State.OOMKilled}}|{{.State.Pid}}|{{.RestartCount}}' \"$container\" 2>/dev/null || printf 'unknown|?|?|?|?')\"\n"
        "  image=\"$(docker inspect -f '{{.Config.Image}}' \"$container\" 2>/dev/null || printf unknown)\"\n"
        "  network_mode=\"$(docker inspect -f '{{.HostConfig.NetworkMode}}' \"$container\" 2>/dev/null || printf unknown)\"\n"
        "  runtime_ip=\"$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}} {{end}}' \"$container\" 2>/dev/null | xargs 2>/dev/null || true)\"\n"
        "  if [[ $network_mode == service:tailscale ]]; then\n"
        "    runtime_ip=\"$(docker exec elesim-tailscale tailscale ip -4 2>/dev/null | xargs 2>/dev/null || true)\"\n"
        "  fi\n"
        "  if [[ -z $runtime_ip ]]; then\n"
        "    runtime_ip=$host_ips\n"
        "  fi\n"
        "  stats=\"$(docker stats --no-stream --format '{{.CPUPerc}}|{{.MemUsage}}|{{.MemPerc}}|{{.PIDs}}' \"$container\" 2>/dev/null || printf 'unavailable|unavailable|unavailable|unavailable')\"\n"
        "  environment=\"$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' \"$container\" 2>/dev/null || true)\"\n"
        "  cvd=\"$(printf '%s\\n' \"$environment\" | sed -n 's/^CUDA_VISIBLE_DEVICES=//p' | head -n1)\"\n"
        "  [[ -n $cvd ]] || cvd=unset\n"
        "  nvd=\"$(printf '%s\\n' \"$environment\" | sed -n 's/^NVIDIA_VISIBLE_DEVICES=//p' | head -n1)\"\n"
        "  [[ -n $nvd ]] || nvd=unset\n"
        "  device_requests=\"$(docker inspect -f '{{json .HostConfig.DeviceRequests}}' \"$container\" 2>/dev/null || printf unknown)\"\n"
        "  domain_id=\"$(printf '%s\\n' \"$environment\" | sed -n 's/^ROS_DOMAIN_ID=//p' | head -n1)\"\n"
        "  rmw=\"$(printf '%s\\n' \"$environment\" | sed -n 's/^RMW_IMPLEMENTATION=//p' | head -n1)\"\n"
        "  dds_interface=\"$(printf '%s\\n' \"$environment\" | sed -n 's/^ELESIM_DDS_NETWORK_INTERFACE=//p' | head -n1)\"\n"
        "  security_profile=\"$(printf '%s\\n' \"$environment\" | sed -n 's/^ELESIM_DDS_SECURITY_PROFILE=//p' | head -n1)\"\n"
        "  [[ -n $domain_id ]] || domain_id=unset\n"
        "  [[ -n $rmw ]] || rmw=unset\n"
        "  [[ -n $dds_interface ]] || dds_interface=automatic\n"
        "  [[ -n $security_profile ]] || security_profile=unset\n"
        "  IFS='|' read -r state exit_code oom_killed pid restart_count <<<\"$status\"\n"
        "  printf '[%s] container=%s state=%s exit=%s oom=%s pid=%s restarts=%s\\n' \"$label\" \"$container\" \"$state\" \"$exit_code\" \"$oom_killed\" \"$pid\" \"$restart_count\"\n"
        "  printf '  image=%s network=%s runtime_ip=%s\\n' \"$image\" \"$network_mode\" \"$runtime_ip\"\n"
        "  printf '  gpu.cuda_visible_devices=%s\\n' \"$cvd\"\n"
        "  printf '  gpu.nvidia_visible_devices=%s\\n' \"$nvd\"\n"
        "  printf '  gpu.device_requests=%s\\n' \"$device_requests\"\n"
        "  printf '  dds.domain_id=%s rmw=%s interface=%s security=%s\\n' \"$domain_id\" \"$rmw\" \"$dds_interface\" \"$security_profile\"\n"
        "  IFS='|' read -r cpu_percent memory_use memory_percent pids <<<\"$stats\"\n"
        "  printf '  resources.cpu_percent=%s memory=%s memory_percent=%s pids=%s\\n' \"$cpu_percent\" \"$memory_use\" \"$memory_percent\" \"$pids\"\n"
        "}\n"
        "status_ui_media() {\n"
        "  local container=$1 receiver_lines\n"
        "  if ! docker container inspect \"$container\" >/dev/null 2>&1; then\n"
        "    return 0\n"
        "  fi\n"
        "  receiver_lines=\"$(docker logs --tail 1000 \"$container\" 2>&1 | grep -E '\\[ui-webrtc\\].*(track=|connection=|receive:|decode:|answer:)' | tail -n8 | tr '\\n' ';' || true)\"\n"
        "  [[ -n $receiver_lines ]] || receiver_lines='no recent WebRTC receiver diagnostic'\n"
        "  printf '  ui.video.receiver=%s\\n' \"$receiver_lines\"\n"
        "}\n"
        "status_sim_media() {\n"
        "  local container=$1 encoder backend display media_line streams frames camera_lines\n"
        "  if ! docker container inspect \"$container\" >/dev/null 2>&1; then\n"
        "    return 0\n"
        "  fi\n"
        "  encoder=\"$(docker logs --tail 1000 \"$container\" 2>&1 | grep -E '\\[sim-media\\] h264 encoder=' | tail -n1 || true)\"\n"
        "  backend=\"$(docker logs --tail 1000 \"$container\" 2>&1 | grep -E '\\[runtime\\] genesis backend requested:' | tail -n1 || true)\"\n"
        "  display=\"$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' \"$container\" 2>/dev/null | sed -n 's/^DISPLAY=/DISPLAY=/p; s/^ELESIM_SIM_VIEWER=/ELESIM_SIM_VIEWER=/p' | tr '\\n' ' ' | sed 's/[[:space:]]*$//' || true)\"\n"
        "  media_line=\"$(docker logs --tail 1000 \"$container\" 2>&1 | grep -E 'h264_nvenc unavailable|falling back to libx264|WebRTC worker unavailable|WebRTC unavailable' | tail -n1 || true)\"\n"
        "  streams=\"$(docker logs --tail 1000 \"$container\" 2>&1 | grep -E '\\[sim-media\\].*(observer|hand_eye_preview)|stream=(observer|hand_eye_preview)' | tail -n2 | tr '\\n' ';' || true)\"\n"
        "  frames=\"$(docker logs --tail 1000 \"$container\" 2>&1 | grep -E '\\[sim-media\\] stream=(observer|hand_eye_preview) frame=(ready|fallback|track-start|dispatched)' | tail -n8 | tr '\\n' ';' || true)\"\n"
        "  camera_lines=\"$(docker logs --tail 1000 \"$container\" 2>&1 | grep -E '\\[sim_camera\\].*(res=|publisher topic=|observer WebRTC source)' | tail -n4 | tr '\\n' ';' || true)\"\n"
        "  [[ -n $encoder ]] || encoder='encoder log not available yet'\n"
        "  [[ -n $backend ]] || backend='Genesis backend log not available yet'\n"
        "  [[ -n $media_line ]] || media_line='none'\n"
        "  [[ -n $streams ]] || streams='observer + hand_eye_preview (configured WebRTC streams; no recent stream log)'\n"
        "  [[ -n $frames ]] || frames='no first-frame/fallback diagnostic yet'\n"
        "  [[ -n $camera_lines ]] || camera_lines='camera details not available yet'\n"
        "  [[ -n $display ]] || display='DISPLAY/ELESIM_SIM_VIEWER not exported'\n"
        "  printf '  sim.video.encoder=%s\\n' \"$encoder\"\n"
        "  printf '  sim.video.backend=%s\\n' \"$backend\"\n"
        "  printf '  sim.video.display=%s\\n' \"$display\"\n"
        "  printf '  sim.video.fallback=%s\\n' \"$media_line\"\n"
        "  printf '  sim.video.streams=%s\\n' \"$streams\"\n"
        "  printf '  sim.video.frames=%s\\n' \"$frames\"\n"
        "  printf '  sim.video.camera=%s\\n' \"$camera_lines\"\n"
        "  printf '  sim.video.transport=WebRTC DTLS/SRTP; DDS carries signaling only\\n'\n"
        "}\n"
        + rendered_calls
        + ("\n" if rendered_calls else "")
        + sim_call
        + "printf '%s\\n' '--- compose view ---'\n"
        + command
        + " ps --all --format 'service={{.Service}} container={{.Name}} state={{.State}} ports={{.Ports}}' 2>/dev/null || true\n"
    )


def render_native_status_wrapper(
    *,
    robot_unit: str,
    bridge_unit: str,
    edition: str = "general-native",
) -> str:
    """Render a read-only status command for a native Robot installation."""

    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "umask 077\n"
        "if (( $# == 1 )) && [[ $1 == --gpu-devices ]]; then\n"
        "  if command -v nvidia-smi >/dev/null 2>&1; then\n"
        "    nvidia-smi --query-gpu=index,uuid --format=csv,noheader,nounits 2>/dev/null || true\n"
        "  fi\n"
        "  exit 0\n"
        "fi\n"
        "host_name=\"$(hostname -f 2>/dev/null || hostname 2>/dev/null || printf unknown)\"\n"
        "host_ips=\"$(hostname -I 2>/dev/null | tr '\\n' ' ' | xargs 2>/dev/null || true)\"\n"
        "[[ -n $host_ips ]] || host_ips=unknown\n"
        "printf 'EleSim status (current-host only)\\n'\n"
        "printf 'edition=%s\\n' "
        + _quoted(edition)
        + "\n"
        "printf 'host=%s\\n' \"$host_name\"\n"
        "printf 'host_ips=%s\\n' \"$host_ips\"\n"
        "printf 'scope=native Robot systemd units on this host\\n'\n"
        "if (( $# != 0 )); then\n"
        "  printf '사용법: elesim-status [--gpu-devices]\\n' >&2\n"
        "  exit 64\n"
        "fi\n"
        "status_unit() {\n"
        "  local label=$1 unit=$2 active sub_state\n"
        "  active=\"$(systemctl is-active \"$unit\" 2>/dev/null || true)\"\n"
        "  sub_state=\"$(systemctl show -p SubState --value \"$unit\" 2>/dev/null || true)\"\n"
        "  [[ -n $active ]] || active=not-found\n"
        "  [[ -n $sub_state ]] || sub_state=unknown\n"
        "  printf '[%s] unit=%s active=%s substate=%s host=%s ips=%s\\n' \"$label\" \"$unit\" \"$active\" \"$sub_state\" \"$host_name\" \"$host_ips\"\n"
        "}\n"
        "status_unit robot "
        + _quoted(robot_unit)
        + "\n"
        "status_unit unitree_bridge "
        + _quoted(bridge_unit)
        + "\n"
    )


__all__ = ["render_compose_status_wrapper", "render_native_status_wrapper"]
