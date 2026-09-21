#!/usr/bin/env python3
"""Run the real EleSim connection-manager page with safe preview backends.

This serves the production connection-manager HTML/CSS/JavaScript and supplies
an in-memory topology, fake installation catalogs, fake host-key probes, and a
side-effect-free job runner.  It never opens Docker, SSH, Tailscale, or a real
installation path.
"""

from __future__ import annotations

import argparse
import shutil
import tempfile
import time
import webbrowser
from pathlib import Path
from typing import Any, Mapping

ROOT = Path(__file__).resolve().parents[2]
import sys

sys.path[:0] = [
    str(ROOT / "payload/runtime/common/protocol"),
    str(ROOT / "payload/runtime/docker/setup/app"),
]

from elesim_connections.connection_gui import (  # noqa: E402
    ConnectionManagerApplication,
    ConnectionManagerServer,
)
from elesim_connections.connection_manager import (  # noqa: E402
    ConnectionTopology,
    DdsEndpoint,
    DdsGraphSettings,
    DeploymentUnit,
    ManagedHost,
    RoleAssignment,
    SshEndpoint,
)


FINGERPRINT = "SHA256:" + "P" * 43
INSTALL_UUID = "4b0bd96f-24d4-4ab7-9711-c865fe4e2896"
PROJECT = "elesim-quick_zebra"
LOCAL_RELEASE = "a" * 64
REMOTE_RELEASE = "b" * 64


def _topology() -> ConnectionTopology:
    return ConnectionTopology(
        system_id="preview_lab",
        security_profile="sros2",
        dds_graph=DdsGraphSettings(
            domain_id=18,
            discovery_mode="static",
        ),
        hosts=(
            ManagedHost(
                host_id="com1",
                local=True,
                dds=DdsEndpoint("100.101.125.114", "tailscale0", "tailscale"),
                ssh=None,
                units=(
                    DeploymentUnit(
                        unit_id="runtime",
                        assignments=(
                            RoleAssignment("pilot", "pilot-1", LOCAL_RELEASE),
                            RoleAssignment("ui", "ui-1", LOCAL_RELEASE),
                        ),
                        install_root="/home/user/ws/newsim",
                        bin_dir="/home/user/ws/newsim/bin",
                        install_uuid=INSTALL_UUID,
                        project=PROJECT,
                        release_key=LOCAL_RELEASE,
                    ),
                ),
            ),
            ManagedHost(
                host_id="com2",
                local=False,
                dds=DdsEndpoint("100.74.222.24", "tailscale0", "tailscale"),
                ssh=SshEndpoint(
                    host="100.74.222.24",
                    port=22,
                    user="hckang",
                    identity_file="",
                    pinned_fingerprint=FINGERPRINT,
                    auth_mode="tailscale",
                ),
                units=(
                    DeploymentUnit(
                        unit_id="runtime",
                        assignments=(RoleAssignment("sim", "sim-1", REMOTE_RELEASE),),
                        install_root="/home/hckang/ws/newsim",
                        bin_dir="/home/hckang/ws/newsim/bin",
                        install_uuid=INSTALL_UUID,
                        project=PROJECT,
                        release_key=REMOTE_RELEASE,
                    ),
                ),
            ),
        ),
    ).validate()


class PreviewApplication(ConnectionManagerApplication):
    """Connection-manager application with every external boundary replaced."""

    def installation_choices(self, payload: Mapping[str, Any]) -> dict[str, object]:
        if not isinstance(payload, Mapping):
            raise ValueError("preview lookup payload must be an object")
        return {
            "installations": [
                {
                    "name": "quick_zebra",
                    "install_uuid": INSTALL_UUID,
                    "project": PROJECT,
                    "releases": [
                        {
                            "key": LOCAL_RELEASE,
                            "label": "1. pilot: bright_walrus · ui: quiet_lark",
                            "role_labels": {
                                "pilot": "pilot: bright_walrus",
                                "ui": "ui: quiet_lark",
                                "sim": "sim: silver_fox",
                            },
                            "roles": ["pilot", "sim", "ui"],
                        },
                        {
                            "key": REMOTE_RELEASE,
                            "label": "2. sim: patient_badger",
                            "role_labels": {"sim": "sim: patient_badger"},
                            "roles": ["sim"],
                        },
                    ],
                }
            ]
        }

    def probe_fingerprint(self, _payload: Mapping[str, Any]) -> dict[str, object]:
        return {"fingerprint": FINGERPRINT}


def _runner(topology: ConnectionTopology, action: str, log) -> ConnectionTopology:
    phases = {
        "prepare": (
            "Preparing runtime network infrastructure on each host.",
            "network: com1",
            "network: com2",
            "register: com1/runtime",
            "register: com2/runtime",
            "All scoped instance registrations completed atomically.",
        ),
        "start": (
            "Preparing runtime network infrastructure on each host.",
            "Prechecking runtime networks on all hosts.",
            "preflight: com1",
            "preflight: com2",
            "Preparing images on all hosts first.",
            "Starting runtimes for active roles.",
            "start: com1",
            "start: com2",
            "DDS readiness: ready (preview)",
        ),
        "check": ("Checking saved topology and host reachability (preview).",),
        "stop": ("Stopping selected runtime units (preview).",),
        "provision": ("Generating managed SROS2 authority (preview).",),
        "deploy": ("Deploying role-scoped security bundles (preview).",),
        "rotate": ("Rotating managed SROS2 generation (preview).",),
        "recover": ("Recovering the last scoped transaction (preview).",),
    }.get(action, (f"Running {action} (preview).",))
    for message in phases:
        log(message)
        time.sleep(0.12)
    return topology


def _status(topology: ConnectionTopology) -> dict[str, object]:
    return {
        "available": True,
        "hosts": [
            {
                "host_id": host.host_id,
                "reachable": True,
                "inventory_ready": True,
                "state": "ready",
                "roles": list(host.roles),
                "detail": "preview host; no remote command was run",
                "gpu_policy": {
                    role: {"mode": "inherit", "device": ""}
                    for role in ("pilot", "sim")
                    if role in host.roles
                },
                "gpu_devices": (
                    [
                        {"index": "0", "name": "Preview GPU", "uuid": "GPU-PREVIEW-0"},
                        {"index": "1", "name": "Preview GPU 1", "uuid": "GPU-PREVIEW-1"},
                    ]
                    if any(role in host.roles for role in ("pilot", "sim"))
                    else []
                ),
            }
            for host in topology.hosts
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0, help="0 selects a free loopback port")
    parser.add_argument("--token", default="connection-preview")
    parser.add_argument("--no-open", action="store_true", help="do not ask the desktop browser to open")
    args = parser.parse_args(argv)

    state_root = Path(tempfile.mkdtemp(prefix="elesim-connection-preview-"))
    state_path = state_root / "topology.json"
    topology = _topology()
    topology.save(state_path)

    application = PreviewApplication(
        state_path=state_path,
        token=args.token,
        runner=_runner,
        status_provider=_status,
        local_install_root=Path("/home/user/ws/newsim"),
        local_bin_dir=Path("/home/user/ws/newsim/bin"),
        gpu_mode="inherit",
    )
    server = ConnectionManagerServer((args.host, args.port), application)
    actual_host, actual_port = server.server_address[:2]
    url = f"http://127.0.0.1:{actual_port}/?token={args.token}"
    print(f"[connection-preview] {url}", flush=True)
    print("[connection-preview] Production connection-manager assets; all backends are fake.", flush=True)
    if not args.no_open:
        try:
            webbrowser.open(url, new=2)
        except Exception as exc:  # pragma: no cover - desktop dependent
            print(f"[connection-preview] browser open skipped: {exc}", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        return 130
    finally:
        application.cancel_and_wait()
        server.server_close()
        shutil.rmtree(state_root, ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
