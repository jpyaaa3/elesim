#!/usr/bin/env python3
"""Serve a safe browser preview of the EleSim setup wizard.

The preview uses the production wizard assets and HTTP handlers, but replaces
the installer with a short, cancellable log simulation.  It never invokes
Docker, writes an installation, probes a real SSH host, or removes anything.
"""

from __future__ import annotations

import argparse
import sys
import tempfile
import time
from pathlib import Path
from typing import Callable


ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(ROOT / "payload/runtime/common/protocol"),
    str(ROOT / "payload/runtime/docker/tools/app"),
]

from elesim_setup import gui as wizard_gui  # noqa: E402
from elesim_setup.capabilities import GpuDevice, HostCapabilities  # noqa: E402
from elesim_setup.gui import WizardApplication, WizardServer  # noqa: E402
from elesim_setup.request import SetupRequest  # noqa: E402


PREVIEW_FINGERPRINT = "SHA256:elesim-setup-preview"


def _preview_fingerprint(_host: str, _port: int) -> str:
    return PREVIEW_FINGERPRINT


def _preview_capabilities() -> HostCapabilities:
    return HostCapabilities(
        architecture="x86_64",
        os_id="ubuntu",
        os_version="22.04",
        jetson=False,
        robot_installable=False,
        developer_installable=True,
        display_available=True,
        ssh_agent=True,
        gpu_devices=(
            GpuDevice(
                index="0",
                name="Preview GPU",
                uuid="GPU-PREVIEW-0",
            ),
            GpuDevice(
                index="1",
                name="Preview GPU 1",
                uuid="GPU-PREVIEW-1",
            ),
        ),
        wsl=True,
        wslg_available=True,
    )


def _preview_runner(
    request: SetupRequest,
    log: Callable[[str], None],
) -> None:
    """Emit realistic installer phases without changing the host."""

    roles = ", ".join(request.roles) if request.roles else "development workspace"
    phases = (
        f"Previewing {request.edition} installation",
        f"Selected roles: {roles}",
        "Checking host capabilities",
        "Validating paths and runtime settings",
        "Preparing generated configuration",
        "Writing command wrappers (simulated)",
        "Registering shell integration (simulated)",
        "Preview complete: no files, packages, images, or services were changed",
    )
    for index, phase in enumerate(phases, 1):
        log(f"[{index}/{len(phases)}] {phase}")
        time.sleep(0.28)


class PreviewApplication(WizardApplication):
    """Wizard application with only side-effect-free preview operations."""


def _application(token: str) -> tuple[PreviewApplication, Path]:
    preview_root = Path(tempfile.mkdtemp(prefix="elesim-setup-preview-"))
    application = PreviewApplication(
        source_root=ROOT,
        invocation_dir=preview_root,
        capabilities=_preview_capabilities(),
        repository="preview/elesim",
        ref="preview",
        token=token,
        runner=_preview_runner,
        allowed_roots=(ROOT, Path.home(), preview_root, Path("/tmp")),
    )
    return application, preview_root


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8791)
    parser.add_argument("--token", default="preview-token")
    args = parser.parse_args(argv)

    # The production handler imports this function once at module load time.
    # Replace it only in this short-lived preview process so no real SSH probe
    # can happen accidentally.
    wizard_gui.probe_ssh_fingerprint = _preview_fingerprint
    application, preview_root = _application(args.token)
    server = WizardServer((args.host, args.port), application)
    actual_host, actual_port = server.server_address[:2]
    display_host = "127.0.0.1" if actual_host in {"0.0.0.0", "::"} else actual_host
    print(
        f"[setup-preview] http://{display_host}:{actual_port}/?token={args.token}",
        flush=True,
    )
    print(f"[setup-preview] temporary prefix: {preview_root}", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        return 130
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
