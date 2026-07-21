from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

from conftest import ROOT
from elesim_setup.doctor import NetworkDoctor, PASS
from elesim_setup.state import InstallState, NetworkSettings, SecuritySettings
from misc.infra.bootstrap_security import generate


def _free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def _wait_for_port(port: int) -> None:
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                return
        except OSError:
            time.sleep(0.05)
    raise TimeoutError(f"router port {port} did not open")


def _start_router(config: Path) -> subprocess.Popen:
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(ROOT / "packages/protocol/src"), str(ROOT / "router/src"))
    )
    return subprocess.Popen(
        (sys.executable, "-m", "elesim_router.main", "--config", str(config)),
        cwd=ROOT,
        env=environment,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


@pytest.mark.parametrize("secure", [False, True])
def test_doctor_registers_and_discovers_against_real_router(tmp_path: Path, secure: bool) -> None:
    port = _free_port()
    credentials = tmp_path / "secrets"
    security = {
        "curve_server_secret_file": "",
        "curve_public_keys_dir": "",
        "endpoint_registry_file": "",
        "allow_insecure_remote": False,
    }
    state_security = SecuritySettings()
    if secure:
        generate(
            credentials,
            tmp_path / "coturn.env",
            turn_public_ip="127.0.0.1",
            turn_realm="test.local",
            force=False,
        )
        security.update(
            {
                "curve_server_secret_file": str(credentials / "curve/router/router.key_secret"),
                "curve_public_keys_dir": str(credentials / "curve/authorized"),
                "endpoint_registry_file": str(credentials / "curve/endpoints.yaml"),
            }
        )
        state_security = SecuritySettings(mode="curve", credentials_root=str(credentials))
    config = tmp_path / "router.yaml"
    config.write_text(
        yaml.safe_dump(
            {
                "schema_version": 2,
                "router": {
                    "bind_endpoint": f"tcp://127.0.0.1:{port}",
                    "heartbeat_timeout_s": 3.5,
                },
                "security": security,
                "turn": {
                    "urls": [],
                    "static_auth_secret_file": "",
                    "credential_ttl_s": 3600,
                    "refresh_before_s": 600,
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    state = InstallState(
        profile="custom",
        roles=("ui",),
        prefix=str(tmp_path / "install"),
        bin_dir=str(tmp_path / "bin"),
        source_root=str(ROOT),
        network=NetworkSettings(router_port=port),
        security=state_security,
    )

    process = _start_router(config)
    try:
        _wait_for_port(port)
        report = NetworkDoctor(state, timeout_s=2.0).run()
    finally:
        process.terminate()
        process.wait(timeout=5.0)

    statuses = {result.name: result.status for result in report.results}
    assert statuses["Router TCP"] == PASS
    assert statuses["ZMQ protocol"] == PASS
    assert report.ok
