from __future__ import annotations

import http.client
import json
import re
import threading
import time
from pathlib import Path

import pytest

from elesim_setup.connection_gui import (
    ConnectionManagerApplication,
    ConnectionManagerServer,
    connection_web_root,
)
from elesim_setup.connection_manager import (
    ConnectionTopology,
    DdsEndpoint,
    DdsGraphSettings,
    ManagedHost,
    RoleAssignment,
    SshEndpoint,
)


FINGERPRINT = "SHA256:" + "A" * 43


def _ssh(host: str, port: int) -> SshEndpoint:
    return SshEndpoint(
        host=host,
        port=port,
        user="elesim",
        identity_file="~/.ssh/elesim_ed25519",
        pinned_fingerprint=FINGERPRINT,
    )


def _topology() -> ConnectionTopology:
    return ConnectionTopology(
        system_id="lab_arm",
        security_profile="sros2",
        dds_graph=DdsGraphSettings(domain_id=18, discovery_mode="static"),
        hosts=(
            ManagedHost(
                host_id="laptop",
                display_name="Operator laptop",
                local=True,
                dds=DdsEndpoint("100.64.0.10", "tailscale0"),
                ssh=None,
                assignments=(
                    RoleAssignment("pilot", "pilot-main"),
                    RoleAssignment("ui", "ui-main"),
                ),
            ),
            ManagedHost(
                host_id="compute",
                display_name="Compute server",
                local=False,
                dds=DdsEndpoint("100.64.0.20", "tailscale0"),
                ssh=_ssh("compute.example", 2222),
                assignments=(RoleAssignment("sim", "sim-main"),),
            ),
            ManagedHost(
                host_id="robot",
                display_name="Robot Jetson",
                local=False,
                dds=DdsEndpoint("100.64.0.30", "tailscale0"),
                ssh=_ssh("robot.example", 2201),
                assignments=(RoleAssignment("robot", "robot-main"),),
                install_mode="native",
                jetson=True,
                install_root="/opt/elesim-robot",
                bin_dir="/usr/local/bin",
                lifecycle="systemd",
            ),
        ),
    ).validate()


def _simulation_topology() -> ConnectionTopology:
    return ConnectionTopology(
        system_id="lab_sim",
        security_profile="trusted-network",
        topology_mode="simulation-only",
        hosts=(
            ManagedHost(
                host_id="laptop",
                display_name="Simulation laptop",
                local=True,
                dds=DdsEndpoint("100.64.0.40", "tailscale0"),
                ssh=None,
                assignments=(
                    RoleAssignment("pilot", "pilot-main"),
                    RoleAssignment("sim", "sim-main"),
                    RoleAssignment("ui", "ui-main"),
                ),
            ),
        ),
    ).validate()


def _preflight_payload() -> dict[str, object]:
    return {
        "schema_version": 1,
        "discovery_mode": "static",
        "hosts": [
            {
                "id": "laptop",
                "display_name": "Operator laptop",
                "local": True,
                "dds": {"address": "100.64.0.10", "interface": "tailscale0"},
                "ssh": None,
            },
            {
                "id": "compute",
                "display_name": "Compute host",
                "local": False,
                "dds": {"address": "100.64.0.20", "interface": "tailscale0"},
                "ssh": {"host": "100.64.0.20", "port": 22, "user": "elesim"},
            },
        ],
        "probe_ssh": True,
    }


def _application(
    tmp_path: Path,
    *,
    runner=lambda _topology, _action, _log: None,
    probe=None,
    tailscale_probe=None,
) -> ConnectionManagerApplication:
    return ConnectionManagerApplication(
        state_path=tmp_path / "connections.json",
        token="test-session-token",
        runner=runner,
        fingerprint_probe=probe,
        tailscale_fingerprint_probe=tailscale_probe,
    )


def _wait_for_job(app: ConnectionManagerApplication) -> dict[str, object]:
    deadline = time.monotonic() + 2
    while True:
        snapshot = app.job_snapshot()
        if snapshot["status"] not in {"running", "cancelling"}:
            return snapshot
        assert time.monotonic() < deadline
        time.sleep(0.01)


def test_connection_gui_assets_have_bilingual_drag_drop_board() -> None:
    root = connection_web_root()
    catalog = json.loads((root / "i18n.json").read_text(encoding="utf-8"))
    html = (root / "index.html").read_text(encoding="utf-8")
    script = (root / "app.js").read_text(encoding="utf-8")

    assert set(catalog) == {"ko", "en"}
    assert set(catalog["ko"]) == set(catalog["en"])
    assert all(
        (root / name).is_file() for name in ("index.html", "style.css", "app.js")
    )
    assert all(label in html for label in ("COM1", "COM2", "COM3", "Robot"))
    assert 'data-field="unused"' in html
    assert 'id="topology-mode"' in html
    assert "simulation-only" in script
    assert 'id="apply"' in html
    assert 'id="workflow-stage"' in html
    assert 'data-i18n="advanced.title"' not in html
    assert 'id="host-check"' in html
    assert 'id="rotate"' in html
    assert 'id="runtime-stop"' in html
    assert 'id="recover"' not in html
    assert 'id="runtime-restart"' not in html
    assert 'function runApplyJob()' in script
    assert 'apply.textContent = t(sros2 ? "action.provision" : "action.deploy")' in script
    assert '["provision", "deploy", "rotate"].includes(job.action)' in script
    assert 'startJob("check")' in script
    assert 'workflow.stage.ready' in script
    assert 'data-drop-slot="robot"' in html
    assert "dragstart" in script and "dataTransfer" in script
    assert 'roleLocations.robot = "robot"' in script
    assert 'sim: "sim-default"' in script
    assert 'robot: "robot-go2"' in script
    assert "SSH" in catalog["ko"]["boundary.text"]
    assert "런타임" in catalog["ko"]["boundary.text"]
    assert "DDS 도달성" in catalog["ko"]["boundary.text"]
    assert "bidirectional UDP" in catalog["en"]["boundary.text"]
    assert "자동으로 계산" in catalog["ko"]["derived.help"]
    ssh_key_fields = re.findall(
        r'<input\b[^>]*data-field="ssh-key"[^>]*>',
        html,
    )
    assert len(ssh_key_fields) == 4
    assert all("value=" not in field for field in ssh_key_fields)
    ssh_tailscale_fields = re.findall(
        r'<input\b[^>]*data-field="ssh-tailscale"[^>]*>',
        html,
    )
    assert len(ssh_tailscale_fields) == 4
    assert "ssh-tailscale" in script
    assert "빈 칸은 SSH agent" in catalog["ko"]["ssh.help"]
    assert 'data-field="dds-address"' in html
    assert 'placeholder="100.x.y.z"' in html
    assert "Abort" in catalog["ko"]["action.cancel"]


def test_application_validates_and_atomically_saves_mode_0600(tmp_path: Path) -> None:
    app = _application(tmp_path)
    topology = _topology()

    validated = app.validate_topology(topology.to_dict())
    saved = app.save_topology(topology.to_dict())
    context = app.context()

    assert validated["valid"] is True and validated["saved"] is False
    assert saved["saved"] is True and saved["mode"] == "0600"
    assert app.state_path.stat().st_mode & 0o777 == 0o600
    assert context["topology"] == topology.to_dict()
    assert context["manager_transport"]["containerized"] is False
    assert context["manager_transport"]["tailscale_proxy"] is False
    assert context["derived_static_peers"]["compute"] == [
        "100.64.0.10",
        "100.64.0.30",
    ]
    encoded = json.dumps(context)
    assert "BEGIN PRIVATE KEY" not in encoded
    assert "test-session-token" not in encoded

    unsafe = topology.to_dict()
    unsafe["hosts"][1]["ssh"]["private_key"] = "-----BEGIN PRIVATE KEY-----"
    with pytest.raises(ValueError, match="secret material"):
        app.save_topology(unsafe)


def test_context_restores_managed_generation_without_private_material(
    tmp_path: Path,
) -> None:
    authority = tmp_path / "authority" / "lab_arm"
    authority.mkdir(parents=True)
    (authority / "active.json").write_text(
        json.dumps({"generation": "g-20260807t000000000000z-abcdef123456"}),
        encoding="utf-8",
    )
    app = ConnectionManagerApplication(
        state_path=tmp_path / "connections.json",
        token="test-session-token",
        runner=lambda _topology, _action, _log: None,
        authority_root=tmp_path / "authority",
    )
    app.save_topology(_topology().to_dict())

    security = app.context()["security"]
    assert security == {
        "profile": "sros2",
        "managed_generation": "g-20260807t000000000000z-abcdef123456",
    }
    assert "BEGIN PRIVATE KEY" not in json.dumps(security)


def test_application_saves_simulation_only_topology_without_robot(tmp_path: Path) -> None:
    app = _application(tmp_path)
    topology = _simulation_topology()

    response = app.save_topology(topology.to_dict())

    assert response["valid"] is True
    saved = ConnectionTopology.load(tmp_path / "connections.json")
    assert saved.topology_mode == "simulation-only"
    assert {assignment.role for host in saved.hosts for assignment in host.assignments} == {
        "pilot",
        "sim",
        "ui",
    }


def test_fingerprint_probe_uses_explicit_non_default_ssh_port(tmp_path: Path) -> None:
    calls: list[tuple[str, int]] = []

    def probe(host: str, port: int) -> str:
        calls.append((host, port))
        return FINGERPRINT

    app = _application(tmp_path, probe=probe)
    result = app.probe_fingerprint({"host": "compute.example", "port": 2222})

    assert calls == [("compute.example", 2222)]
    assert result == {"fingerprint": FINGERPRINT}
    with pytest.raises(ValueError, match="exactly host and port"):
        app.probe_fingerprint(
            {"host": "compute.example", "port": 2222, "password": "forbidden"}
        )

    invalid = _application(tmp_path, probe=lambda _host, _port: "SHA256:not-valid")
    with pytest.raises(RuntimeError, match="invalid host-key"):
        invalid.probe_fingerprint({"host": "compute.example", "port": 2222})


def test_tailscale_probe_uses_keyless_probe_and_port_22(tmp_path: Path) -> None:
    calls: list[tuple[str, int]] = []

    def probe(host: str, port: int) -> str:
        calls.append((host, port))
        return FINGERPRINT

    app = _application(
        tmp_path,
        probe=lambda _host, _port: pytest.fail("ordinary SSH probe must not run"),
        tailscale_probe=probe,
    )

    assert app.probe_fingerprint(
        {"host": "100.74.222.24", "port": 22, "auth_mode": "tailscale"}
    ) == {"fingerprint": FINGERPRINT}
    assert calls == [("100.74.222.24", 22)]
    with pytest.raises(ValueError, match="port 22"):
        app.probe_fingerprint(
            {"host": "100.74.222.24", "port": 2222, "auth_mode": "tailscale"}
        )


def test_two_host_preflight_is_ephemeral_and_can_probe_ssh(tmp_path: Path) -> None:
    calls: list[tuple[str, int]] = []

    def probe(host: str, port: int) -> str:
        calls.append((host, port))
        return FINGERPRINT

    app = _application(tmp_path, probe=probe)
    result = app.validate_preflight(_preflight_payload())

    assert result["valid"] is True
    assert result["derived_static_peers"] == {
        "laptop": ["100.64.0.20"],
        "compute": ["100.64.0.10"],
    }
    assert calls == [("100.64.0.20", 22)]
    assert result["ssh_checks"]["compute"]["checked"] is True
    assert result["ssh_checks"]["compute"]["fingerprint"] == FINGERPRINT
    assert not (tmp_path / "connections.json").exists()


def test_two_host_preflight_rejects_secrets_and_http_port(tmp_path: Path) -> None:
    app = _application(tmp_path)
    unsafe = _preflight_payload()
    unsafe["token"] = "secret"
    with pytest.raises(ValueError, match="secret material"):
        app.validate_preflight(unsafe)

    invalid = _preflight_payload()
    invalid["hosts"][0]["dds"]["address"] = "100.64.0.10:8080"
    with pytest.raises(ValueError, match="port"):
        app.validate_preflight(invalid)


def test_background_deploy_is_bounded_and_redacts_sensitive_logs(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    calls: list[tuple[ConnectionTopology, str]] = []

    def runner(topology: ConnectionTopology, action: str, log) -> None:
        calls.append((topology, action))
        log("staging compute")
        log("password=must-not-reach-browser")

    app = _application(tmp_path, runner=runner)
    app.save_topology(_topology().to_dict())

    started = app.start_job("deploy")
    finished = _wait_for_job(app)

    assert started["action"] == "deploy"
    assert calls == [(_topology(), "deploy")]
    assert finished["status"] == "completed"
    assert finished["logs"] == [
        "staging compute",
        "password=[redacted]",
    ]
    assert "must-not-reach-browser" not in json.dumps(finished)
    terminal = capsys.readouterr().out
    assert "[connection-manager] staging compute" in terminal
    assert "[connection-manager] password=[redacted]" in terminal
    assert "must-not-reach-browser" not in terminal


def test_security_generation_actions_require_sros2(tmp_path: Path) -> None:
    app = _application(tmp_path)
    raw = _topology().to_dict()
    raw["security_profile"] = "trusted-network"
    app.save_topology(raw)

    with pytest.raises(ValueError, match="requires the sros2"):
        app.start_job("provision")
    with pytest.raises(ValueError, match="requires the sros2"):
        app.start_job("rotate")


def test_background_job_cancels_at_the_next_log_boundary(tmp_path: Path) -> None:
    entered = threading.Event()
    continue_runner = threading.Event()

    def runner(_topology: ConnectionTopology, _action: str, log) -> None:
        log("staged")
        entered.set()
        continue_runner.wait(timeout=2)
        log("switch")

    app = _application(tmp_path, runner=runner)
    app.save_topology(_topology().to_dict())
    app.start_job("rotate")
    assert entered.wait(timeout=1)

    with pytest.raises(RuntimeError, match="배포 중"):
        app.save_topology(_topology().to_dict())
    with pytest.raises(RuntimeError, match="rollback"):
        app.request_shutdown()

    snapshot = app.cancel_job()
    continue_runner.set()
    finished = _wait_for_job(app)

    assert snapshot["status"] == "cancelling"
    assert finished["status"] == "cancelled"
    assert finished["logs"] == ["staged"]


def test_cancel_after_runner_commit_is_reported_as_completed(tmp_path: Path) -> None:
    committed = threading.Event()
    finish_runner = threading.Event()

    def runner(_topology: ConnectionTopology, _action: str, log) -> None:
        log("commit-authority")
        committed.set()
        finish_runner.wait(timeout=2)

    app = _application(tmp_path, runner=runner)
    app.save_topology(_topology().to_dict())
    app.start_job("rotate")
    assert committed.wait(timeout=1)

    snapshot = app.cancel_job()
    finish_runner.set()
    finished = _wait_for_job(app)

    assert snapshot["status"] == "cancelling"
    assert finished["status"] == "completed"
    assert finished["logs"] == ["commit-authority"]


def test_server_rejects_non_loopback_bind(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="loopback"):
        ConnectionManagerServer(("0.0.0.0", 0), _application(tmp_path))


def test_http_boundary_requires_token_and_sets_strict_headers(tmp_path: Path) -> None:
    app = _application(tmp_path)
    app.save_topology(_topology().to_dict())
    server = ConnectionManagerServer(("127.0.0.1", 0), app)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address[:2]
    try:
        connection = http.client.HTTPConnection(host, port, timeout=2)
        connection.request("GET", "/api/context")
        response = connection.getresponse()
        assert response.status == 401
        assert response.getheader("Cache-Control") == "no-store"
        assert response.getheader("Content-Security-Policy") == "default-src 'none'"
        response.read()
        connection.close()

        connection = http.client.HTTPConnection(host, port, timeout=2)
        connection.request(
            "GET",
            "/api/context",
            headers={"X-Elesim-Token": "test-session-token"},
        )
        response = connection.getresponse()
        payload = json.loads(response.read())
        assert response.status == 200
        assert payload["topology_exists"] is True
        assert "test-session-token" not in json.dumps(payload)
        connection.close()

        preflight = _preflight_payload()
        preflight["probe_ssh"] = False
        connection = http.client.HTTPConnection(host, port, timeout=2)
        connection.request(
            "POST",
            "/api/preflight",
            body=json.dumps(preflight),
            headers={
                "X-Elesim-Token": "test-session-token",
                "Content-Type": "application/json",
            },
        )
        response = connection.getresponse()
        payload = json.loads(response.read())
        assert response.status == 200
        assert payload["valid"] is True
        assert payload["ssh_checks"]["compute"]["checked"] is False
        connection.close()

        connection = http.client.HTTPConnection(host, port, timeout=2)
        connection.request("GET", "/app.js")
        response = connection.getresponse()
        assert response.status == 200
        assert "script-src 'self'" in response.getheader("Content-Security-Policy")
        assert response.getheader("Cache-Control") == "no-store"
        response.read()
        connection.close()

        connection = http.client.HTTPConnection(host, port, timeout=2)
        connection.request("GET", "/connection_gui.py")
        response = connection.getresponse()
        assert response.status == 404
        assert response.getheader("Cache-Control") == "no-store"
        response.read()
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_http_boundary_rejects_bodies_over_one_mibibyte(tmp_path: Path) -> None:
    app = _application(tmp_path)
    server = ConnectionManagerServer(("127.0.0.1", 0), app)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address[:2]
    try:
        connection = http.client.HTTPConnection(host, port, timeout=2)
        connection.putrequest("POST", "/api/validate")
        connection.putheader("X-Elesim-Token", "test-session-token")
        connection.putheader("Content-Type", "application/json")
        connection.putheader("Content-Length", str(1_048_577))
        connection.endheaders()
        response = connection.getresponse()
        payload = json.loads(response.read())
        assert response.status == 400
        assert "too large" in payload["error"]
        assert response.getheader("Cache-Control") == "no-store"
        connection.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
