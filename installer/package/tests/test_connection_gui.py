from __future__ import annotations

import http.client
import json
import re
import threading
import time
from dataclasses import replace
from pathlib import Path

import pytest

from elesim_setup.connection_gui import (
    ConnectionManagerApplication,
    ConnectionManagerServer,
    connection_web_root,
    installer_web_font_root,
)
from elesim_setup.connection_manager import (
    ConnectionTopology,
    DdsEndpoint,
    DdsGraphSettings,
    ManagedHost,
    RoleAssignment,
    SshEndpoint,
)
from elesim_setup.connections import _BuildLogForwarder


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
                local=False,
                dds=DdsEndpoint("100.64.0.20", "tailscale0"),
                ssh=_ssh("compute.example", 2222),
                assignments=(RoleAssignment("sim", "sim-main"),),
            ),
            ManagedHost(
                host_id="robot",
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
        "schema_version": 2,
        "discovery_mode": "static",
        "hosts": [
            {
                "id": "laptop",
                "local": True,
                "dds": {"address": "100.64.0.10", "interface": "tailscale0"},
                "ssh": None,
            },
            {
                "id": "compute",
                "local": False,
                "dds": {"address": "100.64.0.20", "interface": "tailscale0"},
                "ssh": {
                    "host": "compute-management.example",
                    "port": 22,
                    "user": "elesim",
                },
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


def test_job_keeps_tailscale_login_url_in_browser_interaction_only(
    tmp_path: Path,
) -> None:
    release = threading.Event()

    def runner(topology: ConnectionTopology, _action: str, log):
        forwarder = _BuildLogForwarder(
            topology.host("compute"), log, phase="network"
        )
        forwarder("stderr", "https://login.tail")
        forwarder("stderr", "scale.com/a/abc_123\n")
        release.wait(timeout=2)
        return topology

    app = _application(tmp_path, runner=runner)
    app.save_topology(_topology().to_dict())
    app.start_job("prepare")
    deadline = time.monotonic() + 2
    while app.job_snapshot()["interaction"] is None:
        assert time.monotonic() < deadline
        time.sleep(0.01)
    running = app.job_snapshot()
    assert running["interaction"] == {
        "kind": "tailscale-login",
        "url": "https://login.tailscale.com/a/abc_123",
        "host": "network compute [stderr]",
    }
    assert not any("login.tailscale.com" in line for line in running["logs"])
    release.set()
    assert _wait_for_job(app)["status"] == "completed"


def test_successful_job_persists_sidecar_discovered_topology(tmp_path: Path) -> None:
    original = _topology()
    updated_host = replace(
        original.host("compute"),
        dds=replace(
            original.host("compute").dds,
            address="100.100.100.42",
            address_source="tailscale",
        ),
    )
    updated = replace(
        original,
        hosts=tuple(
            updated_host if host.host_id == "compute" else host
            for host in original.hosts
        ),
    ).validate()
    app = _application(
        tmp_path,
        runner=lambda _topology, _action, _log: updated,
    )
    app.save_topology(original.to_dict())

    app.start_job("prepare")
    finished = _wait_for_job(app)

    assert finished["status"] == "completed"
    assert finished["topology_updated"] is True
    assert app.load_topology().host("compute").dds.address == "100.100.100.42"


def test_job_does_not_resave_a_runner_persisted_topology_after_commit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = _topology()
    updated_host = replace(
        original.host("compute"),
        dds=replace(original.host("compute").dds, address="100.100.100.43"),
    )
    updated = replace(
        original,
        hosts=tuple(
            updated_host if host.host_id == "compute" else host
            for host in original.hosts
        ),
    ).validate()
    original_save = ConnectionTopology.save
    app = _application(tmp_path)
    app.save_topology(original.to_dict())

    def runner(_topology, _action, _log):
        original_save(updated, app.state_path)
        return updated

    app.runner = runner

    def unexpected_save(_topology, _path):
        raise AssertionError("pre-persisted topology must not be saved after commit")

    monkeypatch.setattr(ConnectionTopology, "save", unexpected_save)
    app.start_job("prepare")
    finished = _wait_for_job(app)

    assert finished["status"] == "completed"
    assert finished["topology_updated"] is True
    assert ConnectionTopology.load(app.state_path) == updated


def test_failed_job_reports_a_runner_persisted_topology_update(tmp_path: Path) -> None:
    original = _topology()
    updated_host = replace(
        original.host("compute"),
        dds=replace(original.host("compute").dds, address="100.100.100.44"),
    )
    updated = replace(
        original,
        hosts=tuple(
            updated_host if host.host_id == "compute" else host
            for host in original.hosts
        ),
    ).validate()
    app = _application(tmp_path)
    app.save_topology(original.to_dict())

    def runner(_topology, _action, _log):
        updated.save(app.state_path)
        raise RuntimeError("run prepare before start")

    app.runner = runner
    app.start_job("start")
    finished = _wait_for_job(app)

    assert finished["status"] == "failed"
    assert finished["topology_updated"] is True
    assert ConnectionTopology.load(app.state_path) == updated


def test_connection_gui_assets_have_bilingual_drag_drop_board() -> None:
    root = connection_web_root()
    catalog = json.loads((root / "i18n.json").read_text(encoding="utf-8"))
    html = (root / "index.html").read_text(encoding="utf-8")
    script = (root / "app.js").read_text(encoding="utf-8")
    style = (root / "style.css").read_text(encoding="utf-8")

    assert set(catalog) == {"ko", "en"}
    assert set(catalog["ko"]) == set(catalog["en"])
    assert all(
        (root / name).is_file() for name in ("index.html", "style.css", "app.js")
    )
    assert '<title data-i18n="app.title">Elesim 연결 관리자</title>' in html
    assert (installer_web_font_root() / "NotoSansCJKkr-Regular.otf").is_file()
    assert 'url("/fonts/NotoSansCJKkr-Regular.otf")' in (
        root / "style.css"
    ).read_text(encoding="utf-8")
    assert html.count("data-banner-close") == 2
    assert "banner.querySelector(\".banner-message\")" in script
    assert "setTimeout(() => { banner.hidden = true;" not in script
    assert all(label in html for label in ("COM1", "COM2", "COM3", "Robot"))
    assert 'data-field="unused"' in html
    assert (
        ".host-card.disabled > .ssh-fields { filter: grayscale(1); opacity: .46; }"
        in style
    )
    assert ".host-card.disabled > .ssh-fields,\n\n.unit-lanes" not in style
    assert (
        ".banner.notice { border: 1px solid #9fc4eb; "
        "background: var(--accent-soft); color: var(--accent-dark); }"
        in style
    )
    assert 'id="topology-mode"' in html
    assert "simulation-only" in script
    assert "ensureRoutedDiscovery" in script
    assert "notice.tailscale.static" in script
    assert 'id="apply"' in html
    assert 'id="restart"' in html
    assert 'class="workflow-actions"' in html
    assert 'id="workflow-stage"' not in html
    assert 'data-i18n="advanced.title"' not in html
    assert 'id="host-check"' not in html
    assert 'id="rotate"' not in html
    assert 'id="runtime-stop"' not in html
    assert 'id="recover"' not in html
    assert 'id="runtime-restart"' not in html
    assert 'class="workflow-layout"' in html
    assert 'data-step="login"' not in html
    assert 'id="tailscale-login"' not in html
    assert html.index('data-step="save"') < html.index('data-step="apply"')
    assert html.index('data-step="apply"') < html.index('data-step="start"')
    assert "grid-template-columns: repeat(3" in style
    assert 'data-state="pending"' in html
    assert 'id="cancel"' in html
    assert 'data-i18n="actions.title"' in html
    assert 'data-i18n="actions.help"' not in html
    assert 'maintenance-actions' not in html
    assert 'workflow.save.help' not in html
    assert 'workflow.apply.help' not in html
    assert 'workflow.start.help' not in html
    assert 'workflow.stage.unsaved' not in script
    assert 'derived-heading' not in html
    assert 'derived-peers' not in html
    assert 'renderDerivedPeers' not in script
    assert not any(key.startswith("derived.") for key in catalog["ko"])
    assert 'function runApplyJob()' in script
    assert 'apply.textContent = t("action.prepare")' in script
    assert '["prepare", "provision", "deploy", "rotate", "restart"].includes(action)' in script
    assert 'restart: !running && workflowSaved && workflowApplied' in script
    assert 'workflow.login.title' not in script and 'workflow.login.title' not in html
    assert 'if (action === "network") return "login";' not in script
    assert 'await startJob("network");' not in script
    assert 'sidecarLoginRequired' not in script
    assert 'byId("tailscale-login").addEventListener' not in script
    assert '["prepare", "provision", "deploy", "rotate"].includes(job.action)' in script
    assert 'byId("runtime-start").addEventListener("click", () => startJob("start").catch(showError))' in script
    assert 'byId("restart").addEventListener("click", () => startJob("restart").catch(showError))' in script
    assert 'startJob("check")' not in script
    assert 'workflow.stage.' not in script
    assert 'data-drop-slot="com4"' in html
    assert 'data-slot="robot"' not in html
    assert 'data-drop-unit="runtime"' in html
    assert 'data-drop-unit="robot"' in html
    assert "dragstart" in script and "dataTransfer" in script
    assert "let roleOrder = [...applicationRoles];" in script
    assert "function insertRoleInOrder" in script
    assert "roleOrder.splice(insertionIndex, 0, role);" in script
    assert "const dropBandRatio = 0.5;" in script
    assert "function dropPlacement(zone, pointerY, draggedRole = \"\")" in script
    assert "previous.bottom - previous.height * dropBandRatio" in script
    assert "next.rect.top + next.rect.height * dropBandRatio" in script
    assert "function dropChangesOrder(zone, draggedRole, placement)" in script
    assert "const previewPlacement = allowed && placement && dropChangesOrder" in script
    assert "function updateDropPreview(zone, placement, draggedRole)" in script
    assert "block.classList.add(\"drop-shift\")" in script
    assert ".role-block.drop-shift { transform: translateY(12px); }" in style
    assert "targetRole," in script
    assert 'roleLocations.robot = "robot"' in script
    assert 'sim: "sim-default"' in script
    assert 'robot: "robot-go2"' in script
    assert catalog["ko"]["actions.title"] == "연결 확인"
    assert catalog["en"]["actions.title"] == "Check Connections"
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
    assert "ssh.help" not in catalog["ko"]
    assert catalog["en"]["ssh.title"] == "Private key authentication via SSH"
    ssh_host_fields = re.findall(
        r'<input\b[^>]*data-field="ssh-host"[^>]*>',
        html,
    )
    assert len(ssh_host_fields) == 4
    assert all("readonly" not in value and "disabled" not in value for value in ssh_host_fields)
    assert html.count('<details class="ssh-fields" open>') == 4
    assert "coturn-fields" not in html
    assert "updateCoturnVisibility" not in script
    assert "updateCoturnSecurity" not in script
    assert "host.coturn" not in script
    assert "robot-install-root" not in html
    assert "robot-bin-dir" not in html
    assert 'host.ssh.host' in script
    assert "let schemaVersion = 4;" in script
    assert 'host: field(slot, "ssh-host").value.trim()' in script
    assert 'const host = field(slot, "ssh-host").value.trim();' in script
    assert "syncSshAddress" not in script
    assert 'container_network_mode === "tailscale-sidecar"' in script
    assert "const topologyAppliedByThisJob =" in script
    assert "if (!topologyAppliedByThisJob)" in script
    assert 'data-field="dds-address"' in html
    assert 'placeholder="100.x.y.z"' in html
    assert catalog["ko"]["action.cancel"] == "중단"
    assert catalog["ko"]["action.prepare"] == "실행 준비"
    assert catalog["ko"]["action.restart"] == "재시작"
    assert catalog["en"]["action.prepare"] == "Prepare runtime"
    assert catalog["en"]["action.restart"] == "Restart"


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
    assert context["manager_transport"]["container_network_mode"] == "direct-host"
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


def test_context_exposes_the_selected_container_network_mode(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ELESIM_CONTAINER_NETWORK_MODE", "tailscale-sidecar")

    context = _application(tmp_path).context()

    assert (
        context["manager_transport"]["container_network_mode"]
        == "tailscale-sidecar"
    )


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
    with pytest.raises(RuntimeError, match="invalid host key"):
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
    assert result["ssh_checks"]["compute"]["host"] == "compute-management.example"
    assert calls == [("compute-management.example", 22)]
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


def test_background_prepare_is_a_first_class_job_action(tmp_path: Path) -> None:
    calls: list[str] = []

    def runner(_topology: ConnectionTopology, action: str, log) -> None:
        calls.append(action)
        log("security prepared")

    app = _application(tmp_path, runner=runner)
    app.save_topology(_topology().to_dict())

    started = app.start_job("prepare")
    finished = _wait_for_job(app)

    assert started["action"] == "prepare"
    assert calls == ["prepare"]
    assert finished["status"] == "completed"


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
        connection.request("GET", "/fonts/NotoSansCJKkr-Regular.otf")
        response = connection.getresponse()
        assert response.status == 200
        assert response.getheader("Content-Type") == "font/otf"
        assert "font-src 'self'" in response.getheader("Content-Security-Policy")
        assert len(response.read()) > 1_000_000
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
