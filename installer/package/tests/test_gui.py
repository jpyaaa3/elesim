from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import pytest

from elesim_setup.capabilities import HostCapabilities
from elesim_setup.gui import WizardApplication, web_root


def _capabilities() -> HostCapabilities:
    return HostCapabilities(
        architecture="x86_64",
        os_id="ubuntu",
        os_version="22.04",
        jetson=False,
        robot_installable=False,
        developer_installable=True,
        display_available=True,
        ssh_agent=False,
        gpu_devices=(),
    )


def test_gui_assets_and_korean_english_catalog_are_packaged() -> None:
    root = web_root()
    catalog = json.loads((root / "i18n.json").read_text(encoding="utf-8"))
    html = (root / "index.html").read_text(encoding="utf-8")
    script = (root / "app.js").read_text(encoding="utf-8")
    style = (root / "style.css").read_text(encoding="utf-8")

    assert (root / "index.html").is_file()
    assert (root / "app.js").is_file()
    assert (root / "style.css").is_file()
    assert (root / "fonts/NotoSansCJKkr-Regular.otf").is_file()
    assert set(catalog) == {"ko", "en"}
    assert set(catalog["ko"]) == set(catalog["en"])
    assert "mode.developer" in catalog["ko"]
    assert catalog["ko"]["mode.general"] == "일반 사용자"
    assert catalog["ko"]["mode.developer"] == "개발자"
    for section in ("mode", "roles", "paths", "compute", "review", "install"):
        assert catalog["ko"][f"step.{section}"] == catalog["ko"][f"{section}.title"]
        assert catalog["en"][f"step.{section}"] == catalog["en"][f"{section}.title"]
    assert catalog["en"]["mode.developer.help"] == (
        "Create a complete container with the source and SDKs."
    )
    assert catalog["ko"]["mode.developer.help"] == (
        "전체 소스와 SDK를 포함한 컨테이너를 만듭니다."
    )
    assert "mode.developer.privileged" not in catalog["ko"]
    assert catalog["ko"]["app.title"] == "Elesim 설치 마법사"
    assert catalog["en"]["app.title"] == "Elesim Install Wizard"
    assert 'data-i18n="app.title"' in html
    assert not any(key.startswith("uninstall.") for key in catalog["ko"])
    assert not any(
        key.startswith(("network.", "dds.", "ssh.", "turn."))
        for key in catalog["ko"]
    )

    assert 'const steps = ["mode", "roles", "paths", "compute", "review", "install"];' in script
    assert '"dds-security-profile"' not in script
    assert '"dds-security-provisioning"' not in script
    assert 'const roleOrder = ["sim", "pilot", "ui", "robot"];' in script
    assert 'data-step="network"' not in html
    assert 'data-step-link="network"' not in html
    assert 'data-i18n="network.manager.help"' not in html
    assert 'id="post-install-command"' in html
    assert "source ~/.bashrc" in script
    assert 'id="register-path" type="checkbox" checked' in html
    assert ".command-row code" in style and "white-space: pre-wrap;" in style
    assert "elesim-connections</code>" in html
    assert '`${binDir}/elesim-connections && source ~/.bashrc && ${managerCleanup}`' not in script
    assert '`cd ${shellQuote(binDir)} && source ~/.bashrc && ${managerCleanup}`' in script
    assert "const shellQuote =" in script
    assert "const pendingManaged" not in script
    assert 'const defaultGeneralRoles = ["sim", "pilot", "ui"];' in script
    assert "data-preset" not in script
    assert "applyPreset" not in script
    assert "preset-bar" not in (root / "index.html").read_text(encoding="utf-8")
    assert not any(key.startswith("roles.preset.") for key in catalog["ko"])
    assert '"turn-credential-file"' not in script
    assert 'name="turn-mode"' not in html
    assert 'data-i18n="turn.managed.help"' not in html
    assert 'data-i18n="turn.external"' not in html
    assert 'id="turn-section"' not in html
    assert 'runtime_text_logs: {' in script
    assert 'byId("runtime-text-logs").checked' in script
    assert "router-host" not in script
    assert 'id="privileged-confirm-row"' not in html
    assert 'id="open-uninstall"' not in html
    assert 'id="uninstall-dialog"' not in html
    assert 'id="jaeger-row"' in html
    assert 'data-i18n="mode.jaeger.help"' not in html
    assert "/api/uninstall/guide" not in script
    assert 'byId("close-installer").hidden = !atInstall;' in script
    assert 'id="close-installer"' in html and 'data-i18n="action.close" hidden disabled' in html
    assert "grid-template-columns: 120px minmax(0, 1fr) 120px;" in style
    assert ".navigation > #next-button,\n.navigation > #close-installer" in style
    assert ".choice:has(> input:checked)," in style
    assert ".choice:has(> .choice-radio > input:checked)" in style


def test_context_defaults_to_original_invocation_directory(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    invocation = tmp_path / "install-here"
    invocation.mkdir()
    app = WizardApplication(
        source_root=source,
        invocation_dir=invocation,
        capabilities=_capabilities(),
        repository="owner/repo",
        ref="feature",
        token="test-token",
        runner=lambda _request, _log: None,
    )

    context = app.context()

    assert context["defaults"]["prefix"] == str(invocation)
    assert context["defaults"]["bin_dir"] == str(invocation / "bin")
    assert context["defaults"]["dds_security_profile"] == "sros2"
    assert context["defaults"]["dds_security_provisioning"] == "managed"
    assert context["repository"] == "owner/repo"
    assert context["ref"] == "feature"
    assert context["capabilities"]["robot_installable"] is False


def test_gui_validation_reports_runtime_text_log_choice(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    app = WizardApplication(
        source_root=source,
        invocation_dir=tmp_path,
        capabilities=_capabilities(),
        repository="owner/repo",
        ref="main",
        token="test-token",
        runner=lambda _request, _log: None,
    )
    payload = {
        "language": "ko",
        "edition": "general",
        "roles": ["sim"],
        "prefix": str(tmp_path / "install"),
        "bin_dir": str(tmp_path / "install/bin"),
        "gpu_mode": "cpu",
        "dds_security_profile": "sros2",
        "dds_security_provisioning": "managed",
        "turn_mode": "managed",
        "turn_url": "turn:203.0.113.10:3478?transport=udp",
        "turn_realm": "elesim.local",
        "turn_public_host": "203.0.113.10",
        "runtime_text_logs": {"enabled": False},
    }

    assert app.validate_request(payload)["runtime_text_logs"] is False


def test_gui_allows_trusted_sim_without_coturn(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    app = WizardApplication(
        source_root=source,
        invocation_dir=tmp_path,
        capabilities=_capabilities(),
        repository="owner/repo",
        ref="main",
        token="test-token",
        runner=lambda _request, _log: None,
    )
    payload = {
        "language": "ko",
        "edition": "general",
        "roles": ["sim"],
        "prefix": str(tmp_path / "install"),
        "bin_dir": str(tmp_path / "install/bin"),
        "gpu_mode": "cpu",
        "dds_security_profile": "trusted-network",
        "turn_mode": "none",
    }

    summary = app.validate_request(payload)

    assert summary["security_profile"] == "trusted-network"
    assert summary["turn_mode"] == "none"


def test_directory_browser_cannot_escape_mounted_roots(tmp_path: Path) -> None:
    home = tmp_path / "home"
    home.mkdir()
    (home / "a").mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    app = WizardApplication(
        source_root=home,
        invocation_dir=home,
        capabilities=_capabilities(),
        repository="owner/repo",
        ref="main",
        token="test-token",
        allowed_roots=(home,),
        runner=lambda _request, _log: None,
    )

    listing = app.list_directories(home)
    assert listing["directories"][0]["name"] == "a"
    with pytest.raises(PermissionError):
        app.list_directories(outside)


def test_gui_request_cannot_replace_bootstrap_source_root(tmp_path: Path) -> None:
    source = tmp_path / "trusted-source"
    source.mkdir()
    app = WizardApplication(
        source_root=source,
        invocation_dir=tmp_path,
        capabilities=_capabilities(),
        repository="owner/repo",
        ref="main",
        token="test-token",
        runner=lambda _request, _log: None,
    )
    payload = {
        "language": "ko",
        "edition": "general",
        "roles": ["sim"],
        "prefix": str(tmp_path / "install"),
        "bin_dir": str(tmp_path / "install/bin"),
        "source_root": "/attacker/source",
        "gpu_mode": "cpu",
        "dds_domain_id": 3,
        "dds_security_profile": "sros2",
        "dds_security_provisioning": "managed",
        "turn_mode": "managed",
        "turn_url": "turn:203.0.113.10:3478?transport=udp",
        "turn_realm": "elesim.local",
        "turn_public_host": "203.0.113.10",
    }

    request = app.build_request(payload)

    assert request.source_root == source.resolve()


def test_running_install_can_be_cooperatively_cancelled(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    runner_started = threading.Event()
    continue_runner = threading.Event()

    def runner(_request, log) -> None:
        runner_started.set()
        continue_runner.wait(timeout=2)
        log("checkpoint")

    app = WizardApplication(
        source_root=source,
        invocation_dir=tmp_path,
        capabilities=_capabilities(),
        repository="owner/repo",
        ref="main",
        token="test-token",
        runner=runner,
    )
    payload = {
        "language": "ko",
        "edition": "general",
        "roles": ["sim"],
        "prefix": str(tmp_path / "install"),
        "bin_dir": str(tmp_path / "install/bin"),
        "gpu_mode": "cpu",
        "dds_domain_id": 3,
        "dds_security_profile": "sros2",
        "dds_security_provisioning": "managed",
        "turn_mode": "managed",
        "turn_url": "turn:203.0.113.10:3478?transport=udp",
        "turn_realm": "elesim.local",
        "turn_public_host": "203.0.113.10",
    }

    app.start_install(payload)
    assert runner_started.wait(timeout=1)
    snapshot = app.cancel_install()
    assert snapshot["status"] == "cancelling"
    continue_runner.set()
    deadline = time.monotonic() + 2
    while app.job_snapshot()["status"] not in {"cancelled", "failed"}:
        assert time.monotonic() < deadline
        time.sleep(0.01)

    assert app.job_snapshot()["status"] == "cancelled"


def test_gui_rejects_credential_and_identity_paths_outside_mounted_roots(
    tmp_path: Path,
) -> None:
    home = tmp_path / "home"
    home.mkdir()
    source = home / "source"
    source.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    identity = outside / "id_ed25519"
    identity.write_text("private", encoding="utf-8")
    app = WizardApplication(
        source_root=source,
        invocation_dir=home,
        capabilities=_capabilities(),
        repository="owner/repo",
        ref="main",
        token="test-token",
        allowed_roots=(home,),
        runner=lambda _request, _log: None,
    )
    payload = {
        "language": "ko",
        "edition": "general",
        "roles": ["pilot"],
        "prefix": str(home / "install"),
        "bin_dir": str(home / "install/bin"),
        "gpu_mode": "cpu",
        "dds_security_profile": "sros2",
        "dds_keystore": str(outside / "sros2"),
        "dds_enclave": "/elesim",
        "turn_mode": "none",
        "ssh": {
            "host": "sim.example.com",
            "port": 2222,
            "user": "operator",
            "remote_root": "/srv/elesim/secrets",
            "identity_file": str(identity),
            "accepted_fingerprint": "SHA256:test",
        },
    }

    with pytest.raises(PermissionError):
        app.build_request(payload)


def test_gui_rejects_external_turn_relay_selection(
    tmp_path: Path,
) -> None:
    home = tmp_path / "home"
    home.mkdir()
    source = home / "source"
    source.mkdir()
    app = WizardApplication(
        source_root=source,
        invocation_dir=home,
        capabilities=_capabilities(),
        repository="owner/repo",
        ref="main",
        token="test-token",
        allowed_roots=(home,),
        runner=lambda _request, _log: None,
    )
    payload = {
        "language": "ko",
        "edition": "general",
        "roles": ["sim"],
        "prefix": str(home / "install"),
        "bin_dir": str(home / "install/bin"),
        "gpu_mode": "cpu",
        "dds_security_profile": "sros2",
        "dds_security_provisioning": "managed",
        "turn_mode": "external",
        "turn_url": "turn:relay.example.com:3478?transport=udp",
    }

    with pytest.raises(ValueError, match="managed TURN"):
        app.build_request(payload)
