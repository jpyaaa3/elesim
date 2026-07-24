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

    assert (root / "index.html").is_file()
    assert (root / "app.js").is_file()
    assert (root / "style.css").is_file()
    assert (root / "fonts/NotoSansCJKkr-Regular.otf").is_file()
    assert set(catalog) == {"ko", "en"}
    assert set(catalog["ko"]) == set(catalog["en"])
    assert "mode.developer" in catalog["ko"]

    script = (root / "app.js").read_text(encoding="utf-8")
    assert 'byId("dds-domain-id").value = context.defaults.dds_domain_id;' in script
    assert '"dds-security-profile"' in script
    assert 'const roleOrder = ["simulator", "controller", "ui", "robot"];' in script
    assert '"turn-credential-file"' in script
    assert "router-host" not in script


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
    assert context["repository"] == "owner/repo"
    assert context["ref"] == "feature"
    assert context["capabilities"]["robot_installable"] is False


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
        "roles": ["simulator"],
        "prefix": str(tmp_path / "install"),
        "bin_dir": str(tmp_path / "install/bin"),
        "source_root": "/attacker/source",
        "gpu_mode": "cpu",
        "dds_domain_id": 3,
        "dds_security_profile": "trusted-network",
        "turn_mode": "none",
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
        "roles": ["simulator"],
        "prefix": str(tmp_path / "install"),
        "bin_dir": str(tmp_path / "install/bin"),
        "gpu_mode": "cpu",
        "dds_domain_id": 3,
        "dds_security_profile": "trusted-network",
        "turn_mode": "none",
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
        "roles": ["controller"],
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


def test_gui_external_turn_credential_is_simulator_only_and_path_contained(
    tmp_path: Path,
) -> None:
    home = tmp_path / "home"
    home.mkdir()
    source = home / "source"
    source.mkdir()
    credentials = home / "turn.credentials.json"
    credentials.write_text(
        '{"username":"lab-user","credential":"lab-password"}\n',
        encoding="utf-8",
    )
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
        "roles": ["simulator"],
        "prefix": str(home / "install"),
        "bin_dir": str(home / "install/bin"),
        "gpu_mode": "cpu",
        "dds_security_profile": "trusted-network",
        "turn_mode": "external",
        "turn_url": "turn:relay.example.com:3478?transport=udp",
        "turn_credential_file": str(credentials),
    }

    request = app.build_request(payload)
    assert request.turn.credential_path == credentials.resolve()

    outside = tmp_path / "outside.credentials.json"
    outside.write_text(
        '{"username":"other","credential":"secret"}\n',
        encoding="utf-8",
    )
    payload["turn_credential_file"] = str(outside)
    with pytest.raises(PermissionError):
        app.build_request(payload)
