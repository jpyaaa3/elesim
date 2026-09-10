from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import pytest

from elesim_setup.instance_security import (
    SecurityAuthorityError,
    activate_instance_security,
    active_instance_security_path,
    stage_instance_security,
)
import elesim_setup.instance_security as security_module
from elesim_setup.instances import InstanceEndpoint, InstanceState


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"


def _instance(system: str, *, duplicate: bool = False) -> InstanceState:
    endpoints = [
        InstanceEndpoint("pilot", f"{system}-pilot"),
        InstanceEndpoint("sim", f"{system}-sim"),
    ]
    if duplicate:
        endpoints.append(InstanceEndpoint("pilot", f"{system}-pilot-2"))
    return InstanceState(
        system,
        "a" * 64,
        tuple(endpoints),
        20 if system == "alpha" else 21,
    )


def _view(root: Path, system: str, role: str, endpoint: str, marker: str) -> Path:
    view = root / role / "keystore"
    (view / "public").mkdir(parents=True)
    (view / "public" / "identity_ca.cert.pem").write_text("public", encoding="utf-8")
    enclave = view / "enclaves" / "elesim" / system / role / endpoint
    enclave.mkdir(parents=True)
    (enclave / "cert.pem").write_text(f"{marker}-cert", encoding="utf-8")
    (enclave / "key.pem").write_text(f"{marker}-key", encoding="utf-8")
    return view


def _views(tmp_path: Path, system: str, marker: str = "one") -> dict[str, Path]:
    source = tmp_path / f"source-{system}-{marker}"
    return {
        role: _view(source, system, role, f"{system}_{role}", marker)
        for role in ("pilot", "sim")
    }


def test_two_systems_and_rotation_are_scoped(tmp_path: Path) -> None:
    prefix = tmp_path / "prefix"
    alpha = _instance("alpha")
    beta = _instance("beta")
    first = stage_instance_security(prefix, INSTALL, alpha, "g1", _views(tmp_path, "alpha"))
    activate_instance_security(prefix, INSTALL, alpha, "g1")
    stage_instance_security(prefix, INSTALL, beta, "g1", _views(tmp_path, "beta"))
    activate_instance_security(prefix, INSTALL, beta, "g1")
    alpha_before = (prefix / "instances/alpha/security/current").readlink()

    rotated = stage_instance_security(prefix, INSTALL, alpha, "g2", _views(tmp_path, "alpha", "two"))
    activate_instance_security(prefix, INSTALL, alpha, "g2")

    assert first.root.is_dir() and rotated.root.is_dir()
    assert (prefix / "instances/beta/security/current").resolve().name == "g1"
    assert (prefix / "instances/alpha/security/current").resolve().name == "g2"
    assert alpha_before == Path("generations/g1")
    assert active_instance_security_path(prefix, INSTALL, "alpha", "pilot").is_dir()
    assert active_instance_security_path(prefix, INSTALL, "alpha", "alpha-pilot").is_dir()
    assert active_instance_security_path(prefix, INSTALL, "beta", "sim").is_dir()
    assert (rotated.manifest).is_file()


def test_manifest_contains_digests_and_publish_is_no_replace(tmp_path: Path) -> None:
    prefix = tmp_path / "prefix"
    instance = _instance("alpha")
    result = stage_instance_security(prefix, INSTALL, instance, release_key="g1", source_role_views=_views(tmp_path, "alpha"))
    payload = json.loads(result.manifest.read_text(encoding="utf-8"))
    assert payload["install_uuid"] == INSTALL
    assert payload["system_id"] == "alpha"
    assert payload["generation"] == "g1"
    assert payload["instance_release_key"] == instance.release_key
    for relative, digest in payload["files"].items():
        assert hashlib.sha256((result.root / relative).read_bytes()).hexdigest() == digest
    with pytest.raises(FileExistsError):
        stage_instance_security(prefix, INSTALL, instance, "g1", _views(tmp_path, "alpha", "new"))


def test_failed_stage_leaves_active_generation_unchanged(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    prefix = tmp_path / "prefix"
    instance = _instance("alpha")
    stage_instance_security(prefix, INSTALL, instance, "g1", _views(tmp_path, "alpha"))
    activate_instance_security(prefix, INSTALL, instance, "g1")
    before = (prefix / "instances/alpha/security/current").readlink()
    original = security_module._copy_regular_file
    calls = 0

    def fail_after_one(source: Path, destination: Path, *args, **kwargs):
        nonlocal calls
        calls += 1
        if calls > 1:
            raise OSError("injected copy failure")
        return original(source, destination, *args, **kwargs)

    monkeypatch.setattr(security_module, "_copy_regular_file", fail_after_one)
    with pytest.raises(OSError, match="injected"):
        stage_instance_security(prefix, INSTALL, instance, "g2", _views(tmp_path, "alpha", "two"))
    assert (prefix / "instances/alpha/security/current").readlink() == before
    assert not (prefix / "instances/alpha/security/generations/g2").exists()


@pytest.mark.parametrize("bad_name", ["ca.key.pem", "identity_ca.key.pem", "permissions_ca.key.pem"])
def test_authority_private_material_is_rejected(tmp_path: Path, bad_name: str) -> None:
    views = _views(tmp_path, "alpha")
    (views["pilot"] / "public" / bad_name).write_text("private", encoding="utf-8")
    with pytest.raises(SecurityAuthorityError, match="private"):
        stage_instance_security(tmp_path / "prefix", INSTALL, _instance("alpha"), "g1", views)


def test_unrelated_role_duplicate_and_symlink_are_rejected(tmp_path: Path) -> None:
    views = _views(tmp_path, "alpha")
    outside = tmp_path / "outside"
    outside.write_text("escape", encoding="utf-8")
    (views["pilot"] / "public" / "escape.pem").symlink_to(outside)
    with pytest.raises(SecurityAuthorityError, match="symlink"):
        stage_instance_security(tmp_path / "prefix", INSTALL, _instance("alpha"), "g1", views)
    with pytest.raises(ValueError, match="unique"):
        _instance("alpha", duplicate=True)


def test_source_bundle_manifest_is_verified_and_only_selected_view_copied(tmp_path: Path) -> None:
    # A source role view can be handed in directly; this also proves the
    # resulting tree has no host-bundle sibling role or authority directory.
    views = _views(tmp_path, "alpha")
    result = stage_instance_security(tmp_path / "prefix", INSTALL, _instance("alpha"), "g1", views)
    assert not (result.root / "authority").exists()
    assert not (result.root / "apps" / "ui").exists()


def test_noreplace_preserves_competing_destination(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    prefix = tmp_path / "prefix"
    instance = _instance("alpha")
    destination = prefix / "instances/alpha/security/generations/g1"
    original = security_module._rename_noreplace

    def compete(source: Path, target: Path) -> None:
        target.mkdir(parents=True)
        (target / "competitor").write_text("winner", encoding="utf-8")
        original(source, target)

    monkeypatch.setattr(security_module, "_rename_noreplace", compete)
    with pytest.raises(FileExistsError):
        stage_instance_security(prefix, INSTALL, instance, "g1", _views(tmp_path, "alpha"))
    assert (destination / "competitor").read_text(encoding="utf-8") == "winner"


def test_fifo_security_lock_fails_closed_without_blocking(tmp_path: Path) -> None:
    instance = _instance("alpha")
    prefix = tmp_path / "prefix"
    lock_parent = prefix / "instances/alpha"
    lock_parent.mkdir(parents=True)
    security = lock_parent / "security"
    security.mkdir()
    os.mkfifo(security / ".lock")
    with pytest.raises(SecurityAuthorityError, match="regular file"):
        stage_instance_security(prefix, INSTALL, instance, "g1", _views(tmp_path, "alpha"))


def test_hardlinked_security_lock_fails_closed(tmp_path: Path) -> None:
    instance = _instance("alpha")
    prefix = tmp_path / "prefix"
    security = prefix / "instances/alpha/security"
    security.mkdir(parents=True)
    outside = tmp_path / "outside.lock"
    outside.write_text("")
    (security / ".lock").hardlink_to(outside)
    with pytest.raises(SecurityAuthorityError, match="singly-linked"):
        stage_instance_security(prefix, INSTALL, instance, "g1", _views(tmp_path, "alpha"))


def test_activation_and_active_lookup_validate_install_and_release_binding(tmp_path: Path) -> None:
    prefix = tmp_path / "prefix"
    instance = _instance("alpha")
    result = stage_instance_security(prefix, INSTALL, instance, "g1", _views(tmp_path, "alpha"))
    payload = json.loads(result.manifest.read_text(encoding="utf-8"))
    payload["instance_release_key"] = "b" * 64
    result.manifest.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(SecurityAuthorityError, match="release binding"):
        activate_instance_security(prefix, INSTALL, instance, "g1")
    payload["instance_release_key"] = instance.release_key
    payload["files"]["../../escape"] = "0" * 64
    result.manifest.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(SecurityAuthorityError, match="manifest path"):
        activate_instance_security(prefix, INSTALL, "alpha", "g1")


def test_active_lookup_requires_install_uuid_and_validates_manifest(tmp_path: Path) -> None:
    prefix = tmp_path / "prefix"
    instance = _instance("alpha")
    stage_instance_security(prefix, INSTALL, instance, "g1", _views(tmp_path, "alpha"))
    activate_instance_security(prefix, INSTALL, instance, "g1")
    with pytest.raises(ValueError, match="canonical UUID"):
        active_instance_security_path(prefix, "not-an-install", "alpha", "pilot")
    manifest = prefix / "instances/alpha/security/current/manifest.json"
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    payload["endpoints"][0]["enclave"] = "enclaves/escape"
    manifest.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(SecurityAuthorityError, match="enclave path"):
        active_instance_security_path(prefix, INSTALL, "alpha", "pilot")
