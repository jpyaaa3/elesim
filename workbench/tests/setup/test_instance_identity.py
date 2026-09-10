from __future__ import annotations

import hashlib

import pytest

from elesim_setup.instance_identity import (
    container_name,
    image_reference,
    manager_container_name,
    project_name,
    service_key,
)


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"


def test_project_and_container_names_are_install_scoped_and_bounded() -> None:
    project = project_name(INSTALL)
    assert project == "elesim-runtime-0123456789abcdef0123456789abcdef"
    assert container_name(INSTALL, "pilot-main").startswith(
        "elesim-0123456789abcdef0123456789abcdef-pilot-main-"
    )
    assert len(container_name(INSTALL, "pilot-main")) <= 128
    assert project_name("01234567-89ab-cdef-0123-456789abcdea") != project


def test_service_key_is_readable_and_contains_full_tuple_digest() -> None:
    key = service_key("pilot_a", "camera-left")
    expected = hashlib.sha256(b"pilot_a\0camera-left").hexdigest()
    assert key.startswith("svc-pilot_a-camera-left-")
    assert key.endswith(expected)
    assert len(key) <= 128
    assert service_key("pilot_a", "camera_left") != key


def test_image_reference_has_install_and_fingerprint_scope() -> None:
    image = image_reference(INSTALL, "pilot", "a" * 64)
    assert image == "elesim/pilot:0123456789abcdef0123456789abcdef-" + "a" * 64
    assert ":local" not in image
    assert image_reference(INSTALL, "pilot", "b" * 64) != image


def test_manager_container_name_is_unique_per_system() -> None:
    alpha = manager_container_name(INSTALL, "alpha")
    beta = manager_container_name(INSTALL, "beta")
    assert alpha == "elesim-0123456789abcdef0123456789abcdef-manager-alpha"
    assert beta != alpha
    assert len(alpha) <= 128


@pytest.mark.parametrize(
    "function,args",
    [
        (project_name, ("01234567-89AB-cdef-0123-456789abcdef",)),
        (project_name, ("0123456789abcdef0123456789abcdef",)),
        (service_key, ("Pilot", "endpoint")),
        (service_key, ("pilot", "endpoint/slash")),
        (container_name, (INSTALL, "bad service")),
        (image_reference, (INSTALL, "pilot", "tag:local")),
        (image_reference, (INSTALL, "pilot", "A" * 64)),
        (manager_container_name, (INSTALL, "System")),
    ],
)
def test_identity_inputs_fail_closed(function, args) -> None:
    with pytest.raises(ValueError):
        function(*args)
