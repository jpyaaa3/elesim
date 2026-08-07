from __future__ import annotations

import os

import pytest

from elesim_protocol import DdsRuntimeSettings, DdsTransportError


def test_static_discovery_requires_an_explicit_seed() -> None:
    with pytest.raises(ValueError, match="at least one peer"):
        DdsRuntimeSettings(discovery_mode="static")


def test_legacy_automatic_interface_is_normalized_to_vendor_auto_selection() -> None:
    settings = DdsRuntimeSettings(network_interface="automatic")

    assert settings.network_interface == ""


def test_sros2_requires_keystore_and_absolute_enclave() -> None:
    with pytest.raises(ValueError, match="keystore"):
        DdsRuntimeSettings(security_profile="sros2", enclave="/elesim/robot")
    with pytest.raises(ValueError, match="absolute enclave"):
        DdsRuntimeSettings(
            security_profile="sros2",
            keystore="/tmp/keystore",
            enclave="elesim/robot",
        )


def test_mapping_derives_a_role_scoped_enclave_from_endpoint_id() -> None:
    settings = DdsRuntimeSettings.from_mapping(
        {
            "security_profile": "sros2",
            "keystore": "/tmp/keystore",
            "enclave_base": "/lab",
        },
        endpoint_id="Robot A",
    )

    assert settings.enclave.startswith("/lab/Robot_A_")


def test_apply_environment_rejects_a_conflicting_rmw(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    settings = DdsRuntimeSettings(rmw_implementation="rmw_cyclonedds_cpp")

    with pytest.raises(DdsTransportError, match="already set"):
        settings.apply_environment()


def test_trusted_network_clears_ros_security_switches(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("ROS_SECURITY_ENABLE", "true")
    monkeypatch.setenv("ROS_SECURITY_STRATEGY", "Enforce")
    monkeypatch.setenv("ROS_SECURITY_KEYSTORE", "/tmp/old-keystore")
    monkeypatch.setenv("ROS_SECURITY_ENCLAVE_OVERRIDE", "/old/enclave")
    monkeypatch.delenv("RMW_IMPLEMENTATION", raising=False)
    settings = DdsRuntimeSettings()

    assert settings.apply_environment() == ()
    assert os.environ["ROS_DOMAIN_ID"] == "0"
    assert "ROS_SECURITY_ENABLE" not in os.environ
    assert "ROS_SECURITY_STRATEGY" not in os.environ
    assert "ROS_SECURITY_KEYSTORE" not in os.environ
    assert "ROS_SECURITY_ENCLAVE_OVERRIDE" not in os.environ
