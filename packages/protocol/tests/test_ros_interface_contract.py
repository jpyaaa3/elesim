from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
INTERFACES = ROOT / "elesim_interfaces"


def test_ros_interface_package_declares_all_phase_one_surfaces() -> None:
    expected = {
        "msg/EndpointDescriptor.msg",
        "msg/EndpointHeartbeat.msg",
        "msg/LeaseFence.msg",
        "msg/MotionAck.msg",
        "msg/MotionCommand.msg",
        "msg/MotionLease.msg",
        "msg/OperatorView.msg",
        "msg/PeerEnvelope.msg",
        "msg/RgbdFrame.msg",
        "msg/SimulationSession.msg",
        "msg/SimulationStatus.msg",
        "msg/Telemetry.msg",
        "srv/AcquireMotionLease.srv",
        "srv/NegotiateWebRtc.srv",
        "srv/OpenSimulationSession.srv",
        "srv/OperatorCommand.srv",
        "srv/RenewMotionLease.srv",
        "action/RunOperatorWorkflow.action",
    }
    cmake = (INTERFACES / "CMakeLists.txt").read_text(encoding="utf-8")

    assert expected <= set(re.findall(r'"((?:msg|srv|action)/[^"]+)"', cmake))
    package = (INTERFACES / "package.xml").read_text(encoding="utf-8")
    assert "<name>elesim_interfaces</name>" in package
    assert "<member_of_group>rosidl_interface_packages</member_of_group>" in package


def test_peer_envelope_exports_exact_wire_bounds() -> None:
    source = (INTERFACES / "msg/PeerEnvelope.msg").read_text(encoding="utf-8")

    assert "uint16 PROTOCOL_MAJOR=6" in source
    assert "string<=128 message_id" in source
    assert "string<=64 type" in source
    assert "string<=128 source_id" in source
    assert "string<=128 source_boot_id" in source
    assert "string<=128 target_id" in source
    assert "string<=128 target_boot_id" in source
    assert "string<=128 lease_id" in source
    assert "uint32 MAX_PAYLOAD_JSON_CHARS=1048576" in source
    assert "string<=1048576 payload_json" in source
    assert "uint32 MAX_TRACE_JSON_CHARS=16384" in source
    assert "string<=16384 trace_json" in source
