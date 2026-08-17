from pathlib import Path

from elesim_setup.runtime_status import (
    render_compose_status_wrapper,
    render_native_status_wrapper,
)


def test_compose_status_reports_host_runtime_and_sim_media_facts() -> None:
    rendered = render_compose_status_wrapper(
        compose=Path("/tmp/compose.yaml"),
        project="elesim-runtime",
        edition="general",
        services=(
            ("pilot", "elesim-pilot"),
            ("sim", "elesim-sim"),
        ),
        sim_container="elesim-sim",
    )

    assert "current-host only" in rendered
    assert "hostname -I" in rendered
    assert "elesim-pilot" in rendered
    assert "CUDA_VISIBLE_DEVICES" in rendered
    assert "NVIDIA_VISIBLE_DEVICES" in rendered
    assert "--gpu-devices" in rendered
    assert "nvidia-smi --query-gpu=index,uuid" in rendered
    assert "ROS_DOMAIN_ID" in rendered
    assert "h264 encoder=" in rendered
    assert "h264_nvenc unavailable" in rendered
    assert "WebRTC DTLS/SRTP" in rendered
    assert "sim.video.frames" in rendered
    assert "ui.video.receiver" in rendered
    assert "docker stats --no-stream" in rendered
    assert rendered.index("status_container sim elesim-sim") < rendered.index(
        "status_sim_media elesim-sim"
    )


def test_native_status_reports_both_robot_units() -> None:
    rendered = render_native_status_wrapper(
        robot_unit="elesim-robot.service",
        bridge_unit="elesim-unitree-bridge.service",
    )

    assert "systemctl is-active" in rendered
    assert "elesim-robot.service" in rendered
    assert "elesim-unitree-bridge.service" in rendered
    assert "--gpu-devices" in rendered
