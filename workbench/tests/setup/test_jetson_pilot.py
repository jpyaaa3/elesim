import os
import subprocess
import sys
from pathlib import Path

import pytest

from elesim_setup.capabilities import detect_jetson_l4t
from elesim_setup.container_installer import ContainerInstaller
from elesim_setup.state import ComputeSettings


@pytest.fixture
def jetson_host(monkeypatch):
    monkeypatch.setenv("ELESIM_HOST_JETSON", "1")
    monkeypatch.setenv(
        "ELESIM_HOST_L4T_RELEASE",
        "# R36 (release), REVISION: 5.0, BOARD: generic, EABI: aarch64",
    )


@pytest.mark.parametrize("gpu_mode", ("inherit", "specific"))
def test_jetson_pilot_context_uses_pinned_cuda126_wheels(local_state, jetson_host, gpu_mode):
    state = local_state(
        roles=("pilot",),
        compute=ComputeSettings(gpu_mode=gpu_mode, gpu_device="GPU-orin" if gpu_mode == "specific" else ""),
    )
    installer = ContainerInstaller(state)
    installer._write_role_context("pilot")
    context = state.prefix_path / "containers/build/pilot"
    lock = (context / "requirements.lock").read_text()

    assert "torch==2.12.1" not in lock
    assert "torchvision==0.27.1" not in lock
    assert "torch-2.11.0-cp310-cp310-linux_aarch64.whl#sha256=46bb8b13f844b2111f6f41abc703d01075a7185e0c587cb002e957ede2b8c8a3" in lock
    assert "torchvision-0.26.0-cp310-cp310-linux_aarch64.whl#sha256=d116f08d3d62417dc36cbf41923e491b8b49b77399575f0878ae391e6e77a159" in lock
    assert installer._role_build_args("pilot")["PILOT_TORCH_STACK"] == "jetson-cu126"
    assert (context / "install_jetson_cuda.sh").is_file()


def test_jetson_cpu_pilot_keeps_cpu_dependencies(local_state, jetson_host):
    state = local_state(roles=("pilot",), compute=ComputeSettings(gpu_mode="cpu"))
    installer = ContainerInstaller(state)
    installer._write_role_context("pilot")
    context = state.prefix_path / "containers/build/pilot"
    assert "torch==2.12.1" in (context / "requirements.lock").read_text()
    assert installer._role_build_args("pilot")["COMPUTE_MODE"] == "cpu"
    assert installer._role_build_args("pilot").get("PILOT_TORCH_STACK", "generic") == "generic"


@pytest.mark.parametrize("release", ("# R35 (release), REVISION: 6.4", "# R36 (release), REVISION: 3.0", "# R36 (release), REVISION: 6.0", "# R38 (release), REVISION: 2.0", "broken"))
def test_unsupported_jetson_pilot_gpu_fails_before_context_mutation(local_state, jetson_host, monkeypatch, release):
    monkeypatch.setenv("ELESIM_HOST_L4T_RELEASE", release)
    state = local_state(roles=("pilot",))
    with pytest.raises(ValueError, match="L4T|Jetson"):
        ContainerInstaller(state)._validate_source()
    assert not state.prefix_path.exists()


def test_jetson_detection_uses_host_release_and_honors_outer_host(tmp_path):
    release = tmp_path / "etc/nv_tegra_release"
    release.parent.mkdir()
    release.write_text("# R36 (release), REVISION: 4.4, BOARD: generic\n")
    assert detect_jetson_l4t(root=tmp_path, environ={}) == (36, 4, 4)
    assert detect_jetson_l4t(root=tmp_path, environ={"ELESIM_HOST_JETSON": "0"}) is None
    assert detect_jetson_l4t(root=tmp_path, environ={
        "ELESIM_HOST_JETSON": "1", "ELESIM_HOST_L4T_RELEASE": "# R36 (release), REVISION: 5.0",
    }) == (36, 5, 0)
    with pytest.raises(ValueError, match="L4T"):
        detect_jetson_l4t(root=tmp_path / "absent", environ={"ELESIM_HOST_JETSON": "1"})


def test_jetson_stack_changes_fingerprint_and_preserves_generic_lock(local_state, jetson_host, monkeypatch):
    state = local_state(roles=("pilot",))
    installer = ContainerInstaller(state)
    installer._write_role_context("pilot")
    jetson_fingerprint = installer._image_fingerprints["pilot"]
    monkeypatch.setenv("ELESIM_HOST_JETSON", "0")
    installer._write_role_context("pilot")
    assert installer._image_fingerprints["pilot"] != jetson_fingerprint
    assert (state.prefix_path / "containers/build/pilot/requirements.lock").read_bytes() == (
        state.source_path / "payload/runtime/docker/pilot/requirements.lock"
    ).read_bytes()


@pytest.mark.parametrize("gpu_mode", ("inherit", "specific", "cpu"))
def test_jetson_runtime_keeps_gpu_reservation_and_cpu_policy(local_state, jetson_host, gpu_mode):
    state = local_state(roles=("pilot",), compute=ComputeSettings(
        gpu_mode=gpu_mode, gpu_device="GPU-orin" if gpu_mode == "specific" else "",
    ))
    installer = ContainerInstaller(state)
    installer._image_fingerprints["elesim/pilot:local"] = "b" * 64
    service = installer._role_service(
        "pilot", instance_scoped=True,
        config_root=state.prefix_path / "instances/test/config",
        cache_root=state.prefix_path / "instances/test/cache",
        data_root=state.prefix_path / "releases/pinned/data",
    )
    if gpu_mode == "cpu":
        assert "runtime" not in service and "gpus" not in service and "deploy" not in service
        assert service["environment"]["CUDA_VISIBLE_DEVICES"] == ""
    else:
        assert service["runtime"] == "nvidia"
        if gpu_mode == "specific":
            devices = service["deploy"]["resources"]["reservations"]["devices"]
            assert devices[0]["device_ids"] == ("GPU-orin",)
            assert "CUDA_VISIBLE_DEVICES" not in service["environment"]
        else:
            assert service["gpus"] == "all"


def test_jetson_tools_preserves_runtime_facts_for_scoped_registration(local_state, jetson_host, monkeypatch):
    installer = ContainerInstaller(local_state(roles=("pilot",)))
    installer._image_fingerprints["elesim/tools:local"] = "a" * 64
    environment = installer._tools_service()["environment"]
    assert environment["ELESIM_HOST_JETSON"] == "1"
    monkeypatch.setenv("ELESIM_HOST_L4T_RELEASE", environment["ELESIM_HOST_L4T_RELEASE"])
    service = {"environment": {}}
    installer._apply_compute(service)
    assert service["runtime"] == "nvidia"


@pytest.mark.parametrize("failure,expected", (("download", 23), ("checksum", 1)))
def test_jetson_repository_failure_stops_before_apt(tmp_path, failure, expected):
    script = Path(__file__).resolve().parents[3] / "payload/runtime/docker/shared/install_jetson_cuda.sh"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    (fake_bin / "dpkg").write_text("#!/bin/sh\nprintf '%s\\n' arm64\n")
    (fake_bin / "curl").write_text(
        f"#!{sys.executable}\nimport sys\nfrom pathlib import Path\n"
        + ("sys.exit(23)\n" if failure == "download" else
           "Path(sys.argv[sys.argv.index('-o')+1]).write_text('invalid key')\n")
    )
    marker = tmp_path / "apt-was-called"
    (fake_bin / "apt-get").write_text(f"#!{sys.executable}\nfrom pathlib import Path\nPath({str(marker)!r}).touch()\n")
    for path in fake_bin.iterdir():
        path.chmod(0o755)
    result = subprocess.run(
        ["sh", str(script)], capture_output=True, text=True,
        env={**os.environ, "PATH": str(fake_bin) + os.pathsep + os.environ.get("PATH", "")},
    )
    assert result.returncode == expected, result.stdout + result.stderr
    assert not marker.exists()
