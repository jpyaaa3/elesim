"""UI graphics policy must use the host display backend and the selected adapter."""

import pytest

from elesim_setup import container_installer
from elesim_setup.capabilities import GpuDevice
from elesim_setup.container_installer import ContainerInstaller
from elesim_setup.state import ComputeSettings


@pytest.fixture
def ui_service(local_state, monkeypatch):
    def make(compute, *, wslg=True, devices=None, scoped=False):
        monkeypatch.setattr(container_installer, "detect_install_wslg", lambda: wslg)
        monkeypatch.setattr(container_installer, "detect_install_gpu_devices", lambda:
            tuple(devices if devices is not None else [
                GpuDevice("0", "NVIDIA GeForce RTX 3050 Laptop GPU", "GPU-display"),
                GpuDevice("2", "NVIDIA RTX A6000", "GPU-compute"),
            ]))
        state = local_state(roles=("ui",))
        installer = ContainerInstaller(state)
        installer._image_fingerprints["elesim/ui:local"] = "a" * 64
        root = state.prefix_path / "instances/test"
        paths = dict(config_root=root / "config", cache_root=root / "cache",
            data_root=root / "data", instance_scoped=True) if scoped else {}
        return installer._role_service("ui", compute=compute, **paths)
    return make


@pytest.mark.parametrize("scoped", [False, True])
@pytest.mark.parametrize("selector", ["2", "GPU-compute"])
def test_wslg_selected_gpu_uses_d3d12_resources_and_adapter_name(ui_service, scoped, selector):
    service = ui_service(ComputeSettings("specific", selector), scoped=scoped)
    env = service["environment"]
    assert env["ELESIM_UI_GRAPHICS_BACKEND"] == "wslg"
    assert env["__GLX_VENDOR_LIBRARY_NAME"] == "mesa"
    assert env["GALLIUM_DRIVER"] == "d3d12"
    assert env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] == "NVIDIA RTX A6000"
    assert env["LIBGL_ALWAYS_SOFTWARE"] == "0"
    assert env["LD_LIBRARY_PATH"].split(":")[0] == "/usr/lib/wsl/lib"
    assert "/usr/lib/wsl:/usr/lib/wsl:ro" in service["volumes"]
    assert "/dev/dxg:/dev/dxg" in service["devices"]
    assert "__NV_PRIME_RENDER_OFFLOAD" not in env
    assert "CUDA_VISIBLE_DEVICES" not in env
    assert service["deploy"]["resources"]["reservations"]["devices"][0]["device_ids"] == (selector,)


def test_wslg_free_gpu_has_no_host_index_override(ui_service):
    service = ui_service(ComputeSettings("inherit"))
    assert service["environment"]["MESA_D3D12_DEFAULT_ADAPTER_NAME"] == "NVIDIA GeForce RTX 3050 Laptop GPU"
    assert service["gpus"] == "all"
    assert service["environment"]["CUDA_VISIBLE_DEVICES"] is None


@pytest.mark.parametrize("compute", [ComputeSettings("cpu"), ComputeSettings("specific", "0")])
def test_cpu_and_native_gpu_avoid_wslg_resources(ui_service, compute):
    service = ui_service(compute, wslg=compute.gpu_mode == "cpu")
    assert not any("/usr/lib/wsl" in item for item in service["volumes"])
    assert not any("/dev/dxg" in item for item in service.get("devices", []))
    assert "GALLIUM_DRIVER" not in service["environment"]
    if compute.gpu_mode == "cpu":
        assert service["environment"]["LIBGL_ALWAYS_SOFTWARE"] == "1"
    else:
        assert service["environment"]["__GLX_VENDOR_LIBRARY_NAME"] == "nvidia"


@pytest.mark.parametrize("devices", [[], [GpuDevice("1", "NVIDIA RTX", "GPU-other")]])
def test_wslg_refuses_missing_selected_adapter(ui_service, devices):
    with pytest.raises(ValueError, match="WSLg.*GPU"):
        ui_service(ComputeSettings("specific", "0"), devices=devices)


def test_wslg_refuses_ambiguous_specific_adapter_name(ui_service):
    devices = [GpuDevice("0", "NVIDIA RTX", "GPU-a"), GpuDevice("1", "NVIDIA RTX", "GPU-b")]
    with pytest.raises(ValueError, match="WSLg.*adapter name"):
        ui_service(ComputeSettings("specific", "GPU-b"), devices=devices)


def test_tools_registration_keeps_outer_wslg_facts_without_exposing_devices(local_state, monkeypatch):
    monkeypatch.setattr(container_installer, "detect_install_wslg", lambda: True)
    monkeypatch.setattr(container_installer, "detect_install_gpu_devices", lambda: (
        GpuDevice("0", "NVIDIA RTX 3050 Laptop GPU", "GPU-display"),
    ))
    installer = ContainerInstaller(local_state(roles=("ui",)))
    installer._image_fingerprints["elesim/tools:local"] = "b" * 64
    service = installer._tools_service()
    env = service["environment"]
    assert env["ELESIM_HOST_WSLG"] == "1"
    assert env["ELESIM_HOST_GPU_LIST"] == "GPU 0: NVIDIA RTX 3050 Laptop GPU (UUID: GPU-display)"
    assert not service.get("devices")
    assert not any("/usr/lib/wsl" in item for item in service["volumes"])
