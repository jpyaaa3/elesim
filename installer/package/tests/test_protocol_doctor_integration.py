from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_setup_runtime_depends_on_ros_interfaces_not_pyzmq() -> None:
    requirements = (
        ROOT / "installer/package/requirements.lock"
    ).read_text(encoding="utf-8")
    metadata = (ROOT / "installer/package/pyproject.toml").read_text(
        encoding="utf-8"
    )

    assert "pyzmq" not in requirements.lower()
    assert "pyzmq" not in metadata.lower()
    assert (ROOT / "packages/elesim_interfaces/package.xml").is_file()
    assert (ROOT / "packages/elesim_interfaces/msg/RgbdFrame.msg").is_file()


def test_container_images_build_the_interface_overlay() -> None:
    app = (ROOT / "environment/containers/Dockerfile.app").read_text(
        encoding="utf-8"
    )
    tools = (ROOT / "environment/containers/Dockerfile.tools").read_text(
        encoding="utf-8"
    )

    for dockerfile in (app, tools):
        assert "interfaces/elesim_interfaces" in dockerfile
        assert "colcon" in dockerfile
        assert " build " in dockerfile
        assert "rmw-cyclonedds-cpp" in dockerfile
        assert "rosidl-default-generators" in dockerfile
    # ``elesim-net namespace-check`` runs inside the tools image and uses
    # ``ip -j route get`` for static peers.  Without iproute2 the check would
    # fail only after installation, making a valid topology look broken.
    assert "iproute2" in tools
