from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]


def test_setup_runtime_depends_on_ros_interfaces_not_pyzmq() -> None:
    requirements = (
        ROOT / "misc/tooling/setup/requirements.lock"
    ).read_text(encoding="utf-8")
    metadata = (ROOT / "misc/tooling/setup/pyproject.toml").read_text(
        encoding="utf-8"
    )

    assert "pyzmq" not in requirements.lower()
    assert "pyzmq" not in metadata.lower()
    assert (ROOT / "packages/elesim_interfaces/package.xml").is_file()
    assert (ROOT / "packages/elesim_interfaces/msg/RgbdFrame.msg").is_file()


def test_container_images_build_the_interface_overlay() -> None:
    app = (ROOT / "misc/infra/containers/Dockerfile.app").read_text(
        encoding="utf-8"
    )
    tools = (ROOT / "misc/infra/containers/Dockerfile.tools").read_text(
        encoding="utf-8"
    )

    for dockerfile in (app, tools):
        assert "interfaces/elesim_interfaces" in dockerfile
        assert "colcon" in dockerfile
        assert " build " in dockerfile
        assert "rmw-cyclonedds-cpp" in dockerfile
        assert "rosidl-default-generators" in dockerfile
