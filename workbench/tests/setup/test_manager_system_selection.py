import subprocess
from pathlib import Path

from elesim_setup.container_installer import _manager_wrapper


def _select(root: Path, *args: str):
    wrapper = _manager_wrapper(
        compose=root / "compose.yaml", compose_wrapper=root / "compose",
        state_path=root / "topology.json", authority_root=root / "authority",
        local_install_root=root, local_bin_dir=root / "bin",
        maintenance_root=root / "maintenance",
        install_uuid="11111111-1111-1111-1111-111111111111", guard="",
        container_network_mode="direct-host", gpu_mode="cpu", gpu_device="",
        project="elesim-runtime-test", scoped_systems=True,
    )
    fragment = wrapper[wrapper.index("manager_system=\n"):wrapper.index("manager_container=elesim-")]
    return subprocess.run(
        ["bash", "-c", "set -eu\n" + fragment + '\nprintf "%s" "$manager_system"', "test", *args],
        capture_output=True, text=True, start_new_session=True,
    )


def _saved(root: Path, name: str):
    path = root / "connections" / name / "topology.json"
    path.parent.mkdir(parents=True)
    path.write_text("{}")


def test_fresh_manager_opens_unbound_editor(tmp_path):
    result = _select(tmp_path)
    assert result.returncode == 0
    assert result.stdout.startswith("editor_")


def test_saved_system_is_not_implicitly_selected(tmp_path):
    _saved(tmp_path, "school")
    assert _select(tmp_path).stdout.startswith("editor_")


def test_multiple_systems_do_not_require_terminal(tmp_path):
    _saved(tmp_path, "alpha")
    _saved(tmp_path, "beta")
    result = _select(tmp_path)
    assert result.returncode == 0
    assert result.stdout.startswith("editor_")
    assert _select(tmp_path, "--system", "beta").stdout == "beta"


def test_explicit_new_system_and_invalid_arguments(tmp_path):
    assert _select(tmp_path, "--system=new_system").stdout == "new_system"
    for args in [("--system=",), ("--system",), ("--system=UPPER",),
                 ("--system=", "--system=second"), ("--system=../bad",)]:
        assert _select(tmp_path, *args).returncode == 2
