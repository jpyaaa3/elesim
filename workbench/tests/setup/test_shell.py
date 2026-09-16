from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

from elesim_setup.shell import (
    inspect_bash_path,
    managed_path_block,
    register_bash_path,
    unregister_bash_path,
    render_operator_wrapper,
    write_executable,
)


@pytest.mark.parametrize("action,mapped", [("up", "up"), ("down", "down"), ("logs", "logs"), ("info", "status"), ("remove", "remove")])
def test_operator_routes_system_and_preserves_arguments(tmp_path, action, mapped):
    bin_dir = tmp_path / "bin with spaces"
    write_executable(bin_dir / "elesim", render_operator_wrapper(bin_dir, scoped=True))
    write_executable(bin_dir / "elesim-instance", '#!/bin/bash\nprintf "%s\\n" "$@"\n')
    result = subprocess.run([bin_dir / "elesim", action, "alpha", "two words"], capture_output=True, text=True)
    assert result.returncode == 0
    assert result.stdout.splitlines() == ["alpha", mapped, "two words"]


@pytest.mark.parametrize("sidecar,status", [(False, 0), (True, 0), (True, 17)])
def test_operator_update_orders_backends_and_stops_on_failure(tmp_path, sidecar, status):
    write_executable(tmp_path / "elesim", render_operator_wrapper(tmp_path, scoped=True, tailscale=sidecar))
    write_executable(tmp_path / "elesim-update", '#!/bin/bash\nprintf "source-update\\n"\n')
    write_executable(tmp_path / "elesim-tailscale", f'#!/bin/bash\nprintf "tailscale-%s\\n" "$1"\nexit {status}\n')
    result = subprocess.run([tmp_path / "elesim", "update"], capture_output=True, text=True)
    assert result.returncode == status
    assert result.stdout.splitlines() == (["tailscale-update"] if sidecar else []) + ([] if status else ["source-update"])


def test_operator_rejects_invalid_update_before_mutation(tmp_path):
    write_executable(tmp_path / "elesim", render_operator_wrapper(tmp_path, scoped=True, tailscale=True))
    for arguments in [("update", "--bogus"), ("tailscale", "update"), ("up",)]:
        result = subprocess.run([tmp_path / "elesim", *arguments], capture_output=True, text=True)
        assert result.returncode == 64


def test_operator_native_info_and_remove(tmp_path):
    write_executable(tmp_path / "elesim", render_operator_wrapper(tmp_path, scoped=False))
    write_executable(tmp_path / "elesim-status", '#!/bin/bash\nprintf "native-info\\n"\n')
    result = subprocess.run([tmp_path / "elesim", "info"], capture_output=True, text=True)
    assert result.returncode == 0
    assert result.stdout == "native-info\n"
    result = subprocess.run([tmp_path / "elesim", "remove", "alpha"], capture_output=True, text=True)
    assert result.returncode == 64
    assert "elesim uninstall" in result.stderr


def test_managed_path_block_quotes_literal_install_path() -> None:
    block = managed_path_block(Path("/home/user/Elesim Folder/bin"))

    assert "export PATH='/home/user/Elesim Folder/bin':\"$PATH\"" in block
    assert block.startswith("# >>> Elesim managed PATH >>>")
    assert block.endswith("# <<< Elesim managed PATH <<<\n")


def test_register_bash_path_is_idempotent_and_preserves_other_content(
    tmp_path: Path,
) -> None:
    bashrc = tmp_path / ".bashrc"
    bashrc.write_text("export EDITOR=vim\n", encoding="utf-8")

    first = register_bash_path(Path("/opt/elesim/bin"), bashrc=bashrc)
    second = register_bash_path(Path("/opt/elesim/bin"), bashrc=bashrc)

    content = bashrc.read_text(encoding="utf-8")
    assert first.changed is True
    assert second.changed is False
    assert content.count("# >>> Elesim managed PATH >>>") == 1
    assert content.startswith("export EDITOR=vim\n")
    assert first.backup is not None
    assert first.backup.read_text(encoding="utf-8") == "export EDITOR=vim\n"


def test_register_replaces_previous_managed_path(tmp_path: Path) -> None:
    bashrc = tmp_path / ".bashrc"
    bashrc.write_text(managed_path_block(Path("/old/bin")), encoding="utf-8")

    register_bash_path(Path("/new/bin"), bashrc=bashrc)

    content = bashrc.read_text(encoding="utf-8")
    assert "/old/bin" not in content
    assert "/new/bin" in content


def test_unregister_removes_only_exact_block_and_preserves_other_content(
    tmp_path: Path,
) -> None:
    bashrc = tmp_path / ".bashrc"
    bashrc.write_text(
        "export EDITOR=vim\n" + managed_path_block(Path("/opt/elesim/bin")),
        encoding="utf-8",
    )

    result = unregister_bash_path(Path("/opt/elesim/bin"), bashrc=bashrc)

    assert result.changed is True
    assert result.matched is True
    assert bashrc.read_text(encoding="utf-8") == "export EDITOR=vim\n"
    assert result.backup is not None


def test_unregister_preserves_foreign_or_newer_path_block(tmp_path: Path) -> None:
    bashrc = tmp_path / ".bashrc"
    original = managed_path_block(Path("/newer/bin"))
    bashrc.write_text(original, encoding="utf-8")

    assert inspect_bash_path(Path("/old/bin"), bashrc=bashrc) == "foreign"
    result = unregister_bash_path(Path("/old/bin"), bashrc=bashrc)

    assert result.changed is False
    assert result.matched is False
    assert bashrc.read_text(encoding="utf-8") == original


def test_bashrc_symlink_is_rejected_without_touching_target(tmp_path: Path) -> None:
    outside = tmp_path / "outside.bashrc"
    outside.write_text("keep-me\n", encoding="utf-8")
    linked = tmp_path / ".bashrc"
    linked.symlink_to(outside)

    with pytest.raises(ValueError, match="symlink"):
        register_bash_path(Path("/opt/elesim/bin"), bashrc=linked)
    with pytest.raises(ValueError, match="symlink"):
        inspect_bash_path(Path("/opt/elesim/bin"), bashrc=linked)
    with pytest.raises(ValueError, match="symlink"):
        unregister_bash_path(Path("/opt/elesim/bin"), bashrc=linked)

    assert outside.read_text(encoding="utf-8") == "keep-me\n"


def test_bashrc_symlinked_parent_is_rejected(tmp_path: Path) -> None:
    outside = tmp_path / "outside"
    outside.mkdir()
    linked_parent = tmp_path / "home"
    linked_parent.symlink_to(outside, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        register_bash_path(
            Path("/opt/elesim/bin"),
            bashrc=linked_parent / ".bashrc",
        )


@pytest.mark.parametrize("value", ["/tmp/bad\npath", "/tmp/bad\rpath"])
def test_path_registration_rejects_line_injection(value: str) -> None:
    with pytest.raises(ValueError):
        managed_path_block(Path(value))
