from __future__ import annotations

from pathlib import Path

import pytest

from elesim_setup.shell import (
    inspect_bash_path,
    managed_path_block,
    register_bash_path,
    unregister_bash_path,
)


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
