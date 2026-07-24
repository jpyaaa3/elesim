from __future__ import annotations

from pathlib import Path

import pytest

from elesim_setup.shell import managed_path_block, register_bash_path


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


@pytest.mark.parametrize("value", ["/tmp/bad\npath", "/tmp/bad\rpath"])
def test_path_registration_rejects_line_injection(value: str) -> None:
    with pytest.raises(ValueError):
        managed_path_block(Path(value))
