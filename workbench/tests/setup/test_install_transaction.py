from pathlib import Path

import pytest

from elesim_setup.container_installer import ContainerInstaller
from elesim_setup.gui import InstallCancelled
from elesim_setup.install_transaction import FreshInstallRollback
from elesim_setup.ownership import OwnershipManifest


@pytest.mark.parametrize("checkpoint", ["[2/6]", "[3/6]", "[5/6]", "[6/6]"])
def test_fresh_cancel_rolls_back_generated_outputs_and_allows_retry(
    local_state, tmp_path, monkeypatch, checkpoint,
):
    operator = tmp_path / "operator"
    operator.mkdir()
    monkeypatch.setenv("ELESIM_OPERATOR_HOME", str(operator))
    state = local_state(roles=("pilot",))
    state.prefix_path.mkdir()
    sentinel = state.prefix_path / "unrelated.txt"
    sentinel.write_text("preserve")
    state.bin_path.mkdir()
    command = state.bin_path / "unrelated-command"
    command.write_text("preserve")

    def cancel(message):
        if message.startswith(checkpoint):
            raise InstallCancelled("requested")

    with pytest.raises(InstallCancelled):
        ContainerInstaller(state, log=cancel).run()
    assert sentinel.read_text() == "preserve"
    assert command.read_text() == "preserve"
    assert not (state.prefix_path / "install-ownership.json").exists()
    assert not (state.prefix_path / "containers").exists()

    ContainerInstaller(state, log=lambda _: None).run()
    manifest = OwnershipManifest.load(state.prefix_path / "install-ownership.json")
    assert manifest.docker is not None


def test_fresh_generation_failure_rolls_back_like_cancellation(local_state, tmp_path, monkeypatch):
    operator = tmp_path / "operator"
    operator.mkdir()
    monkeypatch.setenv("ELESIM_OPERATOR_HOME", str(operator))
    state = local_state(roles=("pilot",))
    installer = ContainerInstaller(state, log=lambda _: None)

    def fail():
        raise OSError("injected tools context failure")

    monkeypatch.setattr(installer, "_write_tools_context", fail)
    with pytest.raises(OSError, match="injected"):
        installer.run()
    assert not state.prefix_path.exists()
    assert not state.bin_path.exists()
    ContainerInstaller(state, log=lambda _: None).run()


def test_rollback_refuses_existing_output(tmp_path):
    prefix = tmp_path / "prefix"
    generated = prefix / "data"
    generated.mkdir(parents=True)
    with pytest.raises(FileExistsError):
        FreshInstallRollback(prefix=prefix, bin_dir=prefix / "bin", generated_paths=[generated], enabled=True)


def test_rollback_does_not_follow_replaced_ancestor(tmp_path):
    prefix = tmp_path / "prefix"
    prefix.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    preserved = outside / "generated"
    preserved.write_text("external")
    transaction = FreshInstallRollback(
        prefix=prefix, bin_dir=prefix / "bin", generated_paths=[prefix / "generated"], enabled=True,
    )
    with pytest.raises(RuntimeError, match="rollback.*incomplete"):
        with transaction:
            prefix.rmdir()
            prefix.symlink_to(outside, target_is_directory=True)
            raise OSError("failure")
    assert preserved.read_text() == "external"


def test_commit_preserves_completed_install_when_later_reporting_fails(tmp_path):
    prefix = tmp_path / "prefix"
    generated = prefix / "data"
    with pytest.raises(RuntimeError, match="reporting"):
        with FreshInstallRollback(
            prefix=prefix, bin_dir=prefix / "bin", generated_paths=[generated], enabled=True,
        ) as transaction:
            generated.mkdir(parents=True)
            transaction.commit()
            raise RuntimeError("reporting")
    assert generated.is_dir()
