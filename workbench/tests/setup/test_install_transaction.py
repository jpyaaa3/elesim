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


def test_cancelled_refresh_restores_control_files_and_allows_retry(local_state, tmp_path, monkeypatch):
    monkeypatch.setenv("ELESIM_OPERATOR_HOME", str(tmp_path / "operator"))
    state = local_state(roles=("pilot",))
    ContainerInstaller(state, log=lambda _: None).run()
    manifest_path = state.prefix_path / "install-ownership.json"
    manifest = OwnershipManifest.load(manifest_path)
    controls = [Path(entry.path) for entry in manifest.wrappers]
    controls.extend([state.state_path, state.prefix_path / "containers/compose.yaml", manifest_path])
    before = {path: path.read_bytes() for path in controls}
    changed = local_state(roles=("pilot", "ui"))

    def cancel_after_wrappers(message):
        if message.startswith("[6/6]"):
            raise InstallCancelled("requested")

    with pytest.raises(InstallCancelled):
        ContainerInstaller(changed, log=cancel_after_wrappers).run()
    for path, content in before.items():
        assert path.read_bytes() == content, path
    ContainerInstaller(changed, log=lambda _: None).run()
    assert OwnershipManifest.load(manifest_path).install_uuid == manifest.install_uuid


def test_refresh_control_rollback_preserves_modes_and_removes_new_files(tmp_path):
    from elesim_setup.install_transaction import RefreshControlRollback
    manifest = tmp_path / "manifest.json"
    manifest.write_text("original ownership")
    wrapper = tmp_path / "wrapper"
    wrapper.write_text("old")
    wrapper.chmod(0o750)
    new = tmp_path / "new-wrapper"
    with pytest.raises(OSError, match="injected"):
        with RefreshControlRollback(manifest=manifest, paths=[wrapper, new]):
            wrapper.write_text("partial")
            wrapper.chmod(0o600)
            new.write_text("new")
            raise OSError("injected")
    assert wrapper.read_text() == "old"
    assert wrapper.stat().st_mode & 0o777 == 0o750
    assert not new.exists()


def test_refresh_control_rollback_refuses_changed_ownership(tmp_path):
    from elesim_setup.install_transaction import RefreshControlRollback
    manifest = tmp_path / "manifest.json"
    manifest.write_text("old")
    wrapper = tmp_path / "wrapper"
    wrapper.write_text("old")
    with pytest.raises(RuntimeError, match="ownership changed"):
        with RefreshControlRollback(manifest=manifest, paths=[wrapper]):
            manifest.write_text("different writer committed")
            wrapper.write_text("different writer")
            raise OSError("failure")
    assert wrapper.read_text() == "different writer"


def test_refresh_control_rollback_never_follows_replaced_symlink(tmp_path):
    from elesim_setup.install_transaction import RefreshControlRollback
    manifest = tmp_path / "manifest.json"
    manifest.write_text("old")
    wrapper = tmp_path / "wrapper"
    wrapper.write_text("old")
    external = tmp_path / "external"
    external.write_text("preserve")
    with pytest.raises(RuntimeError, match="rollback was incomplete"):
        with RefreshControlRollback(manifest=manifest, paths=[wrapper]):
            wrapper.unlink()
            wrapper.symlink_to(external)
            raise OSError("failure")
    assert external.read_text() == "preserve"


def test_committed_refresh_is_not_rolled_back_on_later_error(tmp_path):
    from elesim_setup.install_transaction import RefreshControlRollback
    manifest = tmp_path / "manifest.json"
    manifest.write_text("old")
    wrapper = tmp_path / "wrapper"
    wrapper.write_text("old")
    with pytest.raises(OSError):
        with RefreshControlRollback(manifest=manifest, paths=[wrapper]) as transaction:
            wrapper.write_text("committed")
            manifest.write_text("committed")
            transaction.commit()
            raise OSError("reporting failed")
    assert wrapper.read_text() == "committed"


def test_install_plan_points_to_topology_setup(local_state):
    from elesim_setup.container_installer import build_container_plan
    state = local_state(roles=("pilot", "sim", "ui"))
    actions = build_container_plan(state)
    next_action = next(action for action in actions if action.title == "Next")
    assert str(state.bin_path / "elesim-connections") in next_action.detail
    assert all("elesim-up" not in action.detail for action in actions)


def test_progress_can_cancel_next_installation_after_previous_commit():
    from elesim_setup.install_progress import InstallProgress
    cancelled = False
    progress = InstallProgress(lambda _: None, lambda: cancelled)
    progress.begin_installation()
    progress.commit_installation()
    cancelled = True
    progress("completed first installation")
    with pytest.raises(InstallCancelled):
        progress.begin_installation()


def test_container_commit_callback_precedes_completion_logs(local_state, monkeypatch, tmp_path):
    from elesim_setup.install_progress import InstallProgress
    monkeypatch.setenv("ELESIM_OPERATOR_HOME", str(tmp_path / "operator"))
    cancelled = False

    def write(message):
        nonlocal cancelled
        if message.startswith("[Complete]"):
            cancelled = True

    progress = InstallProgress(write, lambda: cancelled)
    state = local_state(roles=("pilot",))
    ContainerInstaller(state, log=progress, on_commit=progress.commit_installation).run()
    assert progress.committed
    assert OwnershipManifest.load(state.prefix_path / "install-ownership.json")
