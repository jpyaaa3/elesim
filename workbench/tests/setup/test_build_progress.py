import json
import os
import pty
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

from elesim_setup import build_progress
from elesim_setup.ownership import install_host_uninstaller_bundle
from elesim_setup.updater import render_compose_build_progress


def invoke(tmp_path, script, mode="plain", result_file=None):
    command = [sys.executable, build_progress.__file__, "--log-dir", str(tmp_path / "logs"),
               "--mode", mode]
    if result_file is not None:
        command.extend(("--result-file", str(result_file)))
    return subprocess.run(
        [*command, "--", sys.executable, "-c", script],
        capture_output=True, timeout=10,
    )


def test_plain_progress_keeps_full_private_log_and_bounded_terminal(tmp_path):
    result = invoke(tmp_path, "for i in range(10000): print('build line', i)")
    assert result.returncode == 0, result.stderr
    assert result.stdout == b""
    assert b"Completed" in result.stderr
    assert b"...\n" in result.stderr
    assert b"lines omitted" not in result.stderr
    assert b"build line 9999" in result.stderr
    assert len(result.stderr.splitlines()) == 7
    log, = (tmp_path / "logs").glob("*.log")
    assert len(log.read_text().splitlines()) == 10000
    assert log.stat().st_mode & 0o777 == 0o600
    assert log.parent.stat().st_mode & 0o777 == 0o700


def test_build_report_lists_exported_role_images(tmp_path):
    result_file = tmp_path / "result" / "images.txt"
    result = invoke(
        tmp_path,
        "print('#52 naming to docker.io/elesim/sim:quick_zebra-ivory_llama 0.0s done'); "
        "print('#72 naming to docker.io/elesim/pilot:quick_zebra-plain_horse done'); "
        "print('#73 naming to docker.io/elesim/ui:quick_zebra-silent_lynx done'); "
        "print('#90 naming to docker.io/elesim/tools:quick_zebra-jolly_canary done')",
        result_file=result_file,
    )
    assert result.returncode == 0, result.stderr
    assert result_file.read_text() == (
        "Installation name=quick_zebra\n"
        "elesim/tools=jolly_canary\n"
        "elesim/sim=ivory_llama\n"
        "elesim/pilot=plain_horse\n"
        "elesim/ui=silent_lynx\n"
    )
    assert b"Built images" not in result.stderr


def test_build_report_accepts_compose_image_summary_lines(tmp_path):
    result_file = tmp_path / "result" / "images.txt"
    result = invoke(
        tmp_path,
        "print(' Image elesim/tools:quick_zebra-jolly_canary Built'); "
        "print(' Image elesim/sim:quick_zebra-ivory_llama Built'); "
        "print(' Image elesim/pilot:quick_zebra-plain_horse Built'); "
        "print(' Image elesim/ui:quick_zebra-silent_lynx Built')",
        result_file=result_file,
    )
    assert result.returncode == 0, result.stderr
    assert result_file.read_text() == (
        "Installation name=quick_zebra\n"
        "elesim/tools=jolly_canary\n"
        "elesim/sim=ivory_llama\n"
        "elesim/pilot=plain_horse\n"
        "elesim/ui=silent_lynx\n"
    )


def test_complete_image_report_includes_reused_expected_images(tmp_path):
    result_file = tmp_path / "result" / "images.txt"
    result = subprocess.run(
        (
            sys.executable,
            str(build_progress.__file__),
            "--write-report",
            "--result-file",
            str(result_file),
            "--expected-image",
            "elesim/ui:quick_zebra-silent_lynx",
            "--expected-image",
            "elesim/tools:quick_zebra-jolly_canary",
            "--expected-image",
            "elesim/sim:quick_zebra-ivory_llama",
        ),
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert result_file.read_text() == (
        "Installation name=quick_zebra\n"
        "elesim/tools=jolly_canary\n"
        "elesim/sim=ivory_llama\n"
        "elesim/ui=silent_lynx\n"
    )


@pytest.mark.parametrize("mode", ["auto", "verbose"])
def test_non_tty_stream_is_preserved_for_connection_manager(tmp_path, mode):
    result = invoke(tmp_path, "import os; os.write(1,b'out\\n'); os.write(2,b'err\\n')", mode)
    assert result.returncode == 0
    assert result.stdout == b"out\nerr\n"
    log, = (tmp_path / "logs").glob("*.log")
    assert log.read_bytes() == result.stdout
    assert b"\x1b" not in result.stderr


def test_failure_returns_original_status_and_only_safe_tail(tmp_path):
    result = invoke(tmp_path, "import sys; print('\\x1b[31merror\\x1b[0m'); "
                    "[print('line', i) for i in range(40)]; sys.exit(42)")
    assert result.returncode == 42
    assert b"line 39" in result.stderr and b"line 0\n" not in result.stderr
    assert b"\x1b" not in result.stderr
    assert len(result.stderr.splitlines()) <= 16
    log, = (tmp_path / "logs").glob("*.log")
    assert b"\x1b[31merror" in log.read_bytes()


def test_symlink_ancestor_refused_before_command_runs(tmp_path):
    real = tmp_path / "real"
    real.mkdir()
    (tmp_path / "logs").symlink_to(real, target_is_directory=True)
    result = invoke(tmp_path, "raise AssertionError('must not execute')")
    assert result.returncode == 74
    assert b"must not execute" not in result.stderr
    assert not list(real.iterdir())


def test_shared_log_directory_refused(tmp_path):
    logs = tmp_path / "logs"
    logs.mkdir(mode=0o755)
    logs.chmod(0o755)
    result = invoke(tmp_path, "print('must not execute')")
    assert result.returncode == 74
    assert not list(logs.iterdir())


def test_wsl_windows_credential_helper_is_removed_for_build_child(tmp_path, monkeypatch):
    docker_config = tmp_path / "docker"
    docker_config.mkdir()
    docker_config.joinpath("config.json").write_text(json.dumps({
        "auths": {"registry.example": {"auth": "keep-me"}},
        "credsStore": "desktop.exe",
        "credHelpers": {"registry.example": "desktop.exe", "other.example": "pass"},
    }))
    monkeypatch.setenv("DOCKER_CONFIG", str(docker_config))
    result = invoke(
        tmp_path,
        "from pathlib import Path; import os; "
        "print(Path(os.environ['DOCKER_CONFIG'], 'config.json').read_text())",
    )
    assert result.returncode == 0, result.stderr
    log, = (tmp_path / "logs").glob("*.log")
    sanitized = json.loads(log.read_text())
    assert sanitized == {
        "auths": {"registry.example": {"auth": "keep-me"}},
        "credHelpers": {"other.example": "pass"},
    }
    assert json.loads(docker_config.joinpath("config.json").read_text())["credsStore"] == "desktop.exe"


def test_signal_reaches_child_and_keeps_transcript(tmp_path):
    process = subprocess.Popen(
        [sys.executable, build_progress.__file__, "--log-dir", str(tmp_path / "logs"),
         "--mode", "plain", "--", sys.executable, "-c",
         "import signal,time; signal.signal(signal.SIGTERM, lambda *_: exit(0)); "
         "print('ready', flush=True); time.sleep(60)"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            logs = list((tmp_path / "logs").glob("*.log"))
            if logs and b"ready" in logs[0].read_bytes():
                break
            time.sleep(0.02)
        else:
            pytest.fail("child did not become ready")
        process.send_signal(signal.SIGTERM)
        _, stderr = process.communicate(timeout=8)
        assert process.returncode == 143, stderr
        assert b"ready" in logs[0].read_bytes()
    finally:
        if process.poll() is None:
            process.terminate()
            process.communicate(timeout=8)


def test_host_bundle_contains_standalone_progress_program(tmp_path):
    prefix, bin_dir = tmp_path / "install", tmp_path / "bin"
    prefix.mkdir()
    bin_dir.mkdir()
    install_host_uninstaller_bundle(prefix=prefix, bin_dir=bin_dir)
    helper = prefix / "maintenance/elesim_setup/build_progress.py"
    result = subprocess.run([sys.executable, "-I", str(helper), "--help"], capture_output=True)
    assert result.returncode == 0, result.stderr


def test_terminal_text_removes_escape_sequences_and_respects_wide_characters():
    assert build_progress._display_text("\x1b[31mred\x1b[0m\x1b]0;title\x07") == "red"
    assert build_progress._fit_row("가나다abc", 5) == "가나"
    assert build_progress._muted("  └ Full log: /tmp/build.log", True).startswith("\x1b[90m")
    assert build_progress._muted("  └ Full log: /tmp/build.log", False) == "  └ Full log: /tmp/build.log"


def test_real_terminal_redraws_instead_of_appending_build_output(tmp_path):
    master, slave = pty.openpty()
    process = subprocess.Popen(
        [sys.executable, build_progress.__file__, "--log-dir", str(tmp_path / "logs"),
         "--", sys.executable, "-c",
         "import time; [print('line', i) for i in range(500)]; time.sleep(0.6)"],
        stdout=slave, stderr=slave, env={**os.environ, "TERM": "xterm",
                                      "ELESIM_BUILD_PROGRESS": "auto", "ELESIM_VERBOSE": "0"},
    )
    os.close(slave)
    output = bytearray()
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if select.select([master], [], [], 0.1)[0]:
                try:
                    chunk = os.read(master, 65536)
                except OSError:
                    break  # PTY slave closed.
                if not chunk:
                    break
                output.extend(chunk)
        process.wait(timeout=2)
        assert process.returncode == 0, bytes(output)
        assert b"\x1b[2K" in output
        assert b"line 0\r\n" not in output
        assert output.count(b"\n") == 7
        log, = (tmp_path / "logs").glob("*.log")
        assert len(log.read_text().splitlines()) == 500
    finally:
        os.close(master)
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=8)


def test_compact_pipe_does_not_fall_back_to_raw_output(tmp_path):
    result = invoke(tmp_path, "[print('dependency', i) for i in range(100)]", "compact")
    assert result.returncode == 0
    assert result.stdout == b""
    assert b"...\n" in result.stderr
    assert b"dependency 0\n" not in result.stderr
    assert b"\x1b" not in result.stderr


def test_installer_notices_remain_visible_while_dependency_output_is_folded(tmp_path):
    result = subprocess.run(
        [sys.executable, build_progress.__file__, "--log-dir", str(tmp_path / "logs"),
         "--mode", "compact", "--title", "Generate installation artifacts",
         "--notice-prefix", "[", "--notice-prefix", "$ ", "--", sys.executable, "-c",
         "print('[warning] Review interface settings'); "
         "[print('dependency', i) for i in range(100)]; "
         "print('$ sudo systemctl daemon-reload')"],
        capture_output=True, timeout=10,
    )
    assert result.returncode == 0
    assert b"[warning] Review interface settings" in result.stderr
    assert b"$ sudo systemctl daemon-reload" in result.stderr
    assert b"...\n" in result.stderr
    assert b"dependency" not in result.stderr
    log, = (tmp_path / "logs").glob("*.log")
    assert len(log.read_text().splitlines()) == 102


def test_verbose_environment_overrides_explicit_compact(tmp_path, monkeypatch):
    monkeypatch.setenv("ELESIM_VERBOSE", "1")
    result = invoke(tmp_path, "print('raw detail')", "compact")
    assert result.returncode == 0
    assert result.stdout == b"raw detail\n"


@pytest.mark.parametrize("arguments,wrapped", [
    (["--progress", "plain", "-f", "compose.yaml", "build", "pilot"], True),
    (["--file", "build", "config"], False),
    (["--file=compose.yaml", "build", "ui"], True),
    (["exec", "tools", "echo", "build"], False),
])
def test_updated_compose_catches_old_update_build_without_intercepting_other_commands(
    tmp_path, arguments, wrapped,
):
    prefix = tmp_path / "install with spaces"
    prefix.mkdir()
    bin_dir = prefix / "bin"
    bin_dir.mkdir()
    install_host_uninstaller_bundle(prefix=prefix, bin_dir=bin_dir)
    docker = bin_dir / "docker"
    docker.write_text("#!/bin/sh\nprintf 'docker detail\\n'\nexit 23\n")
    docker.chmod(0o755)
    wrapper = bin_dir / "elesim-compose"
    wrapper.write_text("#!/bin/bash\nset -euo pipefail\n" + render_compose_build_progress(prefix)
                       + 'exec docker compose "$@"\n')
    wrapper.chmod(0o755)
    result = subprocess.run([str(wrapper), *arguments], capture_output=True, timeout=10,
                            env={**os.environ, "PATH": f"{bin_dir}:{os.environ['PATH']}",
                                 "ELESIM_BUILD_PROGRESS": "compact", "ELESIM_VERBOSE": "0",
                                 "ELESIM_PROGRESS_ACTIVE": "0"})
    assert result.returncode == 23
    assert (b"...\n" in result.stderr) == wrapped
    assert len(list((prefix / "logs/build").glob("*.log"))) == int(wrapped)
    if wrapped:
        result = subprocess.run(
            [sys.executable, build_progress.__file__, "--log-dir", str(prefix / "logs/build"),
             "--mode", "compact", "--", str(wrapper), *arguments],
            capture_output=True, timeout=10,
            env={**os.environ, "PATH": f"{bin_dir}:{os.environ['PATH']}", "ELESIM_VERBOSE": "0"},
        )
        assert result.returncode == 23
        assert result.stderr.count(b"Runtime image build") == 1
        assert len(list((prefix / "logs/build").glob("*.log"))) == 2
