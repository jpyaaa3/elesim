from pathlib import Path
import os
import pty
import select
import signal
import subprocess
import sys
import time

import pytest

from elesim_setup import build_progress
from elesim_setup.ownership import install_host_uninstaller_bundle


def invoke(tmp_path, script, mode="plain"):
    return subprocess.run(
        [sys.executable, build_progress.__file__, "--log-dir", str(tmp_path / "logs"),
         "--mode", mode, "--", sys.executable, "-c", script],
        capture_output=True, timeout=10,
    )


def test_plain_progress_keeps_full_private_log_and_bounded_terminal(tmp_path):
    result = invoke(tmp_path, "for i in range(10000): print('build line', i)")
    assert result.returncode == 0, result.stderr
    assert result.stdout == b""
    assert b"Completed" in result.stderr
    assert len(result.stderr.splitlines()) == 2
    log, = (tmp_path / "logs").glob("*.log")
    assert len(log.read_text().splitlines()) == 10000
    assert log.stat().st_mode & 0o777 == 0o600
    assert log.parent.stat().st_mode & 0o777 == 0o700


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
    assert len(result.stderr.splitlines()) <= 14
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
        assert output.count(b"\n") == 2
        log, = (tmp_path / "logs").glob("*.log")
        assert len(log.read_text().splitlines()) == 500
    finally:
        os.close(master)
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=8)
