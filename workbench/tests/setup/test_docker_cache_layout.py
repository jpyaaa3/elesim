"""Ordering/ shell checks; real BuildKit cache hits require a live daemon."""
from pathlib import Path
import os
import re
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3] / "payload/runtime/docker"
FILES = ("shared/Dockerfile.app", "setup/Dockerfile", "dev/Dockerfile",
         "pilot/Dockerfile.release", "sim/Dockerfile.release", "ui/Dockerfile.release")


def run_commands(text):
    return [re.sub(r"^RUN\s+(?:--mount=\S+\s+)*", "", line)
            for line in text.replace("\\\n", " ").splitlines()
            if line.startswith("RUN ")]


def docker_stages(text):
    """Inspect independent BuildKit stages without mixing their COPY order."""
    return re.findall(r"(?ms)^FROM ([^\n]+)\n(.*?)(?=^FROM |\Z)", text)


@pytest.mark.parametrize("name", FILES)
def test_pip_runs_use_external_cache_and_valid_shell(name):
    text = (ROOT / name).read_text()
    assert text.startswith("# syntax=docker/dockerfile:1\n")
    assert "--no-cache-dir" not in text
    assert "pip cache purge" not in text
    for instruction in text.replace("\\\n", " ").splitlines():
        if not instruction.startswith("RUN "):
            continue
        if "pip install" in instruction:
            target = "/var/lib/elesim" if name.startswith("shared/") else "/root"
            assert f"--mount=type=cache,target={target}/.cache/pip,sharing=locked" in instruction
        shell = re.sub(r"^RUN\s+(?:--mount=\S+\s+)*", "", instruction)
        result = subprocess.run(["sh", "-n"], input=shell, text=True, capture_output=True)
        assert result.returncode == 0, result.stderr


def test_runtime_source_does_not_invalidate_external_dependencies():
    text = docker_stages((ROOT / "shared/Dockerfile.app").read_text())[-1][1]
    assert text.index("cmake --install") < text.index("ARG COMPUTE_MODE")
    assert text.index('"torch==$torch_version"') < text.index("COPY requirements.lock")
    assert text.index("COPY requirements.lock") < text.index("ARG INSTALL_GO2_MPC")
    assert text.index("go2-convex-mpc.git@") < text.index("COPY interfaces/")
    assert text.index('pip install "setuptools') < text.index("COPY app/")
    assert "pip install --no-deps" in text.split("COPY app/", 1)[1]


@pytest.mark.parametrize("failure", [
    "apt-get update", "apt-get install", "git clone", "commit-check",
    "cmake -S", "cmake --build", "cmake --install", "ldconfig", "python -c",
])
def test_dependency_failure_cannot_be_hidden_by_cleanup(tmp_path, failure):
    text = (ROOT / "shared/Dockerfile.app").read_text()
    commands = run_commands(text)
    command = next(c for c in commands if
                   ("robotpkg-py310-pinocchio" if failure.startswith("apt-get")
                    else "git clone") in c)
    # All external commands are test doubles. Do not run apt/cmake/git/rm or
    # write to /etc on the host when exercising the generated shell body.
    command = command.replace("/etc/apt/sources.list.d/robotpkg.list", str(tmp_path / "robotpkg.list"))
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    driver = fake_bin / "driver"
    driver.write_text(f"#!{sys.executable}\n" + '''
import os, sys
from pathlib import Path
name = Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ['COMMAND_LOG'], 'a') as log:
    log.write(name + ' ' + ' '.join(args) + '\\n')
failure = os.environ['FAIL_COMMAND']
if (name + ' ' + ' '.join(args)).startswith(failure):
    sys.exit(23)
if name == 'dpkg':
    print('amd64')
elif name == 'git' and args[0] == '-C':
    print('wrong' if failure == 'commit-check' else os.environ['CASADI_GIT_COMMIT'])
''')
    driver.chmod(0o755)
    for name in ("apt-get", "git", "cmake", "ldconfig", "python", "rm", "dpkg"):
        (fake_bin / name).symlink_to(driver)
    log = tmp_path / "commands.log"
    result = subprocess.run(
        ["/bin/sh", "-c", command], capture_output=True, text=True,
        env={**os.environ, "PATH": str(fake_bin), "ROLE": "sim",
             "FAIL_COMMAND": failure, "COMMAND_LOG": str(log),
             "CASADI_GIT_REF": "3.7.2", "CASADI_GIT_COMMIT": "expected",
             "OSQP_GIT_REF": "v0.6.3", "CASADI_BUILD_JOBS": "4"},
    )
    assert result.returncode == (1 if failure == "commit-check" else 23), result.stdout + result.stderr
    assert not any(line.startswith("rm ") for line in log.read_text().splitlines())


def test_developer_does_not_silently_ignore_rosdep_failures():
    # No repository workflow invokes rosdep; initialization is not required
    # for the explicitly apt/pip-installed development stack.
    assert "rosdep init" not in (ROOT / "dev/Dockerfile").read_text()


def test_tools_source_and_interfaces_do_not_invalidate_python_dependencies():
    text = (ROOT / "setup/Dockerfile").read_text()
    assert text.index("python3-cffi python3-cryptography") < text.index("COPY interfaces/")
    assert text.index("COPY interfaces/") < text.index("COPY app/ /")


@pytest.mark.parametrize("role", ("pilot", "sim", "ui"))
def test_release_dependencies_precede_wheels_and_runtime_data(role):
    text = docker_stages((ROOT / role / "Dockerfile.release").read_text())[-1][1]
    deps = text.index("pip install -r")
    markers = ["COPY wheels/", "COPY config/", "ARG APP_WHEEL", "ARG PROTOCOL_WHEEL"]
    if role in {"pilot", "sim"}:
        markers.append("COPY data/")
    else:
        assert "COPY data/" not in text
    for marker in markers:
        assert deps < text.index(marker)
    assert "python3 -m pip check" in text


@pytest.mark.parametrize("name,source", [
    ("shared/Dockerfile.app", "COPY app/"),
    ("pilot/Dockerfile.release", "COPY wheels/"),
])
def test_see_native_builder_keeps_its_own_dependency_cache(name, source):
    stages = docker_stages((ROOT / name).read_text())
    builder = next(body for header, body in stages if header.endswith(" AS see_builder"))
    runtime = stages[-1][1]
    assert builder.index("libpcl-dev") < builder.index(source) < builder.index("cmake -S")
    assert builder.index("cmake -S") < builder.index("cmake --build")
    assert "--mount=type=cache,target=/var/cache/apt,sharing=locked" in builder
    assert "Acquire::Retries=3" in builder and "Acquire::http::Timeout=30" in builder
    assert "libpcl-dev" not in runtime
    assert runtime.index("pip install -r") < runtime.index("COPY --from=see_builder")


@pytest.mark.parametrize("name", ["shared/Dockerfile.app", "pilot/Dockerfile.release"])
def test_native_builder_fetches_ubuntu_over_https_only_in_its_image(name, tmp_path):
    stages = docker_stages((ROOT / name).read_text())
    builder = next(body for header, body in stages if header.endswith(" AS see_builder"))
    command = run_commands(builder)[0]
    source = tmp_path / 'sources.list'
    source.write_text('deb http://archive.ubuntu.com/ubuntu jammy main\n'
        'deb http://security.ubuntu.com/ubuntu jammy-security main\n'
        'deb http://packages.ros.org/ros2/ubuntu jammy main\n')
    for original, replacement in [('/etc/apt/sources.list', source),
            ('/etc/apt/apt.conf.d/docker-clean', tmp_path/'docker-clean'),
            ('/var/lib/apt/lists', tmp_path/'lists'),
            ('/opt/elesim/native', tmp_path/'native')]:
        command = command.replace(original, str(replacement))
    assert '/etc/' not in command and '/opt/' not in command and '/var/lib/' not in command
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    apt = fake_bin / 'apt-get'
    apt.write_text('#!/bin/sh\nexit 0\n')
    apt.chmod(0o755)
    result = subprocess.run(['/bin/sh', '-c', command], text=True, capture_output=True,
        env={**os.environ, 'ROLE':'pilot', 'PATH':str(fake_bin)+os.pathsep+os.defpath})
    assert result.returncode == 0, result.stderr
    assert source.read_text() == ('deb https://archive.ubuntu.com/ubuntu jammy main\n'
        'deb https://security.ubuntu.com/ubuntu jammy-security main\n'
        'deb http://packages.ros.org/ros2/ubuntu jammy main\n')


def test_developer_identity_does_not_invalidate_dependencies():
    text = (ROOT / "dev/Dockerfile").read_text()
    for arg in ("USERNAME", "UID", "GID"):
        assert text.index("python3-cffi python3-cryptography") < text.index(f"ARG {arg}=")
    assert text.index("cmake --install") < text.index("ARG COMPUTE_MODE")


def test_developer_optional_mpc_matches_runtime_build_switch():
    text = (ROOT / "dev/Dockerfile").read_text()
    assert "ARG INSTALL_GO2_MPC=1" in text
    assert 'if [ "$INSTALL_GO2_MPC" = 1 ]; then' in text
    assert "go2-convex-mpc.git@" in text
    assert "COPY runtime-contract/ /opt/elesim/runtime-contract/" in text
