from pathlib import Path

import pytest

from elesim_setup.host_helper import HostHelperError, _validate_command


def _paths() -> tuple[Path, Path]:
    return Path("/opt/elesim/containers/compose.yaml"), Path("/opt/elesim/bin")


def test_host_helper_allows_only_fixed_compose_lifecycle_shapes() -> None:
    compose, bin_dir = _paths()
    prefix = (
        "docker",
        "compose",
        "-p",
        "elesim-runtime",
        "-f",
        str(compose),
    )
    for suffix in (
        ("config", "--quiet"),
        ("ps", "--status", "running", "--services"),
        ("build", "pilot", "ui"),
        ("stop", "sim"),
        ("start", "pilot"),
        ("up", "-d", "--no-build", "--remove-orphans"),
    ):
        _validate_command(
            (*prefix, *suffix),
            compose=compose,
            bin_dir=bin_dir,
            project="elesim-runtime",
        )


@pytest.mark.parametrize(
    "argv",
    (
        ("docker", "run", "--privileged", "alpine"),
        (
            "docker",
            "compose",
            "-p",
            "other",
            "-f",
            "/opt/elesim/containers/compose.yaml",
            "stop",
            "pilot",
        ),
        (
            "docker",
            "compose",
            "-p",
            "elesim-runtime",
            "-f",
            "/tmp/compose.yaml",
            "build",
            "pilot",
        ),
        (
            "docker",
            "compose",
            "-p",
            "elesim-runtime",
            "-f",
            "/opt/elesim/containers/compose.yaml",
            "build",
            "manager",
        ),
        (
            "docker",
            "compose",
            "-p",
            "elesim-runtime",
            "-f",
            "/opt/elesim/containers/compose.yaml",
            "down",
        ),
    ),
)
def test_host_helper_rejects_daemon_escape_shapes(argv: tuple[str, ...]) -> None:
    compose, bin_dir = _paths()
    with pytest.raises(HostHelperError):
        _validate_command(
            argv,
            compose=compose,
            bin_dir=bin_dir,
            project="elesim-runtime",
        )


def test_host_helper_limits_network_cli_to_installed_wrapper() -> None:
    compose, bin_dir = _paths()
    _validate_command(
        (str(bin_dir / "elesim-net"), "show"),
        compose=compose,
        bin_dir=bin_dir,
        project="elesim-runtime",
    )
    with pytest.raises(HostHelperError):
        _validate_command(
            ("/tmp/elesim-net", "show"),
            compose=compose,
            bin_dir=bin_dir,
            project="elesim-runtime",
        )
    with pytest.raises(HostHelperError):
        _validate_command(
            (str(bin_dir / "elesim-net"), "uninstall"),
            compose=compose,
            bin_dir=bin_dir,
            project="elesim-runtime",
        )
