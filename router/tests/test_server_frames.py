from __future__ import annotations

import subprocess
import sys

import pytest

from elesim_protocol import ProtocolError
from elesim_router.main import decode_router_frames


@pytest.mark.parametrize("frames", ([], [b"identity"], [b"a", b"b", b"c"]))
def test_malformed_router_multipart_is_rejected_without_unpacking_crash(frames: list[bytes]) -> None:
    with pytest.raises(ProtocolError, match="exactly two frames"):
        decode_router_frames(frames)


def test_module_entrypoint_does_not_preload_itself() -> None:
    completed = subprocess.run(
        [sys.executable, "-W", "error::RuntimeWarning", "-m", "elesim_router.main", "--help"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
