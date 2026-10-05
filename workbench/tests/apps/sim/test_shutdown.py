import signal

import pytest

from elesim_sim.shutdown import graceful_sigterm


def test_sigterm_unwinds_cleanup_and_restores_previous_handler():
    previous = signal.getsignal(signal.SIGTERM)
    cleaned = []
    with pytest.raises(SystemExit) as result:
        with graceful_sigterm():
            try:
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
            finally:
                cleaned.append(True)
    assert result.value.code == 0
    assert cleaned == [True]
    assert signal.getsignal(signal.SIGTERM) == previous


def test_normal_exit_restores_sigterm_handler():
    previous = signal.getsignal(signal.SIGTERM)
    with graceful_sigterm():
        assert callable(signal.getsignal(signal.SIGTERM))
    assert signal.getsignal(signal.SIGTERM) == previous
