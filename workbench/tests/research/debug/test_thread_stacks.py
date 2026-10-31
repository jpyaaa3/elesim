import faulthandler
import io
import threading
import time

import pytest

from workbench.research.support.thread_stacks import ThreadStackSampler


def test_periodic_stacks_capture_worker_without_the_native_watchdog(monkeypatch):
    def forbidden(*args,**kwargs):
        raise AssertionError('native watchdog must not be armed')
    monkeypatch.setattr(faulthandler,'dump_traceback_later',forbidden)
    stop = threading.Event()
    def held_worker(): stop.wait(2.)
    thread = threading.Thread(target=held_worker,name='owned-test-worker')
    output = io.StringIO()
    sampler = ThreadStackSampler(output,interval_s=.005)
    thread.start()
    sampler.start()
    try:
        deadline = time.monotonic()+2.
        while sampler.samples < 3 and time.monotonic()<deadline: time.sleep(.005)
        assert sampler.samples >= 3 and not sampler.error
    finally:
        sampler.close()
        stop.set()
        thread.join(2.)
    assert 'owned-test-worker' in output.getvalue() and 'held_worker' in output.getvalue()
    assert not sampler._thread.is_alive()


def test_blocked_writer_remains_owned_until_join_completes():
    entered,release = threading.Event(),threading.Event()
    class Sink:
        def write(self,text):
            entered.set()
            release.wait(2.)
        def flush(self): pass
    sampler = ThreadStackSampler(Sink(),interval_s=.001).start()
    assert entered.wait(2.)
    try:
        with pytest.raises(TimeoutError): sampler.close(timeout_s=.001)
        assert sampler._thread.is_alive()
    finally:
        release.set()
        sampler.close()
    assert not sampler._thread.is_alive()
