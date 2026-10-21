import ctypes
import functools
import multiprocessing as mp
import time

from elesim_protocol import DdsRuntimeSettings
from elesim_robot.camera.process import CameraPublisherProcess


class BlockingSdkCamera:
    def __init__(self, *, entered, **_kwargs):
        self.entered = entered

    def start(self):
        self.entered.set()
        # PyDLL holds this process's GIL, reproducing a vendor SDK wait.
        ctypes.PyDLL(None).sleep(15)

    def stop(self):
        pass


class NoPublisher:
    def __init__(self, *_args, **_kwargs):
        pass

    def close(self):
        pass


def test_vendor_sdk_gil_stall_cannot_block_parent_control_loop():
    entered = mp.get_context("spawn").Event()
    worker = CameraPublisherProcess(
        "/elesim/test/rgbd/frame", backend="zed", endpoint_id="camera-process-check", boot_id="test",
        dds_settings=DdsRuntimeSettings(), width=4, height=2, fps=30,
        camera_factory=functools.partial(BlockingSdkCamera, entered=entered),
        publisher_factory=NoPublisher,
    )
    try:
        worker.start()
        assert entered.wait(8), "camera child did not reach the simulated SDK call"
        started = time.monotonic()
        for _ in range(10):
            worker.status()
            time.sleep(.01)
        assert time.monotonic() - started < .5
    finally:
        stopping = time.monotonic()
        worker.stop()
        assert time.monotonic() - stopping < 5
    assert not worker._process.is_alive()
