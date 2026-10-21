import sys
from types import SimpleNamespace

import numpy as np
import pytest

from elesim_robot.camera.zed import ZedMiniCamera


def sdk(monkeypatch, *, available=True):
    frames = []
    class Mat:
        def get_data(self):
            return self.data

    class Camera:
        closed = False
        @staticmethod
        def get_device_list():
            return [SimpleNamespace(camera_model="mini", camera_state="available" if available else "unavailable", serial_number=123 if available else 0)]

        def open(self, params):
            assert params.serial == 123
            return 0

        def get_camera_information(self, resolution):
            assert (resolution.width, resolution.height) == (4, 2)
            return SimpleNamespace(camera_configuration=SimpleNamespace(
                calibration_parameters=SimpleNamespace(
                    left_cam=SimpleNamespace(fx=100, fy=110, cx=1, cy=.5),
                    right_cam=SimpleNamespace(fx=101, fy=111, cx=2, cy=.6),
                ),
            ))

        def grab(self, _runtime):
            return 0

        def retrieve_image(self, mat, view, memory, resolution):
            assert view in {"left", "right"} and memory == "cpu"
            value = 10 if view == "left" else 40
            mat.data = np.full((2, 4, 4), [value, 20, 30, 255], dtype=np.uint8)
            frames.append(mat)
            return 0

        def retrieve_measure(self, mat, measure, memory, resolution):
            assert measure == "depth" and memory == "cpu"
            mat.data = np.array([[1000.2, np.nan, np.inf, -1], [1234.6, 65536, 0, 2000]], dtype=np.float32)
            return 0

        def close(self):
            self.closed = True

    class InitParameters:
        def set_from_serial_number(self, serial):
            self.serial = serial

    sl = SimpleNamespace(Camera=Camera, Mat=Mat, InitParameters=InitParameters,
        RuntimeParameters=lambda: object(), Resolution=lambda w,h: SimpleNamespace(width=w,height=h),
        MODEL=SimpleNamespace(ZED_M="mini"), CAMERA_STATE=SimpleNamespace(AVAILABLE="available"),
        RESOLUTION=SimpleNamespace(HD720=1), UNIT=SimpleNamespace(MILLIMETER=1),
        DEPTH_MODE=SimpleNamespace(QUALITY=1), ERROR_CODE=SimpleNamespace(SUCCESS=0),
        VIEW=SimpleNamespace(LEFT="left", RIGHT="right"),
        MEASURE=SimpleNamespace(DEPTH="depth"), MEM=SimpleNamespace(CPU="cpu"))
    monkeypatch.setitem(sys.modules, "pyzed", SimpleNamespace(sl=sl))
    monkeypatch.setitem(sys.modules, "pyzed.sl", sl)
    return sl, frames


def test_zed_stereo_rgbd_has_both_images_intrinsics_metric_depth_and_owned_color(monkeypatch):
    _sl, images = sdk(monkeypatch)
    camera = ZedMiniCamera(color_width=4, color_height=2)
    camera.start()
    frame = camera.capture()
    assert frame.color_bgr.shape == (2, 4, 3)
    assert list(frame.color_bgr[0, 0]) == [10, 20, 30]
    assert frame.right_color_bgr.shape == (2, 4, 3)
    assert list(frame.right_color_bgr[0, 0]) == [40, 20, 30]
    images[0].data[:] = 0
    assert frame.color_bgr[0, 0, 0] == 10
    np.testing.assert_array_equal(frame.depth_raw, [[1000, 0, 0, 0], [1235, 0, 0, 2000]])
    assert frame.depth_raw.dtype == np.uint16 and frame.depth_scale == .001
    assert (frame.intrinsics.fx, frame.intrinsics.fy, frame.intrinsics.width, frame.intrinsics.height) == (100, 110, 4, 2)
    assert (frame.right_intrinsics.fx, frame.right_intrinsics.fy) == (101, 111)
    device = camera._camera
    camera.stop()
    assert device.closed


def test_zed_hid_only_unavailable_device_does_not_attempt_open(monkeypatch):
    sl, _images = sdk(monkeypatch, available=False)
    monkeypatch.setattr(sl.Camera, "open", lambda *_args: pytest.fail("unavailable device must not be opened"))
    with pytest.raises(RuntimeError, match="No ZED Mini video device"):
        ZedMiniCamera().start()
