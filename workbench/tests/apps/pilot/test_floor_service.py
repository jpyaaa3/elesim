from __future__ import annotations

import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

from elesim_pilot.vision.floor_calibration import FloorSettings
from elesim_pilot.vision.floor_service import FloorCalibrationService
from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics


def frame(stamp=100.0, *, boot="boot", height=0.8):
    return SimpleNamespace(
        color_bgr=np.zeros((120, 160, 3), dtype=np.uint8),
        depth_raw=np.full((120, 160), height, dtype=np.float32), depth_scale=1.0,
        intrinsics=CameraIntrinsics(100, 100, 80, 60, 160, 120),
        ts=stamp, seq=int(stamp*100), source_id="sim", source_boot_id=boot, calibration_id="camera-v1",
        camera_world_origin=(0, 0, height), camera_world_look=(0, 0, -1), camera_world_right=(1, 0, 0))


def wait_processed(service, count):
    deadline = time.monotonic()+3
    while time.monotonic() < deadline:
        result = service.snapshot()
        if result["processed"] >= count:
            return result
        time.sleep(0.005)
    raise AssertionError(service.snapshot())


def test_real_frame_to_worker_reference_tracking_export_and_expiry(monkeypatch, tmp_path):
    clock = [100.0]
    service = FloorCalibrationService(max_hz=10, wall_clock=lambda: clock[0],
        monotonic=lambda: clock[0], settings=FloorSettings(stable_frames=1))
    monkeypatch.setattr("elesim_pilot.vision.floor_service.tempfile.mkdtemp", lambda **_: str(tmp_path))
    try:
        assert service.submit(frame())
        result = wait_processed(service, 1)
        assert result["state"] == "candidate"
        service.confirm(result["candidates"][0]["id"])
        clock[0] += 0.2
        assert service.submit(frame(clock[0]))
        result = wait_processed(service, 2)
        assert result["valid"] and result["correction_available"]
        assert result["processing_ms"] >= 0
        evidence = [json.loads(line) for line in Path(service.export()).read_text().splitlines()]
        assert len(evidence) == 2
        assert evidence[1]["identity"][:2] == ["sim", "boot"]
        assert evidence[1]["observed_planes"][0]["offset_m"] == pytest.approx(0.8, abs=1e-5)
        assert evidence[1]["predicted_camera_pose"][2][3] == 0.8
        clock[0] += 1
        assert service.snapshot()["state"] == "lost"
        assert not service.snapshot()["correction_available"]
    finally:
        service.close()
    assert not service.submit(frame(clock[0]))
    assert not service.snapshot()["correction_available"]
    assert not service._thread.is_alive()


def test_unverified_source_history_can_record_but_not_apply_visual_correction():
    f = frame()
    pose = np.diag([1., -1., -1., 1.])
    pose[2, 3] = 0.85
    service = FloorCalibrationService(reference_height_m=0.0, apply=True,
        pose_provider=lambda _: pose, wall_clock=lambda: 100,
        settings=FloorSettings(stable_frames=1))
    try:
        service.submit(f)
        result = wait_processed(service, 1)
        assert result["valid"]
        assert result["distance_error_m"] == pytest.approx(-0.05, abs=1e-5)
        assert not result["synchronization_verified"]
        assert service.corrected_world_point(f, [0, 0, 0.8]) is None
        service.clock_verified = True
        point = service.corrected_world_point(f, [0, 0, 0.8])
        np.testing.assert_allclose(point, [0, 0, 0], atol=1e-5)
        assert service.corrected_world_point(frame(100.0, boot="other"), [0, 0, 0.8]) is None
        assert service.corrected_world_point(frame(101.0), [0, 0, 0.8]) is None
    finally:
        service.close()


def test_optical_pose_and_floor_overlay_do_not_require_a_detector_or_motion_client():
    service = FloorCalibrationService(reference_height_m=0, wall_clock=lambda: 100,
                                      settings=FloorSettings(stable_frames=1))
    try:
        service.submit(frame())
        result = wait_processed(service, 1)
        assert result["valid"]
        assert not result["apply_requested"]
        assert service.corrected_world_point(frame(), [0, 0, 0.8]) is None
    finally:
        service.close()


def test_missing_pose_and_bad_capture_time_are_reported_without_fabricating_a_timestamp():
    f = frame(0)
    f.camera_world_origin = f.camera_world_look = f.camera_world_right = None
    service = FloorCalibrationService(wall_clock=lambda: 100)
    try:
        service.submit(f)
        result = wait_processed(service, 1)
        assert not result["valid"]
        assert result["reason"] == "invalid_or_stale_capture_time"
    finally:
        service.close()


def test_capture_adapter_preserves_source_time_identity_and_calibration():
    from elesim_pilot.vision.perception.sim_rendered_camera import SimRenderedCamera
    source = frame()
    source.arm_q = (0.1, 0.2, 0.3, 0.4)
    adapter = SimRenderedCamera()
    adapter._sub = SimpleNamespace(recv_latest=lambda **_: source)
    actual = adapter.capture(retries=1)
    assert (actual.ts, actual.seq, actual.source_id, actual.source_boot_id, actual.calibration_id) == (
        source.ts, source.seq, "sim", "boot", "camera-v1")
    assert actual.arm_q == source.arm_q


def test_decoding_encoded_depth_preserves_metric_data_and_provenance():
    from elesim_protocol.encoded_rgbd import encoded_frame_from_rgbd, rgbd_from_encoded_frame
    f = frame()
    encoded = encoded_frame_from_rgbd(f, source_id=f.source_id, source_boot_id=f.source_boot_id,
        calibration_id=f.calibration_id, color_codec="raw", depth_codec="zlib")
    actual = rgbd_from_encoded_frame(encoded)
    assert actual.ts == f.ts
    assert (actual.source_id, actual.source_boot_id, actual.calibration_id) == ("sim", "boot", "camera-v1")
    np.testing.assert_array_equal(actual.depth_raw, f.depth_raw)


def test_operator_floor_reference_operations_never_call_motion_commands():
    from elesim_pilot.operator import OperatorDispatcher
    from elesim_pilot.pick import ControlService, PanelState
    from elesim_protocol import decode_value, dumps_envelope, loads_envelope, make_envelope
    service = ControlService(PanelState())
    service._floor_calibration = FloorCalibrationService(reference_height_m=0, wall_clock=lambda: 100,
        settings=FloorSettings(stable_frames=1))
    observer = service._floor_calibration
    try:
        observer.submit(frame())
        status = wait_processed(observer, 1)
        dispatcher = OperatorDispatcher(service.state, service)
        request = {"request_id": "floor-confirm", "operation": "service_call",
            "name": "confirm_floor_calibration", "args": [status["candidates"][0]["id"]]}
        request = loads_envelope(dumps_envelope(make_envelope("operator_intent", "ui", target_id="pilot", payload=request)))
        result = dispatcher.handle(request.payload)
        response = loads_envelope(dumps_envelope(make_envelope("operator_result", "pilot", target_id="ui", payload=result)))
        result = response.payload
        assert result["ok"]
        assert decode_value(result["result"])["reference"]["kind"] == "initial_observation"
        result = dispatcher.handle({"request_id": "floor-reset", "operation": "service_call",
            "name": "reset_floor_calibration", "args": []})
        assert result["ok"]
        assert service.client is None
    finally:
        observer.close()


def test_preview_overlay_never_changes_original_rgb_or_depth():
    service = FloorCalibrationService(reference_height_m=0, wall_clock=lambda: 100,
                                      settings=FloorSettings(stable_frames=1))
    f = frame()
    color, depth = f.color_bgr.copy(), f.depth_raw.copy()
    try:
        service.submit(f)
        wait_processed(service, 1)
        overlay = service.overlay(f, f.color_bgr)
        assert np.any(overlay != color)
        np.testing.assert_array_equal(f.color_bgr, color)
        np.testing.assert_array_equal(f.depth_raw, depth)
    finally:
        service.close()


def test_reset_during_fit_discards_old_epoch_instead_of_restoring_reference(monkeypatch):
    import threading
    from elesim_pilot.vision import floor_service
    started, release = threading.Event(), threading.Event()
    original = floor_service.detect_planes
    def delayed(*args):
        started.set()
        assert release.wait(2)
        return original(*args)
    monkeypatch.setattr(floor_service, "detect_planes", delayed)
    service = FloorCalibrationService(reference_height_m=0, wall_clock=lambda: 100,
                                      settings=FloorSettings(stable_frames=1))
    try:
        service.submit(frame())
        assert started.wait(2)
        service.reset()
        release.set()
    finally:
        release.set()
        service.close()
    assert service.snapshot()["processed"] == 0
    assert not service.snapshot()["valid"]


def test_source_pose_pairing_interpolates_source_clock_not_arrival_time(monkeypatch):
    from elesim_pilot.pick import ControlService, PanelState
    from elesim_pilot.pick import perception
    service = ControlService(PanelState(), hand_eye_transform=np.eye(4), ik_context={"test": True})
    def fk(_context, q, _mount, *, parent_frame):
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        return pose
    monkeypatch.setattr(perception, "camera_world_transform", fk)
    for stamp, x in [(100.0, 0.0), (100.1, 0.1)]:
        service.accept_floor_telemetry({"measurement_time_s": stamp, "q_source": "measured",
            "q": [x, 0.2, 0.8, 0.0], "imu_model": {"id": "identity", "version": 1}},
            source_id="sim", source_boot_id="boot")
    pose = service._floor_camera_pose(frame(100.05))
    np.testing.assert_allclose(pose[:3, 3], [0.05, 0.2, 0.8])
    evidence = service._floor_pose_evidence(frame(100.05))
    assert [s["measurement_time_s"] for s in evidence["samples"]] == [100, 100.1]
    assert evidence["samples"][0]["imu_model"]["id"] == "identity"
    assert service._floor_camera_pose(frame(100.05, boot="other")) is None


def test_saved_float_depth_preserves_submetre_values_and_calibration_metadata(tmp_path):
    from elesim_pilot.vision.perception.capture import save_perception_frame_bundle
    f = frame()
    save_perception_frame_bundle(out_dir=tmp_path, color_bgr=f.color_bgr,
        depth_raw=f.depth_raw, stem="float", meta={"depth_scale": f.depth_scale, "capture_time_s": f.ts})
    raw = np.load(tmp_path / "float_depth.npz", allow_pickle=False)
    np.testing.assert_array_equal(raw["depth"], f.depth_raw)
    assert json.loads((tmp_path / "float_meta.json").read_text())["depth_scale"] == 1


@pytest.mark.parametrize("operation", ["confirm", "reset", "configure"])
def test_active_pick_cannot_change_floor_reference_or_coordinate_interpretation(operation):
    from elesim_pilot.pick import ControlService, PanelState
    service = ControlService(PanelState())
    service.state.pick_running = True
    original = service._perception_cfg
    with pytest.raises(RuntimeError, match="stop active"):
        if operation == "confirm":
            service.confirm_floor_calibration("plane-1")
        elif operation == "reset":
            service.reset_floor_calibration()
        else:
            service.update_perception_config({"floor_calibration_apply": True})
    assert service._perception_cfg is original
