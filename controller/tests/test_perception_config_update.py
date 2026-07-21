from __future__ import annotations

from types import SimpleNamespace

from elesim_controller.config import PerceptionConfig
from elesim_controller.pick import ControlService, PanelState


def test_ui_perception_patch_preserves_controller_only_tracking_fields() -> None:
    baseline = PerceptionConfig(
        target_label="old",
        track_lost_frames=37,
        track_csrt_psr_threshold=0.123,
    )
    service = ControlService(PanelState(), perception_cfg=baseline)
    ui_patch = SimpleNamespace(
        enabled=True,
        detector_config="detector.json",
        mode="sim",
        detector="hsv",
        provider="local",
        preview_bind="tcp://127.0.0.1:5570",
        preview_endpoint="tcp://127.0.0.1:5570",
        preview_jpeg_quality=70,
        target_label="sim_sphere",
        yolo_device="",
        publish_hz=12.0,
        show_preview=False,
        pipeline="yolo_seg",
        tracker="csrt",
        run_local=True,
    )

    service.update_perception_config(ui_patch)

    assert isinstance(service._perception_cfg, PerceptionConfig)
    assert service._perception_cfg.target_label == "sim_sphere"
    assert service._perception_cfg.track_lost_frames == 37
    assert service._perception_cfg.track_csrt_psr_threshold == 0.123
