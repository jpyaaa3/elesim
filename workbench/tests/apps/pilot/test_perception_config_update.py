from __future__ import annotations

from types import SimpleNamespace

import pytest

from elesim_pilot.config import PerceptionConfig
from elesim_pilot.pick import ControlService, PanelState


@pytest.mark.parametrize("patch", [{"provider": "host"}, {"run_local": False}])
def test_unsupported_remote_update_preserves_current_config(patch) -> None:
    baseline = PerceptionConfig(target_label="keep")
    state = PanelState()
    service = ControlService(state, perception_cfg=baseline)
    previous_label = state.visual_target_label
    with pytest.raises(ValueError, match="Pilot-owned local perception"):
        service.update_perception_config({"target_label": "discard", **patch})
    assert service._perception_cfg is baseline
    assert state.visual_target_label == previous_label


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
