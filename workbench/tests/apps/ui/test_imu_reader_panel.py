from types import SimpleNamespace as NS

from elesim_ui.panels import imu_models


def test_reader_values_visible_even_without_model_catalog(monkeypatch):
    rendered = []
    monkeypatch.setattr(imu_models, "panel_header", lambda *_a, **_k: (True,))
    monkeypatch.setattr(imu_models, "imgui", NS(text=rendered.append,
                                               text_wrapped=rendered.append))
    samples = {
        "IMU1": {"sequence": 3, "fresh": False, "received_age_s": 2.0, "rpy_rad": [1, 2, 3]},
        "IMU2": {"sequence": None, "fresh": False, "received_age_s": .01, "rpy_rad": [0, 0, 0]},
    }
    panel = NS(_imu_model_header_init_open=True, service=NS(imu_models=[], imu_model_status={
        "reader": {"connected": True, "sensors": samples, "error": ""},
    }))
    imu_models.draw_imu_model_panel(panel)
    assert any("IMU1: stale" in text for text in rendered)
    assert any("IMU2: legacy" in text for text in rendered)
    assert any("+1.000" in text for text in rendered)


def test_full_reader_panel_handles_initial_zero_quaternion_and_shows_vectors(monkeypatch):
    rendered = []
    monkeypatch.setattr(imu_models, "panel_header", lambda *_a, **_k: (True,))
    monkeypatch.setattr(imu_models, "imgui", NS(text=rendered.append, text_wrapped=rendered.append))
    panel = NS(_imu_model_header_init_open=True, service=NS(imu_models=[], imu_model_status={
        "reader": {"connected": True, "sensors": {"IMU1": {
            "source_format": "full-imu-v1", "sequence": 3, "fresh": False,
            "reports_recent": True, "received_age_s": .02, "rpy_rad": None,
            "full_imu": {"gyro": [1, 2, 3]},
        }}},
    }))
    imu_models.draw_imu_model_panel(panel)
    assert any("orientation freshness unverified" in text for text in rendered)
    assert any("Reports: recent" in text for text in rendered)
    assert any("waiting for a valid quaternion" in text for text in rendered)
    assert any("gyro: +1.000" in text for text in rendered)
