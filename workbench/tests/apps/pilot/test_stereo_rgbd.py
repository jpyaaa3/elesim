from __future__ import annotations

from pathlib import Path

import yaml

from elesim_pilot.config import PerceptionConfig, load_runtime_role_config
from elesim_pilot.main import _ControlFacade
from elesim_protocol import (
    DdsRuntimeSettings,
    MEDIA_KIND_RGBD,
    MEDIA_SECURITY_NONE,
    MEDIA_TRANSPORT_DDS,
    MediaStreamDescriptor,
)


ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "payload").is_dir())


def test_pilot_config_derives_right_broker_topic_for_older_encoded_configs(tmp_path):
    raw = yaml.safe_load((ROOT / "payload/config/pilot/runtime.yaml").read_text())
    raw["rgbd"]["wire"].pop("right_topic", None)
    path = tmp_path / "pilot-runtime.yaml"
    path.write_text(yaml.safe_dump(raw))

    config = load_runtime_role_config(path)

    assert config.rgbd["wire"]["right_topic"] == "/elesim/pilot_main/rgbd/right"


def test_pilot_relays_left_depth_and_right_stereo_topics(monkeypatch, tmp_path):
    created = []

    class Relay:
        def __init__(self, source, broker, **kwargs):
            self.source = source
            self.broker = broker
            self.kwargs = kwargs
            self.started = False
            self.closed = False
            created.append(self)

        def start(self):
            self.started = True

        def close(self):
            self.closed = True

    class Service:
        def __init__(self):
            self._perception_cfg = PerceptionConfig()

        def update_perception_config(self, config):
            self._perception_cfg = config

    class Connection:
        pilot_id = "pilot-main"

    monkeypatch.setattr("elesim_pilot.main.DdsRgbdRelay", Relay)
    monkeypatch.setattr("elesim_pilot.main.ImuModelCatalog", lambda *_args: object())
    def stream(topic):
        return MediaStreamDescriptor(
            transport=MEDIA_TRANSPORT_DDS,
            media_kind=MEDIA_KIND_RGBD,
            endpoint=topic,
            security=MEDIA_SECURITY_NONE,
            format="encoded-rgbd-v1",
        ).to_dict()
    facade = _ControlFacade(
        Service(),
        Connection(),
        dds_settings=DdsRuntimeSettings(),
        imu_model_directory=tmp_path,
        rgbd_broker_topic="/elesim/pilot_main/rgbd/frame",
        rgbd_right_broker_topic="/elesim/pilot_main/rgbd/right",
        rgbd_left_preview_topic="/elesim/pilot_main/rgbd/left_preview",
    )

    facade.configure_target_stream(
        {
            "endpoint_id": "robot-go2",
            "instance_id": "robot-boot",
            "streams": {
                "rgbd": stream("/elesim/robot_go2/rgbd/frame"),
                "rgbd_right": stream("/elesim/robot_go2/rgbd/right"),
            },
        }
    )

    assert [(relay.source, relay.broker) for relay in created] == [
        ("/elesim/robot_go2/rgbd/frame", "/elesim/pilot_main/rgbd/frame"),
        ("/elesim/robot_go2/rgbd/frame", "/elesim/pilot_main/rgbd/left_preview"),
        ("/elesim/robot_go2/rgbd/right", "/elesim/pilot_main/rgbd/right"),
    ]
    assert created[0].kwargs.get("strip_depth", False) is False
    assert created[1].kwargs["strip_depth"] is True
    assert all(relay.started for relay in created)
    facade.close()
    assert all(relay.closed for relay in created)
