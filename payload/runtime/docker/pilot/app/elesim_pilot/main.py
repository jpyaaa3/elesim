#!/usr/bin/env python3
"""Laptop-side owner of IK, perception and Pick/Gaze workflows."""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path

from elesim_protocol import (
    CAPABILITY_SIM_MOCK_HUG,
    DdsRuntimeSettings,
    MEDIA_KIND_RGBD,
    MEDIA_TRANSPORT_DDS,
    MediaStreamDescriptor,
    SimQ,
)

from elesim_pilot.connection import PilotConnection
from elesim_pilot.arm_trajectory import ArmTrajectory
from elesim_pilot.imu_models import ImuModelCatalog
from elesim_pilot.operator import OperatorDispatcher
from elesim_pilot.runtime import build_control_runtime
from elesim_pilot.config import load_app_config, load_runtime_role_config
from elesim_pilot.pick import ControlClient
from elesim_pilot.pick.mock_hug import MockHugCoordinator
from elesim_protocol.tracing import configure_tracing, shutdown_tracing, span
from elesim_pilot.simulation_sync import SimulationWorkflowSync
from elesim_pilot.shutdown import graceful_sigterm
from elesim_pilot.vision.rgbd import DdsRgbdRelay
from elesim_pilot.reconstruction.live_scan import LiveScan
from elesim_pilot.reconstruction.artifacts import default_scene_output_root


def _default_config_root() -> Path:
    for root in Path(__file__).resolve().parents:
        candidate = root / "payload/config/pilot"
        if candidate.is_dir():
            return candidate
    return Path("/opt/elesim/config")


_CONFIG_ROOT = _default_config_root()


class _ControlFacade:
    def __init__(
        self,
        service,
        connection: PilotConnection,
        *,
        dds_settings: DdsRuntimeSettings,
        imu_model_directory: Path | None = None,
        rgbd_broker_topic: str = "",
        rgbd_right_broker_topic: str = "",
        rgbd_left_preview_topic: str = "",
    ) -> None:
        self._service = service
        self._connection = connection
        self._dds_settings = dds_settings
        self._imu_models = ImuModelCatalog(imu_model_directory or (_CONFIG_ROOT / "imu_models"))
        self._rgbd_broker_topic = str(rgbd_broker_topic).strip()
        self._rgbd_right_broker_topic = str(rgbd_right_broker_topic).strip()
        self._rgbd_left_preview_topic = str(rgbd_left_preview_topic).strip()
        self._rgbd_relay: DdsRgbdRelay | None = None
        self._rgbd_right_relay: DdsRgbdRelay | None = None
        self._rgbd_left_preview_relay: DdsRgbdRelay | None = None
        self._mock_hug: MockHugCoordinator | None = None
        self._arm_trajectory: ArmTrajectory | None = None
        self._object_scan: LiveScan | None = None

    @property
    def object_scan(self):
        return {} if self._object_scan is None else self._object_scan.status()

    def start_object_scan(self, target_id='target', max_views=12):
        if self._object_scan is None:
            raise RuntimeError('object scan is not initialized')
        return self._object_scan.start(target_id, max_views)

    def cancel_object_scan(self):
        return self._object_scan.cancel()

    @property
    def arm_trajectory(self):
        return {} if self._arm_trajectory is None else self._arm_trajectory.status()

    def guard_operator_intent(self, operation, name):
        if self._object_scan is not None:
            self._object_scan.guard_operator(operation, name)
        if self._arm_trajectory is not None:
            self._arm_trajectory.guard_operator(operation, name)

    def plan_arm_to_target(self):
        return self._arm_trajectory.plan_arm_to_target()

    def execute_arm_trajectory(self, plan_id):
        return self._arm_trajectory.execute_arm_trajectory(plan_id)

    def cancel_arm_trajectory(self):
        return self._arm_trajectory.cancel_arm_trajectory()

    def attach_mock_hug(self, coordinator: MockHugCoordinator) -> None:
        self._mock_hug = coordinator

    def compute_mock_hug(self) -> dict[str, object]:
        if self._mock_hug is None:
            raise RuntimeError("mock hug coordinator is not initialized")
        self._require_sim_target()
        return self._mock_hug.compute()

    def execute_mock_hug(self, solution_id: str) -> dict[str, object]:
        if self._mock_hug is None:
            raise RuntimeError("mock hug coordinator is not initialized")
        self._require_sim_target()
        return self._mock_hug.execute(solution_id)

    def _require_sim_target(self) -> None:
        self.mock_hug_execution_context()

    def mock_hug_execution_context(self) -> tuple[str, str, str]:
        target = str(self._connection.active_target)
        descriptor = next(
            (
                value
                for value in self._connection.endpoints
                if str(value.get("endpoint_id", "")) == target
            ),
            None,
        )
        capabilities = () if descriptor is None else tuple(descriptor.get("capabilities", ()))
        boot_id = "" if descriptor is None else str(descriptor.get("instance_id", ""))
        lease_id = str(self._connection.lease_id)
        if (
            not target
            or descriptor is None
            or str(descriptor.get("role", "")) != "sim"
            or CAPABILITY_SIM_MOCK_HUG not in capabilities
            or not boot_id
            or not lease_id
        ):
            raise RuntimeError("mock hug requires a capable exact Sim boot and motion lease")
        return target, boot_id, lease_id

    def __getattr__(self, name):
        return getattr(self._service, name)

    def select_endpoint(self, target_id: str) -> None:
        self._connection.select_target(target_id)

    @property
    def imu_models(self) -> list[dict[str, object]]:
        return self._imu_models.list_models()

    def imu_model_status(self) -> dict[str, object]:
        return self._connection.imu_model_status()

    def select_imu_model(self, model_id: str) -> dict[str, object]:
        model = self._imu_models.get(model_id)
        return self._connection.select_imu_model(model.command_payload())

    def configure_target_stream(self, descriptor: dict) -> None:
        self._close_rgbd_relays()
        streams = descriptor.get("streams", {})
        raw_stream = streams.get("rgbd") if isinstance(streams, dict) else None
        if not isinstance(raw_stream, dict):
            return
        stream = MediaStreamDescriptor.from_dict(raw_stream)
        if stream.transport != MEDIA_TRANSPORT_DDS or stream.media_kind != MEDIA_KIND_RGBD:
            raise ValueError("target rgbd stream must be a DDS RGB-D topic")
        source_topic = str(stream.endpoint)
        broker_topic = self._rgbd_broker_topic or source_topic
        source_format = str(stream.format or "raw-rgbd-v1")
        consumer_format = source_format
        if source_topic != broker_topic:
            self._rgbd_relay = DdsRgbdRelay(
                source_topic,
                broker_topic,
                source_format=source_format,
                endpoint_id=self._connection.pilot_id,
                settings=self._dds_settings,
                expected_source_id=str(descriptor.get("endpoint_id", "")),
                expected_boot_id=str(descriptor.get("instance_id", "")),
            )
            self._rgbd_relay.start()
            consumer_format = "encoded-rgbd-v1"
            print(
                f"[pilot_agent] RGBD relay source={source_topic} broker={broker_topic} "
                f"format={source_format}"
            )
        if self._rgbd_left_preview_topic and source_topic != self._rgbd_left_preview_topic:
            self._rgbd_left_preview_relay = DdsRgbdRelay(
                source_topic,
                self._rgbd_left_preview_topic,
                source_format=source_format,
                endpoint_id=f"{self._connection.pilot_id}-rgbd-left-preview",
                settings=self._dds_settings,
                expected_source_id=str(descriptor.get("endpoint_id", "")),
                expected_boot_id=str(descriptor.get("instance_id", "")),
                strip_depth=True,
            )
            self._rgbd_left_preview_relay.start()
            print(
                f"[pilot_agent] RGB-D left preview source={source_topic} "
                f"broker={self._rgbd_left_preview_topic} depth=omitted"
            )
        raw_right_stream = streams.get("rgbd_right")
        if isinstance(raw_right_stream, dict):
            right_stream = MediaStreamDescriptor.from_dict(raw_right_stream)
            if (
                right_stream.transport != MEDIA_TRANSPORT_DDS
                or right_stream.media_kind != MEDIA_KIND_RGBD
            ):
                raise ValueError("target rgbd_right stream must be a DDS RGB-D topic")
            right_source_topic = str(right_stream.endpoint)
            right_broker_topic = self._rgbd_right_broker_topic or right_source_topic
            right_format = str(right_stream.format or "raw-rgbd-v1")
            if right_source_topic != right_broker_topic:
                self._rgbd_right_relay = DdsRgbdRelay(
                    right_source_topic,
                    right_broker_topic,
                    source_format=right_format,
                    endpoint_id=f"{self._connection.pilot_id}-rgbd-right",
                    settings=self._dds_settings,
                    expected_source_id=str(descriptor.get("endpoint_id", "")),
                    expected_boot_id=str(descriptor.get("instance_id", "")),
                )
                self._rgbd_right_relay.start()
                print(
                    f"[pilot_agent] RGB-D right relay source={right_source_topic} "
                    f"broker={right_broker_topic} format={right_format}"
                )
        current = self._service._perception_cfg
        updated = replace(
            current,
            mode="sim",
            provider="local",
            run_local=True,
            sim_camera_topic=broker_topic,
            sim_camera_dds_settings=self._dds_settings,
            sim_camera_source_id=str(descriptor.get("endpoint_id", "")),
            sim_camera_source_boot_id=str(descriptor.get("instance_id", "")),
            sim_camera_wire_format=consumer_format,
        )
        self._service.update_perception_config(updated)
        print(
            f"[pilot_agent] RGBD source={stream.endpoint} "
            f"target={descriptor.get('endpoint_id', '')} transport=DDS"
        )

    def close(self) -> None:
        self._close_rgbd_relays()

    def target_lost(self) -> None:
        self._close_rgbd_relays()

    def _close_rgbd_relays(self) -> None:
        if self._rgbd_relay is not None:
            self._rgbd_relay.close()
            self._rgbd_relay = None
        if self._rgbd_right_relay is not None:
            self._rgbd_right_relay.close()
            self._rgbd_right_relay = None
        if self._rgbd_left_preview_relay is not None:
            self._rgbd_left_preview_relay.close()
            self._rgbd_left_preview_relay = None

    @property
    def available_endpoints(self):
        candidates = {d['endpoint_id']: d for d in self._connection.motion_targets}
        peers = {d['endpoint_id']: d for d in self._connection.endpoints}
        for endpoint_id, descriptor in candidates.items():
            peers.setdefault(endpoint_id, descriptor)
        return [{**d, 'control_selectable': endpoint_id in candidates}
                for endpoint_id, d in peers.items()]

    @property
    def active_endpoint(self) -> str:
        return str(self._connection.active_target)


def _run() -> None:
    parser = argparse.ArgumentParser(description="EleSim control computation agent")
    parser.add_argument("--config", default=str(_CONFIG_ROOT / "config.yaml"))
    parser.add_argument("--mode", default=None, help="select a profile from the application YAML")
    parser.add_argument("--runtime-config", default=str(_CONFIG_ROOT / "runtime.yaml"))
    parser.add_argument("--id", default="")
    parser.add_argument("--target", default="")
    parser.add_argument("--scan-models", default="", help="Pilot-local scan model API 1 YAML")
    parser.add_argument("--scan-output-root", default="", help="immutable scene exchange root; defaults to XDG cache/scene-imports")
    args = parser.parse_args()
    bundle = load_app_config(args.config, mode=args.mode)
    role = load_runtime_role_config(args.runtime_config)
    if role.role != "pilot":
        raise ValueError(f"runtime role must be pilot, got {role.role!r}")
    pilot_id = str(args.id).strip() or role.endpoint_id
    target_id = str(args.target).strip() or role.active_target
    link = ControlClient(cfg=bundle.mapping_config)
    connection = PilotConnection(
        pilot_id=pilot_id,
        mapping=bundle.mapping_config,
        initial_target=target_id,
        state_sink=link,
        dds_settings=role.dds,
        rgbd_topic=str(
            ((role.rgbd or {}).get("wire") or {}).get("topic", "")
        ),
        rgbd_right_topic=str(
            ((role.rgbd or {}).get("wire") or {}).get("right_topic", "")
        ),
    )
    link.attach_sender(connection.submit)
    runtime = build_control_runtime(args.config, link, mode=args.mode)
    facade = _ControlFacade(
        runtime.service,
        connection,
        dds_settings=role.dds,
        imu_model_directory=Path(args.config).resolve().parent / "imu_models",
        rgbd_broker_topic=str(
            ((role.rgbd or {}).get("wire") or {}).get("topic", "")
        ),
        rgbd_right_broker_topic=str(
            ((role.rgbd or {}).get("wire") or {}).get("right_topic", "")
        ),
        rgbd_left_preview_topic=str(
            ((role.rgbd or {}).get("wire") or {}).get("left_preview_topic", "")
        ),
    )
    simulation_sync = SimulationWorkflowSync(runtime.service)
    directory = Path(args.config).resolve().parent
    trajectory = ArmTrajectory(connection, runtime.service, runtime.state,
        model_dir=directory / bundle.ik_config.curobo_model,
        world_path=(directory / bundle.ik_config.curobo_world) if bundle.ik_config.curobo_world else None)
    facade._arm_trajectory = trajectory
    mock_hug = MockHugCoordinator(
        link,
        lambda: simulation_sync.latest,
        lambda: SimQ(
            float(runtime.state.linear),
            float(runtime.state.roll),
            float(runtime.state.theta1),
            float(runtime.state.theta2),
        ),
        mapping=bundle.mapping_config,
        execution_context=facade.mock_hug_execution_context,
    )
    facade.attach_mock_hug(mock_hug)
    object_scan = LiveScan(connection, runtime.service, runtime.state,
        simulation_status=lambda: simulation_sync.latest,
        models_path=(Path(args.scan_models).expanduser().absolute() if args.scan_models
                     else directory/'scan.integrated.yaml'),
        model_dir=(directory/bundle.ik_config.curobo_model).resolve(),
        hand_eye=bundle.sim_config.hand_eye_config,
        output_root=(Path(args.scan_output_root).expanduser().absolute() if args.scan_output_root
                     else default_scene_output_root()),
        other_busy=lambda: trajectory.active or mock_hug.running)
    facade._object_scan = object_scan

    def _telemetry(payload, **source):
        runtime.service.accept_floor_telemetry(payload, **source)
        object_scan.accept_telemetry(payload, **source)

    def _control_tick():
        trajectory.tick()
        object_scan.tick()

    def _closing():
        trajectory.cancel_arm_trajectory()
        object_scan.request_cancel('Pilot stopping')
        object_scan.tick()

    connection.telemetry_observer = _telemetry
    connection.control_tick = _control_tick
    connection.motion_guard = lambda payload: trajectory.allow_motion(payload) and object_scan.allow_motion(payload)
    connection.on_closing = _closing
    simulation_sync.add_cancel_callback(object_scan.request_cancel)
    simulation_sync.add_cancel_callback(mock_hug.cancel)
    def _target_changed(target: str) -> None:
        simulation_sync.clear("simulation target changed")
        if not str(target).strip():
            facade.target_lost()

    connection.on_target_changed = _target_changed
    dispatcher = OperatorDispatcher(runtime.state, facade)
    connection.on_target_selected = facade.configure_target_stream
    connection.operator_handler = dispatcher.handle
    connection.simulation_status_handler = simulation_sync.accept
    connection.start()
    print(
        f"[pilot_agent] DDS system={role.dds.system_id} "
        f"domain={role.dds.domain_id} target={target_id}"
    )
    try:
        while True:
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        object_scan.close()
        trajectory.request_cancel()
        mock_hug.close()
        simulation_sync.close()
        runtime.service.close()
        connection.close()
        # Target callbacks also own these relays. Join their caller before
        # releasing media endpoints from the main thread.
        facade.close()


def main() -> None:
    with graceful_sigterm():
        configure_tracing("elesim-pilot-agent", local_log=True)
        try:
            with span(
                "elesim_pilot.main.main",
                attributes={
                    "code.function.name": "elesim_pilot.main.main",
                    "elesim.flow.id": "pilot.lifecycle",
                },
            ):
                _run()
        finally:
            shutdown_tracing()


if __name__ == "__main__":
    main()
