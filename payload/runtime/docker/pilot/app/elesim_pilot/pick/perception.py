"""Perception capture, preview, and mock-target workflow methods."""
from __future__ import annotations
from collections.abc import Mapping
from dataclasses import fields
from ._deps import *  # noqa: F401,F403

class PerceptionActions:
    def _publish_perception_to_host(
        self,
        *,
        object_camera_xyz: tuple[float, float, float],
        label: str,
        confidence: float,
        image_center_uv: tuple[float, float],
        image_scale: float,
        depth_valid: bool = True,
        object_world: Optional[tuple[float, float, float]] = None,
        camera_world_origin: Optional[tuple[float, float, float]] = None,
        camera_world_look: Optional[tuple[float, float, float]] = None,
        camera_world_right: Optional[tuple[float, float, float]] = None,
    ) -> Optional[tuple[float, float, float]]:
        if self.client is None:
            return None
        freeze_world = bool(self.state.pick_running)
        if bool(self._grasp_uv_only_mode):
            publish_depth = False
        else:
            publish_depth = bool(depth_valid) and (
                not freeze_world or not bool(self._pick_equal_sag_attempted)
            )
        p_world = self.client.send_perception_observation(
            object_camera_xyz=object_camera_xyz,
            label=label,
            confidence=confidence,
            image_center_uv=image_center_uv,
            image_scale=image_scale,
            depth_valid=publish_depth,
            object_world=object_world,
            camera_world_origin=camera_world_origin,
            camera_world_look=camera_world_look,
            camera_world_right=camera_world_right,
        )
        if freeze_world:
            frozen = self._pick_frozen_world()
            return frozen if frozen is not None else p_world
        if p_world is not None:
            self._pick_frozen_world_xyz = tuple(p_world)
        return p_world

    def _stop_remote_preview(self) -> None:
        return

    def start_perception_capture(self, *, config: Optional[PerceptionConfig] = None) -> None:
        if config is not None:
            self.update_perception_config(config)
        if not self._perception_run_local:
            self.state.set_perception_status(
                running=False,
                failed=True,
                msg=(
                    "remote Robot perception is unsupported; select its DDS "
                    "RGB-D stream and run perception in Pilot"
                ),
            )
            return
        old = self._perception_capture
        if old is not None:
            if old.is_running():
                if not old.stop(timeout_s=10.0):
                    self._retire_perception_capture(old)
                    self.state.set_perception_status(
                        running=False,
                        failed=True,
                        msg="prior capture did not stop; retrying start",
                    )
                else:
                    self._retire_perception_capture(old)
            else:
                self._retire_perception_capture(old)
        cfg = self._perception_cfg
        self._perception_cfg = cfg
        self.state.visual_target_label = str(cfg.target_label).strip()
        epoch = int(self._perception_capture_epoch) + 1
        self._perception_capture_epoch = epoch
        cap = PerceptionCapture(
            cfg,
            publish_fn=self._publish_perception_to_host,
            on_snapshot=lambda snap, e=epoch: self._on_perception_snapshot(
                snap,
                capture_epoch=e,
            ),
            target_uv_fn=lambda: (
                float(self.state.visual_target_uv_u),
                float(self.state.visual_target_uv_v),
            ),
            mock_world_xyz_fn=self._mock_world_xyz_from_state,
        )
        self._perception_capture = cap
        self.state.set_perception_status(running=True, failed=False, msg="starting")
        cap.start()

    def stop_perception_capture(self, *, stop_recording: bool = True) -> None:
        if not self._perception_run_local:
            self._stop_remote_preview()
            if bool(stop_recording):
                self._stop_observer_camera_recording()
            if self.client is not None and hasattr(self.client, "send_perception_stop"):
                self.client.send_perception_stop()
            if bool(stop_recording):
                self.state.set_perception_recording(False)
            self.state.set_perception_status(
                running=False,
                failed=False,
                msg="remote: stopping Jetson perception",
            )
            return
        cap = self._perception_capture
        if cap is None:
            if bool(stop_recording):
                self._stop_observer_camera_recording()
                self.state.set_perception_recording(False)
            self.state.set_perception_status(running=False, failed=False, msg="stopped")
            return
        stopped = cap.stop(stop_recording=bool(stop_recording))
        if not stopped:
            if bool(stop_recording):
                self._retire_perception_capture(cap, stop_recording=True)
                self.state.set_perception_recording(False)
            self.state.set_perception_status(running=False, failed=True, msg="stop pending")
            return
        if bool(stop_recording):
            self._retire_perception_capture(cap, stop_recording=True)
            self.state.set_perception_recording(False)
        self.state.set_perception_status(
            running=False,
            failed=False,
            msg="stopped" if bool(stop_recording) else "stopped (recording kept)",
        )

    def refresh_perception_capture(self) -> None:
        if not self._perception_run_local:
            if self.client is not None and hasattr(self.client, "send_perception_refresh"):
                self.client.send_perception_refresh()
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=False,
                msg="remote: refresh requested",
            )
            return
        cap = self._perception_capture
        if cap is None or not cap.is_running():
            self.state.set_perception_status(running=False, failed=True, msg="perception is not running")
            return
        if cap.request_refresh():
            self.state.set_perception_status(running=True, failed=False, msg="refresh requested (YOLO)")
        else:
            self.state.set_perception_status(running=False, failed=True, msg="refresh rejected")

    def _observer_camera_config(self) -> Optional[SimConfig]:
        print(
            "[perception] Pilot-side observer capture is unavailable: "
            "observer pixels are owned by the UI WebRTC session"
        )
        return None

    @staticmethod
    def _observer_record_path_for(record_path: str | Path) -> Path:
        p = Path(record_path)
        stem = p.stem
        if stem.endswith("_record"):
            stem = stem[: -len("_record")]
        return p.with_name(f"{stem}_observer.mp4")

    @staticmethod
    def _observer_snapshot_stem_for(capture_path: str | Path) -> str:
        stem = Path(capture_path).stem
        for suffix in ("_depth_vis", "_color", "_overlay", "_depth", "_meta"):
            if stem.endswith(suffix):
                stem = stem[: -len(suffix)]
                break
        return f"{stem}_observer"

    def _start_observer_camera_recording(self, record_path: str | Path) -> Optional[Path]:
        if self._observer_camera_recorder is not None:
            self._stop_observer_camera_recording()
        cfg = self._observer_camera_config()
        if cfg is None:
            return None
        try:
            from elesim_pilot.vision.sim_camera.recording import SimCameraVideoRecorder
        except Exception as exc:
            print(f"[perception] observer recorder import failed: {exc}")
            return None
        out_path = self._observer_record_path_for(record_path)
        rec = SimCameraVideoRecorder(
            str(cfg.sim_observer_camera_stream),
            out_path=out_path,
            fps=float(getattr(cfg, "sim_observer_camera_record_fps", 30.0)),
            use_jpeg=bool(cfg.sim_observer_camera_jpeg),
        )
        if not rec.start():
            print(f"[perception] observer recording skipped: {rec.last_error}")
            return None
        self._observer_camera_recorder = rec
        self._observer_camera_record_path = out_path
        print(f"[perception] observer recording started: {out_path.resolve()}")
        return out_path

    def _stop_observer_camera_recording(self) -> Optional[tuple[bool, str, int, int, str]]:
        rec = self._observer_camera_recorder
        if rec is None:
            return None
        self._observer_camera_recorder = None
        self._observer_camera_record_path = None
        ok, path_s, frame_count, unique_count, err = rec.stop()
        if ok:
            print(
                "[perception] observer recording saved (%df/%du): %s"
                % (int(frame_count), int(unique_count), path_s)
            )
        else:
            print(f"[perception] observer recording stop failed: {err or path_s}")
        return bool(ok), str(path_s), int(frame_count), int(unique_count), str(err or "")

    def _capture_observer_camera_snapshot(self, paired_path: str | Path) -> Optional[Path]:
        cfg = self._observer_camera_config()
        if cfg is None:
            return None
        try:
            from elesim_pilot.vision.sim_camera.recording import (
                capture_sim_camera_snapshot,
                save_sim_camera_snapshot,
            )
        except Exception as exc:
            print(f"[perception] observer snapshot import failed: {exc}")
            return None
        try:
            frame = capture_sim_camera_snapshot(
                str(cfg.sim_observer_camera_stream),
                use_jpeg=bool(cfg.sim_observer_camera_jpeg),
                timeout_s=1.5,
            )
            if frame is None:
                print("[perception] observer snapshot skipped: no observer camera frame")
                return None
            paired = Path(paired_path)
            stem = self._observer_snapshot_stem_for(paired)
            observer_path = save_sim_camera_snapshot(
                frame=frame,
                out_dir=paired.parent,
                stem=stem,
                meta={
                    "paired_capture": str(paired.resolve()),
                    "stream": str(cfg.sim_observer_camera_stream),
                },
            )
            print(f"[perception] observer snapshot saved {observer_path.resolve()}")
            return observer_path
        except Exception as exc:
            print(f"[perception] observer snapshot failed: {exc}")
            return None

    def capture_perception_frame(self) -> bool:
        """Save latest perception frame (or one-shot sim grab) under logs/perception_capture/."""
        if not self._perception_run_local:
            if self.client is None or not hasattr(self.client, "send_perception_capture"):
                self.state.set_perception_status(
                    running=bool(self.state.perception_running),
                    failed=True,
                    msg="remote: snapshot unsupported by host client",
                )
                return False
            self.client.send_perception_capture(
                include_overlay=bool(self.state.perception_record_with_overlay)
            )
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=False,
                msg="remote: snapshot requested on Jetson",
            )
            return True
        out_dir = default_perception_capture_dir()
        cap = self._perception_capture
        path: Optional[Path] = None
        if cap is not None and cap.has_cached_frame():
            path = cap.save_cached_frames(
                out_dir,
                extra_meta={"mode": str(self._perception_cfg.mode)},
            )
        if path is None:
            path = self._capture_sim_perception_frame_once(out_dir)
        if path is None:
            msg = "capture failed: no frame (start perception or check sim camera)"
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=True,
                msg=msg,
            )
            print(f"[perception] {msg}")
            return False
        path_s = str(path.resolve())
        observer_path = self._capture_observer_camera_snapshot(path)
        observer_s = "" if observer_path is None else str(observer_path.resolve())
        self.state.set_perception_last_capture(path_s)
        msg = (
            f"saved {path_s}"
            if not observer_s
            else f"saved {path_s} + observer {observer_s}"
        )
        self.state.set_perception_status(
            running=bool(self.state.perception_running),
            failed=False,
            msg=msg,
        )
        print(f"[perception] {msg}")
        return True

    def start_perception_recording(self) -> bool:
        """Start recording local perception frames to MP4 under logs/perception_capture/."""
        if not self._perception_run_local:
            if self.client is None or not hasattr(self.client, "send_perception_record_start"):
                self.state.set_perception_status(
                    running=bool(self.state.perception_running),
                    failed=True,
                    msg="remote: recording unsupported by host client",
                )
                return False
            use_overlay = bool(self.state.perception_record_with_overlay)
            self.client.send_perception_record_start(
                include_overlay=use_overlay,
                fps=float(self._perception_cfg.publish_hz),
            )
            self.state.set_perception_recording(True, "Jetson host")
            overlay_tag = "overlay" if use_overlay else "raw"
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=False,
                msg=f"remote: recording start requested on Jetson ({overlay_tag})",
            )
            return True
        cap = self._perception_capture
        if cap is None or not cap.is_running():
            self.state.set_perception_status(running=False, failed=True, msg="perception is not running")
            return False
        use_overlay = bool(self.state.perception_record_with_overlay)
        ok, path_s = cap.start_recording(
            default_perception_capture_dir(),
            fps=float(self._perception_cfg.publish_hz),
            include_overlay=use_overlay,
        )
        if not ok:
            self.state.set_perception_status(running=True, failed=True, msg="recording already active")
            return False
        self.state.set_perception_recording(True, path_s)
        overlay_tag = "overlay" if use_overlay else "raw"
        observer_path = self._start_observer_camera_recording(path_s)
        observer_msg = (
            "" if observer_path is None else f" + observer {observer_path.resolve()}"
        )
        self.state.set_perception_status(
            running=True,
            failed=False,
            msg=f"recording started ({overlay_tag}): {path_s}{observer_msg}",
        )
        print(f"[perception] recording started ({overlay_tag}): {path_s}{observer_msg}")
        return True

    def stop_perception_recording(self) -> bool:
        if not self._perception_run_local:
            if self.client is None or not hasattr(self.client, "send_perception_record_stop"):
                self.state.set_perception_status(
                    running=bool(self.state.perception_running),
                    failed=True,
                    msg="remote: recording unsupported by host client",
                )
                return False
            self.client.send_perception_record_stop()
            self.state.set_perception_recording(False)
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=False,
                msg="remote: recording stop requested on Jetson",
            )
            return True
        cap = self._perception_capture
        if cap is None:
            self.state.set_perception_recording(False)
            self.state.set_perception_status(running=False, failed=True, msg="perception is not running")
            return False
        ok, path_s, frame_count = cap.stop_recording()
        if not ok:
            self._stop_observer_camera_recording()
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=True,
                msg="recording is not active",
            )
            return False
        observer_result = self._stop_observer_camera_recording()
        observer_msg = ""
        if observer_result is not None:
            observer_ok, observer_path_s, observer_frames, observer_unique, observer_err = (
                observer_result
            )
            if observer_ok:
                observer_msg = " | observer %df/%du: %s" % (
                    observer_frames,
                    observer_unique,
                    observer_path_s,
                )
            else:
                observer_msg = f" | observer failed: {observer_err or observer_path_s}"
        self.state.set_perception_recording(False, path_s)
        self.state.set_perception_status(
            running=bool(self.state.perception_running),
            failed=False,
            msg=f"recording saved ({frame_count}f): {path_s}{observer_msg}",
        )
        print(f"[perception] recording saved ({frame_count}f): {path_s}{observer_msg}")
        return True

    def toggle_perception_recording(self) -> bool:
        if bool(self.state.perception_recording):
            return self.stop_perception_recording()
        return self.start_perception_recording()

    def _capture_sim_perception_frame_once(self, out_dir: Path) -> Optional[Path]:
        if str(self._perception_cfg.mode).strip().lower() != "sim":
            return None
        _ensure_pick_place_path()
        try:
            from elesim_pilot.vision.perception.sim_rendered_camera import SimRenderedCamera
        except Exception as exc:
            print(f"[perception] sim camera import failed: {exc}")
            return None
        cfg = self._perception_cfg
        try:
            with SimRenderedCamera(
                topic=str(cfg.sim_camera_topic),
                dds_settings=cfg.sim_camera_dds_settings,
                expected_source_id=str(cfg.sim_camera_source_id),
                expected_boot_id=str(cfg.sim_camera_source_boot_id),
                wire_format=str(cfg.sim_camera_wire_format),
            ) as cam:
                frame = cam.capture(retries=60)
            return save_perception_frame_bundle(
                out_dir=out_dir,
                color_bgr=frame.color_bgr,
                depth_raw=frame.depth_raw,
                meta={
                    "mode": "sim",
                    "one_shot": True,
                    "depth_scale": float(frame.depth_scale),
                },
            )
        except Exception as exc:
            print(f"[perception] one-shot sim capture failed: {exc}")
            return None

    def update_perception_config(self, config: PerceptionConfig | Mapping[str, Any] | Any) -> None:
        if isinstance(config, PerceptionConfig):
            updated = config
        else:
            if isinstance(config, Mapping):
                raw = {str(key): value for key, value in config.items()}
            elif hasattr(config, "__dict__"):
                raw = {
                    str(key): value
                    for key, value in vars(config).items()
                    if not str(key).startswith("_")
                }
            else:
                raise TypeError("perception config update must be an object")
            allowed = {field.name for field in fields(PerceptionConfig)}
            unknown = sorted(set(raw) - allowed)
            if unknown:
                raise ValueError(
                    "unknown perception config fields: " + ", ".join(unknown)
                )
            updated = replace(self._perception_cfg, **raw)
        self._perception_cfg = updated
        self._perception_run_local = self._perception_config_runs_locally(updated)
        self.state.visual_target_label = str(updated.target_label).strip()

    def _mock_world_xyz_from_state(self) -> Optional[tuple[float, float, float]]:
        if str(self._perception_cfg.mode).strip().lower() != "mock":
            return None
        return self.state.mock_object_world_xyz()

    def set_mock_object_world(self, x: float, y: float, z: float) -> None:
        self.state.set_mock_object_world_xyz(float(x), float(y), float(z))

    def mock_object_preferred_dir(self) -> tuple[float, float, float]:
        return self.state.mock_object_preferred_dir()

    def set_mock_object_preferred_dir(self, x: float, y: float, z: float) -> None:
        self.state.set_mock_object_preferred_dir(float(x), float(y), float(z))

    def publish_mock_object_world(self) -> bool:
        """Push current mock object world XYZ to host (updates sim marker)."""
        if self.client is None:
            return False
        world_xyz = self.state.mock_object_world_xyz()
        camera_xyz = self.state.perception_camera_xyz
        if camera_xyz is None:
            camera_xyz = (0.0, 0.0, 0.65)
        label = str(self.state.perception_label).strip() or str(self.state.visual_target_label).strip() or "mock_object"
        confidence = float(self.state.perception_confidence)
        if confidence <= 0.0:
            confidence = 1.0
        image_scale = float(self.state.perception_image_scale)
        if image_scale <= 0.0:
            image_scale = float(self._pick_config_effective().target_scale)
        p_world = self._publish_perception_to_host(
            object_camera_xyz=tuple(float(v) for v in camera_xyz),
            label=label,
            confidence=confidence,
            image_center_uv=(0.0, 0.0),
            image_scale=image_scale,
            depth_valid=True,
            object_world=world_xyz,
        )
        ack_xyz = p_world if p_world is not None else world_xyz
        self.state.set_perception_status(
            running=bool(self.state.perception_running),
            failed=False,
            msg="mock object moved",
            world_xyz=ack_xyz,
            label=label,
            confidence=confidence,
            camera_xyz=camera_xyz,
        )
        return p_world is not None
