"""Coordinate physics edits and a same-revision camera readiness barrier."""
from dataclasses import replace
from pathlib import Path
import time

from .scene_artifact import verify_scene_revision
from .scene_rebuild import SceneRebuilder, SceneRollbackError, import_root, load_candidate


def _stop_cameras(scene):
    scene.suspend_camera_delivery()
    scene.close_frame_dispatchers(strict=True, keep_camera_worker=True)
    scene.suspend_camera_delivery()  # Discard final callbacks from the stopped generation.
    scene.frame_dispatcher = scene.rgbd_dispatcher = None
    if scene.frame_hub is not None:
        scene.frame_hub.clear()
    for mailbox in (scene.video_mailboxes or {}).values():
        mailbox.clear()
    scene._async_frame_seen.clear()
    scene._camera_failures.clear()
    scene._latest_camera_axes = None
    scene._camera_joint_position_cache.clear()
    scene._camera_root_pos_cache = scene._camera_root_quat_cache = None
    scene._last_camera_publish_t = scene._last_observer_camera_publish_t = 0.
    scene._next_camera_publish_sim_t = scene._next_observer_camera_publish_sim_t = 0.


def _start_cameras(app, authorized):
    from elesim_sim.runtime import RuntimePrep

    scene, cfg = app.sim_scene, app.cfg
    streams = {}
    if scene.hand_eye_enabled:
        streams["hand_eye_preview"] = (cfg.sim_camera_width, cfg.sim_camera_height)
    if scene.observer_enabled:
        streams["observer"] = (cfg.sim_observer_camera_width, cfg.sim_observer_camera_height)
    scene.camera_render_epoch += 1
    if streams:
        spec = RuntimePrep(app).camera_render_spec(scene.robot_urdf_path)
        worker = scene.camera_render_worker
        if worker is not None and worker.failure:
            worker.close()
            scene.camera_render_worker = worker = None
        if worker is None:
            scene.configure_camera_render_worker(spec, streams,
                timeout_s=cfg.camera_worker_start_timeout_s, wait=True)
        else:
            worker.rebuild_scene(spec.scene_manifest, spec.scene_revision,
                epoch=scene.camera_render_epoch, timeout_s=cfg.camera_worker_start_timeout_s)
        # Both edit acknowledgements are consumed on the receiver thread.
        # Clearing here excludes every callback from the previous generation.
        scene.suspend_camera_delivery()
        deadline = time.monotonic() + cfg.camera_first_frame_timeout_s
        while not set(streams).issubset(scene.staged_camera_streams()):
            if not authorized():
                raise PermissionError("simulation session changed during camera startup")
            if scene.camera_render_worker.failure:
                raise RuntimeError("rebuilt camera failed: " + scene.camera_render_worker.failure)
            if time.monotonic() >= deadline:
                raise TimeoutError("rebuilt cameras did not produce both first frames")
            now = scene.sim_time_s(float(app.params.dt))
            scene.maybe_publish_camera(arm_q=None, max_hz=cfg.sim_camera_max_hz,
                sim_time_s=now, force=True, rgb_enabled=cfg.sim_camera_rgb,
                depth_enabled=cfg.sim_camera_depth)
            scene.maybe_publish_observer_camera(max_hz=cfg.sim_observer_camera_max_hz,
                sim_time_s=now, force=True)
            time.sleep(.02)
    if not authorized():
        raise PermissionError("simulation session changed before frame activation")
    scene.configure_frame_dispatchers()
    scene.start_frame_dispatchers()
    scene.resume_camera_delivery()
    for name in streams:
        if not scene.flush_video_frame(name, timeout_s=cfg.camera_first_frame_timeout_s):
            raise RuntimeError("rebuilt frame publication failed: " + name)


def apply_scene_command(runtime, command):
    import genesis as gs

    app, scene = runtime.app, runtime.app.sim_scene
    if not gs.use_ndarray or app.cfg.genesis_performance_mode or app.cfg.camera_execution != "async_process":
        raise ValueError("scene rebuild requires dynamic arrays and async cameras")
    mailbox = app.operator_mailbox
    authorized = (lambda: mailbox.owns(command)) if mailbox is not None else (lambda: True)
    if not authorized():
        raise PermissionError("simulation session changed before scene loading")
    root = import_root(app.cfg.scene_import_root)
    candidate = (load_candidate(root, command.arguments["manifest"],
        command.arguments.get("expected_revision", "")) if command.command == "load_scene" else None)
    manifest = str(root / command.arguments["manifest"]) if candidate is not None else ""
    if scene.scene_rebuilder is None:
        scene.scene_rebuilder = SceneRebuilder(scene.scene, gs, app.imported_scene_entities)
    rebuilder = scene.scene_rebuilder
    old_cfg, old_artifact, old_entities = app.cfg, app.imported_scene, app.imported_scene_entities
    mover_name = scene.mover.entity.name
    go2_name = scene.go2_entity.name if scene.go2_entity is not None else None
    target_name = scene.sim_target_entity.name if scene.sim_target_entity is not None else None
    mock_names = {key: value.name for key, value in scene.mock_object_entities.items()}
    held_q = scene.mover.current_4dof_q()
    started = time.monotonic()

    def hold_motion():
        if app.state_source is not None:
            app.state_source.hold_for_scene_rebuild(held_q)

    if mailbox is not None:
        with mailbox.motion_lock:
            if not authorized():
                raise PermissionError("simulation session changed before scene teardown")
            mailbox.rebuilding.set()
            hold_motion()
    else:
        hold_motion()
    if app.runtime_ready_event is not None:
        app.runtime_ready_event.clear()

    def rebind():
        scene.mover.rebind_entity(scene.scene.get_entity(mover_name))
        if go2_name is not None:
            scene.go2_entity = scene.scene.get_entity(go2_name)
            scene.go2.rebind_entity(scene.go2_entity, scene.mover.entity)
        if target_name is not None:
            scene.sim_target_entity = scene.scene.get_entity(target_name)
        scene.mock_object_entities = {key: scene.scene.get_entity(name) for key, name in mock_names.items()}
        for entity in scene.scene.entities:
            if any(isinstance(morph, gs.morphs.Plane) for morph in entity.desc.morphs):
                entity.set_friction(float(app.go2_locomotion_config.mpc_physical_friction))
        app.markers = type(app.markers)()
        runtime._ik_marker = type(runtime._ik_marker)()
        runtime._arm_ghost = type(runtime._arm_ghost)(
            str(Path(app.cfg.build_dir) / app.cfg.arm_urdf_name))
        runtime._ghost_frame = None
        runtime._ghost_alignment_logged_plan_id = ""
        runtime._sim_tip_cache = runtime._sim_tip_dir_cache = None
        runtime._next_feedback_sim_t = 0.
        runtime._force_hand_eye_capture = True

    def cameras(artifact, entities):
        _stop_cameras(scene)
        rollback = entities is None
        app.cfg = old_cfg if rollback else replace(old_cfg, scene_manifest=manifest)
        app.imported_scene = old_artifact if rollback else artifact
        app.imported_scene_entities = old_entities if rollback else entities
        if rollback:
            # Old handles were destroyed too; recover the committed objects by name.
            app.imported_scene_entities = {key: scene.scene.get_entity(value.name)
                for key, value in old_entities.items()}
        _start_cameras(app, (lambda: True) if rollback else authorized)

    def validate():
        if candidate is not None:
            verify_scene_revision(manifest, candidate.revision)

    restored = False
    try:
        _stop_cameras(scene)
        runtime._status_dirty = True
        runtime._publish_simulation_status(force=True)
        entities, _ = rebuilder.replace(candidate, rebind=rebind, cameras=cameras,
            authorized=authorized, validate=validate)
        app.imported_scene_entities = entities
        restored = True
        revision = candidate.revision[:12] if candidate else "empty"
        timings = rebuilder.last_timings
        reason = (f"scene ready: revision={revision} objects={len(entities)} "
            f"physics={timings['physics_s']:.2f}s cameras={timings['cameras_s']:.2f}s "
            f"total={time.monotonic()-started:.2f}s")
        print('[sim-scene] ' + reason, flush=True)
        return reason
    except SceneRollbackError:
        raise
    except Exception as error:
        if scene.scene.is_built and scene._camera_delivery_suspended:
            try:
                cameras(None, None)
            except Exception as rollback_error:
                raise SceneRollbackError(f"camera recovery failed: {rollback_error}") from error
        restored = scene.scene.is_built and not scene._camera_delivery_suspended
        raise
    finally:
        if mailbox is not None:
            with mailbox.motion_lock:
                hold_motion()
                mailbox.rebuilding.clear()
        if app.runtime_ready_event is not None and restored:
            app.runtime_ready_event.set()
