from __future__ import annotations

import math
import time

import imgui

from ui.helpers import panel_header


def _fmt_xyz(vec: tuple[float, float, float] | None, *, signed: bool = False) -> str:
    if vec is None:
        return "—"
    if signed:
        return "(%+.3f, %+.3f, %+.3f)" % (float(vec[0]), float(vec[1]), float(vec[2]))
    return "(%.3f, %.3f, %.3f)" % (float(vec[0]), float(vec[1]), float(vec[2]))


def _normalized_xyz(vec: tuple[float, float, float] | None) -> tuple[float, float, float] | None:
    if vec is None:
        return None
    x, y, z = float(vec[0]), float(vec[1]), float(vec[2])
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 1e-9:
        return None
    return (x / norm, y / norm, z / norm)


def _heartbeat_tag(last_update_s: float, *, active: bool, stale_s: float = 2.0) -> str:
    if not active:
        return "OFF"
    if float(last_update_s) <= 0.0:
        return "WAIT"
    age = max(0.0, time.time() - float(last_update_s))
    if age <= float(stale_s):
        return "LIVE"
    return f"STALE {age:.1f}s"


def _fmt_uv(uv: tuple[float, float] | None) -> str:
    if uv is None:
        return "—"
    return f"({float(uv[0]):+.3f}, {float(uv[1]):+.3f})"


def _blank(value: object) -> str:
    text = str(value or "").strip()
    return text if text else "—"


def _line(label: str, value: object, *, color: tuple[float, float, float] | None = None) -> None:
    text = f"{label}: {_blank(value)}"
    if color is None:
        imgui.text(text)
    else:
        imgui.text_colored(text, float(color[0]), float(color[1]), float(color[2]))


def _host_connected(host) -> bool:
    return host is not None and bool(getattr(host, "connected", False))


def _draw_hardware_brief(panel) -> None:
    host = panel._host_state
    connected = _host_connected(host)
    _line(
        "Host",
        "OK" if connected else "OFF",
        color=None if connected else (0.70, 0.36, 0.05),
    )
    _line("Device", getattr(host, "device", "") if host is not None else "")
    ports = tuple(getattr(host, "ports", ()) or ()) if host is not None else ()
    _line("Ports", ", ".join(str(p) for p in ports))
    _line(
        "Link",
        "rx_age=%.2fs  tx=%d"
        % (
            float(getattr(host, "rx_age_s", 0.0)) if host is not None else 0.0,
            int(getattr(host, "tx_seq", 0)) if host is not None else 0,
        ),
    )
    _line("Torque", "ON" if bool(getattr(host, "torque_enabled", False)) else "OFF")
    _line("Gripper", "CLOSED" if bool(panel.state.claw_closed) else "OPEN")
    _line("Claw current", "%dmA" % int(getattr(host, "claw_current", 0) if host is not None else 0))
    currents = getattr(host, "motor_currents_ma", {}) if host is not None else {}
    current_text = ", ".join(f"{k}={int(v)}mA" for k, v in (currents or {}).items())
    _line("Motor currents", current_text)
    safety_fault = str(getattr(host, "safety_fault", "") if host is not None else "").strip()
    _line("Safety fault", safety_fault, color=(1.0, 0.25, 0.25) if safety_fault else None)
    reply_reason = str(getattr(host, "reply_reason", "") or "").strip()
    reply_ok = bool(getattr(host, "reply_ok", True)) if host is not None else True
    _line("Host reply", reply_reason, color=(1.0, 0.35, 0.35) if reply_reason and not reply_ok else None)


def _draw_arm_brief(panel) -> None:
    host = panel._host_state
    tip_xyz = host.actual_tip_xyz if host is not None else None
    tip_dir = host.actual_tip_dir if host is not None else None
    _line("Tip xyz [m]", _fmt_xyz(tip_xyz))
    _line("Tip dir", _fmt_xyz(_normalized_xyz(tip_dir), signed=True))

    u_now = panel.service.current_control_u()
    _line(
        "U",
        "linear=%.1f  roll=%.1f  seg1=%.1f  seg2=%.1f"
        % (float(u_now.u_linear), float(u_now.u_roll), float(u_now.u_s1), float(u_now.u_s2)),
    )
    off_linear, off_roll, off_s1, off_s2, _ = panel.state.offset_values()
    _line(
        "Offsets",
        "linear=%.1f  roll=%.1f  seg1=%.1f  seg2=%.1f"
        % (float(off_linear), float(off_roll), float(off_s1), float(off_s2)),
    )
    _line("Control lock", "ON" if bool(panel.state.paused) else "OFF")


def _draw_go2_brief(panel) -> None:
    enabled = bool(getattr(panel, "_use_go2", False))
    host = panel._host_state
    vel = (0.0, 0.0, 0.0) if host is None else tuple(float(v) for v in getattr(host, "go2_vel", (0.0, 0.0, 0.0)))
    _line("GO2", "enabled" if enabled else "disabled")
    _line("GO2 vel", "vx=%+.2f  vy=%+.2f  wz=%+.2f" % vel)
    _line("GO2 base pos [m]", _fmt_xyz(getattr(host, "go2_base_pos", None) if host is not None else None, signed=True))
    _line("GO2 base rpy [rad]", _fmt_xyz(getattr(host, "go2_base_rpy", None) if host is not None else None, signed=True))


def _draw_ik_brief(panel) -> None:
    st = panel.state
    status = "idle"
    if bool(st.ik_running):
        status = "running"
    if bool(st.ik_converged):
        status = "converged"
    if bool(st.ik_failed):
        status = "failed"
    _line("IK", "%s  err=%.2fmm" % (status, float(st.ik_err_m) * 1000.0))
    _line(
        "IK solution",
        "roll=%.3f  theta1=%.3f  theta2=%.3f"
        % (float(st.ik_sol_roll), float(st.ik_sol_theta1), float(st.ik_sol_theta2)),
    )
    _line(
        "IK track error",
        "roll=%.3f  theta1=%.3f  theta2=%.3f  bend_max=%.3f"
        % (
            float(st.ik_track_roll_err_rad),
            float(st.ik_track_theta1_err_rad),
            float(st.ik_track_theta2_err_rad),
            float(st.ik_track_bend_max_err_rad),
        ),
    )
    _line("IK msg", st.ik_status_msg)


def _draw_pick_brief(panel) -> None:
    st = panel.state
    pick_running = bool(st.pick_running) or bool(panel.service.pick_e2e_running())
    pick_status = "running" if pick_running else "idle"
    if bool(st.pick_failed):
        pick_status = "failed"
    _line("Pick", "%s  phase=%s" % (pick_status, _blank(st.pick_phase)))
    _line("Pick msg", st.pick_status_msg)
    _line("Sag model", st.sag_model_path)
    _line("Sag status", getattr(panel, "_sag_status_text", ""))


def draw_live_visual_status(panel, *, show_separators: bool = True) -> None:
    """Perception / host relay / gaze heartbeat shown at panel top."""
    st = panel.state
    now = time.time()
    run_local = bool(getattr(panel, "_perception_run_local", True))

    if show_separators:
        imgui.separator()
    imgui.text("Vision / Gaze")
    _line("Perception source", "local" if run_local else "remote")
    _line("Detector config", getattr(panel, "_perception_config_path_draft", ""))
    _line(
        "Visual target",
        "label=%s  scale=%.3f  uv=(%.2f, %.2f)"
        % (
            _blank(st.visual_target_label),
            float(st.visual_target_scale),
            float(st.visual_target_uv_u),
            float(st.visual_target_uv_v),
        ),
    )

    host = panel._host_state
    host_age = -1.0
    host_live = False
    if host is not None and bool(getattr(host, "connected", False)):
        if float(host.perceived_timestamp_s) > 0.0:
            host_age = max(0.0, now - float(host.perceived_timestamp_s))
        host_live = host.perceived_center_uv is not None and host_age >= 0.0 and host_age <= 0.75

    perc_active = bool(st.perception_running) and not bool(st.perception_failed)
    if not run_local and host_live:
        perc_active = True
    perc_tag = _heartbeat_tag(st.perception_last_update_s, active=perc_active)
    if st.perception_failed:
        perc_tag = "FAILED"
    elif not perc_active:
        perc_tag = "OFF"
    elif not run_local:
        perc_tag = "REMOTE"

    _line(
        "Perception",
        "[%s]  frame=%d  center_uv=%s  det=%s  conf=%.2f"
        % (
            perc_tag,
            int(st.perception_frame_idx),
            _fmt_uv(st.perception_center_uv),
            str(st.perception_label) or "(none)",
            float(st.perception_confidence),
        ),
    )
    bw = st.perception_bbox_wh
    _line(
        "Perception track",
        "phase=%s  ok=%d  scale=%.3f  bbox=%dx%d  backend=%s"
        % (
            _blank(st.perception_tracker_phase),
            int(st.perception_track_ok_frames),
            float(st.perception_image_scale),
            int(bw[0]),
            int(bw[1]),
            _blank(st.perception_tracker_backend),
        ),
    )
    _line("Camera XYZ [m]", _fmt_xyz(st.perception_camera_xyz, signed=True))
    _line("World XYZ [m]", _fmt_xyz(st.perception_world_xyz, signed=True))
    _line("Last capture", st.perception_last_capture_path)
    _line("Perception msg", st.perception_status_msg)

    if host is None or not bool(getattr(host, "connected", False)):
        relay_text = "[OFF]  uv=—  scale=—  label=—  age=—"
    else:
        host_age = -1.0
        if float(host.perceived_timestamp_s) > 0.0:
            host_age = max(0.0, now - float(host.perceived_timestamp_s))
        local_age = -1.0
        if float(st.perception_last_update_s) > 0.0:
            local_age = max(0.0, now - float(st.perception_last_update_s))
        host_tag = "OFF"
        if host.perceived_center_uv is not None and host_age >= 0.0 and host_age <= 0.75:
            host_tag = "LIVE"
        elif host.perceived_center_uv is not None and host_age > 0.75:
            host_tag = f"STALE {host_age:.1f}s"
        elif host_age >= 0.0:
            host_tag = f"WAIT {host_age:.1f}s"
        if (
            run_local
            and perc_active
            and local_age >= 0.0
            and local_age <= 0.75
            and host_age > 0.75
            and st.perception_center_uv is not None
        ):
            host_tag = f"{host_tag} (local ok {local_age:.1f}s)"
        scale_str = "—" if host.perceived_scale is None else f"{float(host.perceived_scale):.3f}"
        age_str = "—" if host_age < 0.0 else "%.2fs" % float(host_age)
        relay_text = "[%s]  uv=%s  scale=%s  label=%s  age=%s" % (
            host_tag,
            _fmt_uv(host.perceived_center_uv),
            scale_str,
            str(host.perceived_object_label) or "(none)",
            age_str,
        )
    _line("Host relay", relay_text)

    gaze_tag = "RUNNING" if bool(st.gaze_running) else "OFF"
    _line(
        "Gaze",
        "[%s]  mode=%s  updates=%d  u_err=%+.3f  v_err=%+.3f"
        % (
            gaze_tag,
            str(st.gaze_mode) or "idle",
            int(st.gaze_update_count),
            float(st.gaze_u_err),
            float(st.gaze_v_err),
        ),
    )
    _line(
        "Gaze delta",
        "du roll/s1/s2=%+.4f / %+.4f / %+.4f  obs_age=%.2fs  target_uv=(%.2f, %.2f)"
        % (
            float(st.gaze_du_roll),
            float(st.gaze_du_s1),
            float(st.gaze_du_s2),
            float(st.gaze_obs_age_s),
            float(st.visual_target_uv_u),
            float(st.visual_target_uv_v),
        )
    )
    _line("Gaze msg", st.gaze_status_msg)

    needed = (
        (run_local and not st.perception_running)
        or st.perception_center_uv is None
        or (host is not None and host.perceived_center_uv is None)
    )
    note = ""
    if bool(st.gaze_running) and needed:
        if run_local:
            note = (
                "Gaze needs Perception Start + host UV relay. "
                "Check sim camera, target label, and that a target is visible."
            )
        else:
            note = (
                "Gaze needs Jetson perception_worker + host UV relay. "
                "Check RealSense, target label, and worker process on Jetson."
            )
    elif host is not None and run_local and perc_active and st.perception_center_uv is not None:
        host_age_note = -1.0
        if float(getattr(host, "perceived_timestamp_s", 0.0)) > 0.0:
            host_age_note = max(0.0, now - float(host.perceived_timestamp_s))
        if host_age_note > 0.75:
            note = "Perception is live locally but host relay is stale."
    _line("Vision note", note)

    if show_separators:
        imgui.separator()


def draw_status_panel(panel) -> None:
    if not panel._status_header_init_open:
        cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
        imgui.set_next_item_open(True, cond)
        panel._status_header_init_open = True
    if not panel_header("Status", visible=True)[0]:
        return

    _draw_hardware_brief(panel)
    imgui.separator()
    _draw_arm_brief(panel)
    imgui.separator()
    _draw_go2_brief(panel)
    imgui.separator()
    _draw_ik_brief(panel)
    imgui.separator()
    _draw_pick_brief(panel)
    imgui.separator()
    draw_live_visual_status(panel, show_separators=False)


def draw_gaze_status_compact(panel) -> None:
    st = panel.state
    tag = "RUNNING" if bool(st.gaze_running) else "OFF"
    imgui.text(
        "Gaze [%s] mode=%s updates=%d u_err=%+.3f v_err=%+.3f"
        % (
            tag,
            str(st.gaze_mode) or "idle",
            int(st.gaze_update_count),
            float(st.gaze_u_err),
            float(st.gaze_v_err),
        )
    )
    if str(st.gaze_status_msg).strip():
        imgui.text_wrapped(str(st.gaze_status_msg))
