from __future__ import annotations

import time

import imgui


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


def draw_live_visual_status(panel) -> None:
    """Perception / host relay / gaze heartbeat shown at panel top."""
    st = panel.state
    now = time.time()

    imgui.separator()
    imgui.text("Live Status")

    perc_active = bool(st.perception_running) and not bool(st.perception_failed)
    perc_tag = _heartbeat_tag(st.perception_last_update_s, active=perc_active)
    if st.perception_failed:
        perc_tag = "FAILED"
    elif not st.perception_running:
        perc_tag = "OFF"

    imgui.text(
        "Perception [%s]  frame=%d  center_uv=%s  det=%s  conf=%.2f"
        % (
            perc_tag,
            int(st.perception_frame_idx),
            _fmt_uv(st.perception_center_uv),
            str(st.perception_label) or "(none)",
            float(st.perception_confidence),
        )
    )
    if str(st.perception_status_msg).strip() and (st.perception_failed or not st.perception_running):
        imgui.text_wrapped(f"  {st.perception_status_msg}")

    host = panel._host_state
    st = panel.state
    if host is None or not bool(getattr(host, "connected", False)):
        imgui.text("Host relay [OFF]  (host not connected)")
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
            perc_active
            and local_age >= 0.0
            and local_age <= 0.75
            and host_age > 0.75
            and st.perception_center_uv is not None
        ):
            host_tag = f"{host_tag} (local ok {local_age:.1f}s)"
        scale_str = "—" if host.perceived_scale is None else f"{float(host.perceived_scale):.3f}"
        imgui.text(
            "Host relay [%s]  uv=%s  scale=%s  label=%s"
            % (
                host_tag,
                _fmt_uv(host.perceived_center_uv),
                scale_str,
                str(host.perceived_object_label) or "(none)",
            )
        )
        if perc_active and local_age >= 0.0 and host_age > 0.75:
            imgui.text_wrapped(
                "  Perception is LIVE locally but host relay is stale. "
                "Gaze uses local UV; restart host.py/ctrl.py if relay stays stale."
            )

    gaze_tag = "RUNNING" if bool(st.gaze_running) else "OFF"
    imgui.text(
        "Gaze [%s]  mode=%s  updates=%d  u_err=%+.3f  v_err=%+.3f"
        % (
            gaze_tag,
            str(st.gaze_mode) or "idle",
            int(st.gaze_update_count),
            float(st.gaze_u_err),
            float(st.gaze_v_err),
        )
    )
    if bool(st.gaze_running):
        imgui.text(
            "  du roll/s1/s2=%+.4f / %+.4f / %+.4f  obs_age=%.2fs  target_uv=(%.2f, %.2f)"
            % (
                float(st.gaze_du_roll),
                float(st.gaze_du_s1),
                float(st.gaze_du_s2),
                float(st.gaze_obs_age_s),
                float(st.visual_target_uv_u),
                float(st.visual_target_uv_v),
            )
        )
    if str(st.gaze_status_msg).strip():
        imgui.text_wrapped(f"  {st.gaze_status_msg}")

    needed = (
        not st.perception_running
        or st.perception_center_uv is None
        or (host is not None and host.perceived_center_uv is None)
    )
    if bool(st.gaze_running) and needed:
        imgui.text_wrapped(
            "  Tip: Gaze needs Perception Start + host UV relay. "
            "Check sim camera, target label, and that a target is visible."
        )

    imgui.separator()


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
