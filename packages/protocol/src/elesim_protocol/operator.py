"""Typed operator/controller intent surface carried over protocol v4."""

OPERATOR_VIEW_SCHEMA_VERSION = 1

SERVICE_CALLS = frozenset(
    {
        "apply_partial_control_u", "capture_perception_frame", "disconnect_device",
        "extend_arm_controls", "home_controls", "load_sag_model", "refresh_host_state",
        "refresh_perception_capture", "request_ports", "reset_simulation", "send_claw_command",
        "send_current_target_meta", "send_go2_obstacles_avoid", "send_go2_sport_pose",
        "send_go2_velocity", "send_ready_pose_meta", "send_sag_model_meta", "send_sim_target_xyz",
        "send_planned_move_target",
        "set_device", "set_display_offset", "start_demo4_stop_and_grasp",
        "start_gaze_stabilizer_standing", "start_gaze_stabilizer_walking", "start_ik_solve",
        "start_lji_grasp_only", "start_mobile_gaze_lji_pick_e2e", "start_perception_capture",
        "start_planned_move_generate", "start_planned_move_generate_task_space", "start_planned_move_execute",
        "start_planned_move_preview",
        "stop_gaze_stabilizer", "stop_perception_capture", "stop_pick_e2e",
        "toggle_perception_recording", "torque_off", "torque_on",
        "update_gaze_stabilizer_config", "update_perception_config",
        "select_endpoint", "current_host_state",
        "has_client", "current_control_u", "control_mapping", "pick_e2e_running",
        "planned_move_status",
    }
)

SERVICE_VALUES = frozenset(
    {
        "gaze_config", "available_endpoints", "active_endpoint",
    }
)

STATE_CALLS = frozenset(
    {
        "clear_ik_status", "offset_values", "set_claw_closed", "set_mock_object_preferred_dir",
        "set_mock_object_world_xyz", "set_controls_locked", "set_perception_record_overlay", "set_target",
        "set_target_dir", "set_torque_lock_bypass",
    }
)

STATE_VALUES = frozenset(
    {
        "visual_center_tol",
        "visual_look_distance_m",
        "visual_ready_distance_m",
        "visual_scale_tol",
        "visual_target_label",
        "visual_target_scale",
        "visual_target_uv_u",
        "visual_target_uv_v",
    }
)

OPERATOR_OPERATIONS = frozenset(
    {"snapshot", "view_snapshot", "service_call", "service_get", "state_call", "state_set"}
)
