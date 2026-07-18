"""Typed operator/controller intent surface carried over protocol v3."""

SERVICE_CALLS = frozenset(
    {
        "apply_partial_control_u", "capture_perception_frame", "disconnect_device",
        "extend_arm_controls", "home_controls", "load_sag_model", "refresh_host_state",
        "refresh_perception_capture", "request_ports", "reset_simulation", "send_claw_command",
        "send_current_target_meta", "send_go2_obstacles_avoid", "send_go2_sport_pose",
        "send_go2_velocity", "send_ready_pose_meta", "send_sag_model_meta", "send_sim_target_xyz",
        "set_device", "set_display_offset", "start_demo4_stop_and_grasp",
        "start_gaze_stabilizer_standing", "start_gaze_stabilizer_walking", "start_ik_solve",
        "start_lji_grasp_only", "start_mobile_gaze_lji_pick_e2e", "start_perception_capture",
        "stop_gaze_stabilizer", "stop_perception_capture", "stop_pick_e2e",
        "toggle_perception_recording", "torque_off", "torque_on",
        "update_gaze_stabilizer_config", "update_perception_config",
        "send_sim_camera_input", "select_endpoint",
    }
)

SERVICE_VALUES = frozenset(
    {
        "_gaze_cfg", "_pick_config_effective", "control_mapping", "current_control_u",
        "current_host_state", "gaze_config", "has_client", "pick_e2e_running",
        "available_endpoints", "active_endpoint",
    }
)

STATE_CALLS = frozenset(
    {
        "clear_ik_status", "offset_values", "set_claw_closed", "set_mock_object_preferred_dir",
        "set_mock_object_world_xyz", "set_paused", "set_perception_record_overlay", "set_target",
        "set_target_dir", "set_torque_lock_bypass",
    }
)

OPERATOR_OPERATIONS = frozenset({"snapshot", "service_call", "service_get", "state_call", "state_set"})
