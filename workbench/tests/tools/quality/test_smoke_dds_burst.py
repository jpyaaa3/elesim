from workbench.tests.system import smoke_dds_burst


def test_missing_ros_is_a_failed_gate_not_a_successful_skip(monkeypatch,capsys):
    monkeypatch.setattr(smoke_dds_burst.importlib.util,'find_spec',lambda name:None)
    assert smoke_dds_burst.main()==2
    assert 'ROS 2 rclpy and generated elesim_interfaces are required' in capsys.readouterr().out
