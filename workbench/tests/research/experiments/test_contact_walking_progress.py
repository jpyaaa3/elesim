from workbench.research.debug.profile_sim_walking import has_commanded_progress


def test_stationary_or_opposite_motion_cannot_pass_a_walking_trial():
    assert not has_commanded_progress((.8, 0, 0), [0, 0, 0], duration=6)
    assert not has_commanded_progress((.8, 0, 0), [-1, 0, 0], duration=6)
    assert not has_commanded_progress((0, -.35, 0), [1, 0, 0], duration=6)


def test_forward_back_left_right_need_progress_along_command():
    for command in [(0.8, 0, 0), (-0.8, 0, 0), (0, .35, 0), (0, -.35, 0)]:
        assert has_commanded_progress(command, [command[0], command[1], 0], duration=6)
