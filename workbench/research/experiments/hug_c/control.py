"""Four policy-controlled increments; no scheduled joint targets."""
import numpy as np


def action_rates(settings):
    # Public actions q3/q4 use total bend. Sim commands use radians per node.
    return np.array([settings['action_delta_linear_m'], settings['action_delta_roll_rad'],
                     settings['action_delta_bend_total_rad']/5,
                     settings['action_delta_bend_total_rad']/5])


def command_bounds(settings):
    return (np.array([settings['linear_min_m'], settings['roll_min_rad'],
                      -settings['bend_per_node_rad'], -settings['bend_per_node_rad']]),
            np.array([settings['linear_max_m'], settings['roll_max_rad'],
                      settings['bend_per_node_rad'], settings['bend_per_node_rad']]))


def advance_command(command, actions, settings, xp=np):
    """Shared NumPy/Torch calculation, tested without starting physics."""
    convert = command.new_tensor if hasattr(command, 'new_tensor') else np.asarray
    lower, upper = command_bounds(settings)
    return xp.clip(command + xp.clip(actions, -1, 1)*convert(action_rates(settings)),
                   convert(lower), convert(upper))


def pose_error(actual, target, settings, xp=np):
    convert = actual.new_tensor if hasattr(actual, 'new_tensor') else np.asarray
    scale = convert([settings['linear_max_m']-settings['linear_min_m'],
                     settings['roll_max_rad']-settings['roll_min_rad'],
                     settings['bend_per_node_rad'], settings['bend_per_node_rad']])
    delta = (actual-target)/scale
    return xp.sqrt((delta*delta).sum(-1))
