"""Read native V2 validation subcomponents without modifying planner results."""
from collections import Counter
import math

import numpy as np


def clean(value):
    if isinstance(value, dict):
        return {name: clean(item) for name, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def explain(planner, solved):
    solver = planner.trajopt_solver
    limits = solver.kinematics.config.kinematics_config.joint_limits
    names = list(solver.kinematics.joint_names)
    bounds = {name: getattr(limits, name).detach().cpu().numpy()
              for name in ('position', 'velocity', 'acceleration', 'jerk', 'effort')}
    report = dict(joint_names=names, limits={name: values.tolist() for name, values in bounds.items()},
                  original_result_unchanged=True)
    if solved is not None:
        trajectory = solved.get_interpolated_plan()
        selected = {}
        if trajectory is not None:
            for field in ('position', 'velocity', 'acceleration', 'jerk'):
                tensor = getattr(trajectory, field, None)
                if tensor is None:
                    continue
                values = tensor.detach().cpu().numpy().reshape(-1, len(names))
                lower, upper = bounds[field]
                selected[field] = dict(minimum=values.min(0).tolist(), maximum=values.max(0).tolist(),
                    lower_excess=np.maximum(lower-values, 0).max(0).tolist(),
                    upper_excess=np.maximum(values-upper, 0).max(0).tolist(),
                    violating_samples_per_joint=np.count_nonzero(
                        (values < lower-1e-4) | (values > upper+1e-4), axis=0).tolist(),
                    exact_bound_violating_samples_per_joint=np.count_nonzero(
                        (values < lower) | (values > upper), axis=0).tolist(),
                    all_finite=bool(np.isfinite(values).all()))
        report['selected_interpolated_trajectory'] = selected
    timeopt = solver._timeopt
    if timeopt is not None:
        result = timeopt.optimizer._result
        statuses = result.status.detach().cpu().numpy().tolist()
        details = dict(
            note='Last internal timeopt pass across all seeds; not asserted to be the selected best seed',
            status_codes={'0': 'sampled_feasible', '1': 'no_feasible_uniform_initializer',
                          '2': 'output_validation_failure', '3': 'bad_input'},
            statuses=statuses, status_counts=dict(Counter(map(str, statuses))),
            success=result.success.detach().cpu().tolist(),
            duration_s=result.duration.detach().cpu().tolist(),
            uniform_duration_s=result.uniform_duration.detach().cpu().tolist(),
            numerical_ok=result.optimization_ok.detach().cpu().tolist(),
            interpolation_capacity=int(timeopt.config.interpolation_buffer_size),
            sampled_fields={})
        for field in ('velocity', 'acceleration', 'jerk', 'effort'):
            tensor = result.torque if field == 'effort' else getattr(result.joint_state, field)
            values = tensor.detach().cpu().numpy()
            lower, upper = bounds[field]
            details['sampled_fields'][field] = dict(
                absolute_maximum_per_joint=np.abs(values).max(axis=(0, 1)).tolist(),
                lower_excess_per_joint=np.maximum(lower-values, 0).max(axis=(0, 1)).tolist(),
                upper_excess_per_joint=np.maximum(values-upper, 0).max(axis=(0, 1)).tolist(),
                violating_lanes=np.any((values < lower-1e-4) | (values > upper+1e-4), axis=(1, 2)).tolist())
        report['last_internal_timeopt'] = details
    return clean(report)
