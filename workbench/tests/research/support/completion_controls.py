"""Regression fixture extracted from a retired experiment; no standalone CLI."""


from __future__ import annotations


import numpy as np


from scipy.spatial import cKDTree


from workbench.research.experiments.adapointr import CloudFrame,normalize_partial,restore_completion,sample_input


def prepare_control(points,variant,*,seed=0):
    points=np.asarray(points,dtype=np.float64)
    _,normalized_frame=normalize_partial(points)
    if variant=='native':frame=CloudFrame(np.zeros(3),1.)
    elif variant=='center_only':frame=CloudFrame(normalized_frame.center,1.)
    elif variant=='scale_only_r05':frame=CloudFrame(np.zeros(3),normalized_frame.meters_per_unit)
    elif variant=='center_scale_r05':frame=normalized_frame
    elif variant=='center_scale_r10':frame=CloudFrame(normalized_frame.center,normalized_frame.meters_per_unit/2)
    else:raise ValueError('unknown preprocessing control')
    transformed=(points-frame.center)/frame.meters_per_unit
    return sample_input(transformed,seed=seed),frame


def comparison_metrics(observed,predicted,reference):
    observed,predicted,reference=[np.asarray(v,dtype=np.float64) for v in (observed,predicted,reference)]
    if any(v.ndim!=2 or v.shape[1]!=3 or not len(v) or not np.all(np.isfinite(v)) for v in (observed,predicted,reference)):
        raise ValueError('invalid comparison cloud')
    diagonal=float(np.linalg.norm(np.ptp(observed,axis=0)))
    if diagonal<1e-9:raise ValueError('degenerate observed extent')
    predicted_tree=cKDTree(predicted)
    fidelity=predicted_tree.query(observed)[0]
    to_reference=cKDTree(reference).query(predicted)[0]
    from_reference=predicted_tree.query(reference)[0]
    symmetric=float((to_reference.mean()+from_reference.mean())/2)
    p95=float(np.percentile(fidelity,95))
    return {'input_diagonal':diagonal,'observed_to_prediction_mean':float(fidelity.mean()),
            'observed_to_prediction_p95':p95,'observed_to_prediction_p95_pct_diag':100*p95/diagonal,
            'reference_chamfer_mean':symmetric,'reference_chamfer_mean_pct_diag':100*symmetric/diagonal,
            'reference_distance_p95_pct_diag':100*float(np.percentile(np.concatenate([to_reference,from_reference]),95))/diagonal,
            'reference_centroid_shift_pct_diag':100*float(np.linalg.norm(predicted.mean(0)-reference.mean(0)))/diagonal}
