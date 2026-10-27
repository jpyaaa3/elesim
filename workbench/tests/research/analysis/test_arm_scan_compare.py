import copy

from workbench.research.analysis.arm_scan_compare import compare


def records():
    rows=[{'episode':i,'kind':'box','views_attempted':20,'accepted':False,
           'fused':{'recall_5mm':.55,'surface_p95_mm':3.,'dimension_error_mm':[2.,3.,4.]}}
          for i in range(6)]
    return {'episodes':rows,'collision_audit':{'accepted':True}}


def test_fewer_views_cannot_hide_a_worse_shape():
    baseline=records();candidate=copy.deepcopy(baseline)
    for row in candidate['episodes']:row['views_attempted']=5
    assert compare(baseline,candidate)['accepted']
    candidate['episodes'][2]['fused']['recall_5mm']=.48
    result=compare(baseline,candidate)
    assert not result['accepted']
    assert result['capture_reduction_fraction']==.75
    assert not result['episodes'][2]['paired_quality_preserved']


def test_success_requires_actual_motion_collision_audits():
    baseline=records();candidate=copy.deepcopy(baseline)
    for row in candidate['episodes']:row['views_attempted']=5
    candidate['collision_audit']['accepted']=False
    assert not compare(baseline,candidate)['accepted']
