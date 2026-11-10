import numpy as np
from workbench.research.experiments.hug_ga.search import seed_population, decode, breed, quality


def test_paths_obey_home_limits_and_slew():
    rng=np.random.default_rng(4)
    lo=np.array([-.173,-1.57,-.628,-.628]);hi=np.array([0,1.57,.628,.628])
    start=np.array([-.12,0,-.59,.59]);goal=np.array([0,-1.57,-.31,-.56])
    rates=np.array([.005,.1,.03,.03])
    pop=seed_population(32,6,start,goal,lo,hi,rng)
    paths=decode(pop,start,lo,hi,rates,200)
    np.testing.assert_allclose(paths[:,0],np.tile(start,(32,1)))
    assert np.all(paths>=lo-1e-12) and np.all(paths<=hi+1e-12)
    assert np.all(abs(np.diff(paths,axis=1))<=rates+1e-12)


def report(**changes):
    value=dict(fault=False,wrong_face_any=False,success=False,hold_fraction=0.,lift_fraction=0.,grasp_fraction=0.,terminal_distance_m=.1,duration_s=20.)
    return dict(value,**changes)


def test_real_success_outranks_progress_and_forbidden_success():
    success=report(success=True)
    assert quality(success)>quality(report(hold_fraction=.99,lift_fraction=1,grasp_fraction=1,terminal_distance_m=0))
    assert quality(report())>quality(report(success=True,fault=True))
    assert quality(report())>quality(report(success=True,wrong_face_any=True))
    assert quality(report(success=True,duration_s=10))>quality(success)


def test_breed_keeps_elite_and_is_reproducible():
    population=np.random.default_rng(2).random((16,6,5))
    reports=[report() for _ in population];reports[7]=report(success=True)
    a=breed(population,reports,np.random.default_rng(8))
    b=breed(population,reports,np.random.default_rng(8))
    np.testing.assert_array_equal(a,b);np.testing.assert_array_equal(a[0],population[7])
    assert np.all((a>=0)&(a<=1))
