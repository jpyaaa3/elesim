"""Diameter routing with synthetic geometry/policies; not physics evidence."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from elesim_sim.rl.trajectory import Trajectory
from workbench.research.experiments.hug_c import collection, configure as cfg
from workbench.tests.research.experiments.hug_c_fixtures import with_preparation


@pytest.fixture
def prepared(tmp_path):
    # Embedded DATA deliberately throws. The toy linear XY map is not robot geometry.
    mapper=tmp_path/'mapper.html'
    mapper.write_text(with_preparation('''const DATA=(()=>{throw Error("stored points are forbidden")})();
const rot=()=>{};
function model(q3,q4){if(q3<-25||q4<-25)return {candidates:[]};return {candidates:[{x:q3,y:200-q4,mu:.2,kind:'side',branch:-1}]};}
let current=[];
const CONTACT_OVERLAP_TOL_MM=1;
function collisionGeometry(){return [];}
function selfCollision(){return false;}
function objectCollision(){return false;}
function inverseContactPose(x,y,D,roll,seed){
 const q3=x,q4=200-y;return {q3,q4,c:model(q3,q4).candidates[0]};
}
function muLimit(){}'''))
    output=tmp_path/'collection'
    plan=collection.prepare_collection(mapper,output,step=10,validation=3,test=4)
    return mapper,output,plan


def target(plan):
    profile=plan['profiles'][2]
    q=np.mean(profile['q_triangle'],axis=0)
    return [float(q[0]),float(200-q[1])]


def test_five_independent_diameters_are_only_prepared_not_trained(prepared):
    _,output,plan=prepared
    assert [p['diameter_mm'] for p in plan['profiles']]==list(collection.DIAMETERS)
    assert not list(output.rglob('model_*.pt')) and not list(output.rglob('study'))
    ids=[]
    for p in plan['profiles']:
        pilot=json.loads((output/p['pilot']).read_text())
        assert not pilot['source']['point_data_used']
        assert len([c for c in pilot['cases'] if c['split']=='train'])==3
        assert all(c['diameter_mm']==p['diameter_mm'] for c in pilot['cases'])
        ids.extend(c['case_id'] for c in pilot['cases'])
    assert len(ids)==len(set(ids))


def test_adding_110_keeps_existing_profiles_npz_and_code_hash_unchanged(prepared):
    mapper,output,plan=prepared
    old=copy.deepcopy(plan['profiles']);before=cfg.digest_code()
    artifact=output/'d100'/'already-collected.npz'
    np.savez_compressed(artifact,example=np.arange(10))
    bytes_before=artifact.read_bytes()
    collection.prepare_collection(mapper,output,diameters=[110],step=10,validation=3,test=4)
    updated=collection.load_collection(output/'collection.json')
    assert updated['profiles'][:5]==old and updated['profiles'][5]['diameter_mm']==110
    assert artifact.read_bytes()==bytes_before and cfg.digest_code()==before
    unchanged=(output/'collection.json').read_bytes()
    collection.prepare_collection(mapper,output,diameters=[110],step=10,validation=3,test=4)
    assert (output/'collection.json').read_bytes()==unchanged


def test_exact_xy_inverse_and_q_weights_require_matching_diameter(prepared):
    mapper,output,plan=prepared;index=output/'collection.json';xy=target(plan)
    result=collection.resolve(index,mapper,xy,150,require_trained=False)
    np.testing.assert_allclose(result['xy_mm'],xy)
    np.testing.assert_allclose(result['weights'],[1/3]*3)
    assert result['readiness']=='geometry-only' and result['candidate_requires_physics_validation']
    with pytest.raises(ValueError,match='no trained study'):
        collection.resolve(index,mapper,xy,150)
    with pytest.raises(ValueError,match='not collected'):
        collection.resolve(index,mapper,xy,110,require_trained=False)
    with pytest.raises(ValueError,match='No valid C solution'):
        collection.resolve(index,mapper,[-400,0],150,require_trained=False)


def synthetic_finished_study(output,profile):
    # These arrays/flags test artifact routing, not trained policies or real success.
    root=output/profile['study'];root.mkdir()
    exp=cfg.configure(output/profile['pilot']);cfg.write_json(root/'experiment.json',exp)
    cfg.write_json(root/'status.json',dict(phase='finished'))
    for filename,total in (('validation-report',3),('position-robustness',48),('test-report',4)):
        cfg.write_json(root/(filename+'.json'),dict(schema='elesim-c-hug-evaluation',
                       simulation_context=exp['simulation_context'],total=total,successes=total))
    for c in exp['cases'][:3]:
        metadata=dict(c,simulation_context=exp['simulation_context'],control_mode=cfg.CONTROL_MODE,duration_s=12.)
        t=np.arange(1201)*.01
        commands=np.tile(c['start_command'],(len(t),1))
        trace=Trajectory(t,commands,cfg.CHANNELS,metadata,
                         dict(replay_success=True,physics_validated=True,replay_repetitions=3))
        trace.save(root/'anchors'/(c['case_id']+'.npz'))
        folder=root/'policies';folder.mkdir(exist_ok=True)
        n=len(cfg.OBSERVATIONS)
        np.savez_compressed(folder/(c['case_id']+'.npz'),schema=np.asarray('elesim-c-hug-policy'),
                            schema_version=np.asarray(cfg.POLICY_VERSION),observation_names=np.asarray(cfg.OBSERVATIONS),
                            action_names=np.asarray(cfg.ACTIONS),metadata_json=np.asarray(json.dumps(metadata)),
                            norm_mean=np.zeros(n),norm_std=np.ones(n),norm_eps=np.asarray(1e-8),
                            n_layers=np.asarray(1),w0=np.zeros((4,n)),b0=np.zeros(4))
    return root


def test_selects_three_policy_paths_only_after_all_evaluation_gates(prepared):
    mapper,output,plan=prepared
    root=synthetic_finished_study(output,plan['profiles'][2])
    result=collection.resolve(output/'collection.json',mapper,target(plan),150)
    assert result['readiness']=='validated-anchor-selection'
    assert len(result['anchors'])==3
    assert all(Path(a['policy_npz']).is_file() and Path(a['trajectory_npz']).is_file() for a in result['anchors'])
    report=root/'position-robustness.json';bad=json.loads(report.read_text());bad['successes']=2
    report.write_text(json.dumps(bad))
    with pytest.raises(ValueError,match='evaluation evidence'):
        collection.resolve(output/'collection.json',mapper,target(plan),150)


def test_collection_run_stops_at_first_failure_and_defaults_to_512(prepared,monkeypatch):
    _,output,_=prepared;calls=[]
    def fail(argv):
        calls.append(argv);return SimpleNamespace(returncode=1)
    monkeypatch.setattr(collection.subprocess,'run',fail)
    assert collection.run_collection(output/'collection.json')==1
    assert len(calls)==1 and calls[0][calls[0].index('--num-envs')+1]=='512'
    assert calls[0][calls[0].index('--pilot')+1].endswith('d100/pilot.json')


def test_collection_rejects_modified_inputs(prepared):
    _,output,plan=prepared
    pilot=output/plan['profiles'][0]['pilot'];pilot.write_text(pilot.read_text()+' ')
    with pytest.raises(ValueError,match='input changed'):
        collection.load_collection(output/'collection.json')


def test_collection_check_runs_physics_only_without_optimizer(prepared,monkeypatch):
    _,output,_=prepared;calls=[]
    def check(argv,**kwargs):
        calls.append(argv)
        cfg.write_json(Path(argv[argv.index('--output')+1]),dict(ready=True,physics_tested=True,preparation_tested=True))
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(collection.subprocess,'run',check)
    destination=output/'checks'
    assert collection.check_collection(output/'collection.json',destination)==0
    assert len(calls)==5 and all('check' in c and '--physics' in c and 'train' not in c for c in calls)
    report=json.loads((destination/'report.json').read_text())
    assert report['ready'] and not report['training_started']


def test_catalog_screen_routes_all_diameters_without_learning(prepared,monkeypatch):
    _,output,_=prepared;calls=[]
    def screen(argv,**kwargs):
        calls.append(argv)
        destination=Path(argv[argv.index('--output')+1])
        cfg.write_json(destination/'report.json',dict(results=[dict(static_ready=True),dict(static_ready=False)]))
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(collection.subprocess,'run',screen)
    assert collection.screen_collection(output/'collection.json',output/'screens')==0
    assert len(calls)==5 and all('workbench.research.experiments.hug_c.catalog_preflight' in c and 'train' not in c for c in calls)


def test_triangle_is_selected_after_physical_catalog_filter(prepared):
    mapper,output,plan=prepared;screened=output/'screens'
    ready=[[-25.,-25.],[-15.,-25.],[-25.,-15.]]
    original=(output/'collection.json').read_bytes()
    for profile in plan['profiles']:
        pilot=json.loads((output/profile['pilot']).read_text())
        report=dict(schema='elesim-c-hug-catalog-preflight',schema_version=3,source=pilot['source'],
                    diameter_mm=profile['diameter_mm'],step_deg=10.,mu_max=.4,
                    simulation_context=dict(model_sha256=cfg.asset_digest(),settings=cfg.DEFAULTS),
                    results=[dict(q_deg=q,static_ready=True) for q in ready])
        cfg.write_json(screened/f'd{profile["diameter_mm"]:g}'/'report.json',report)
    destination=output/'reselected'
    new=collection.prepare_collection(mapper,destination,step=10,validation=3,test=4,catalog_preflights=screened)
    assert new['selection']['static_catalog_screened']
    for profile in new['profiles']:
        pilot=json.loads((destination/profile['pilot']).read_text())
        assert profile['feasible_samples']==3
        assert {tuple(c['q_deg']) for c in pilot['cases'][:3]}==set(map(tuple,ready))
        assert all(c['preparation_passed'] for c in pilot['cases'])
    assert (output/'collection.json').read_bytes()==original


def test_exact_xy_selector_rejects_an_excluded_physical_cell_inside_triangle(prepared):
    mapper,output,plan=prepared;profile=plan['profiles'][2];xy=target(plan)
    cell=tuple(np.floor((np.array([xy[0],200-xy[1]])+180)/10).astype(int))
    catalog_path=output/profile['catalog'];catalog=json.loads(catalog_path.read_text())
    catalog['rows']=[r for r in catalog['rows'] if tuple(np.floor((np.array(r['q_deg'])+180)/10).astype(int))!=cell]
    catalog_path.write_text(json.dumps(catalog));profile['catalog_sha256']=cfg.sha(catalog_path)
    pilot_path=output/profile['pilot'];pilot=json.loads(pilot_path.read_text())
    pilot['selection']['static_catalog_screened']=True
    pilot_path.write_text(json.dumps(pilot));profile['pilot_sha256']=cfg.sha(pilot_path)
    collection.save_plan(output/'collection.json',plan)
    with pytest.raises(ValueError,match='No valid C solution'):
        collection.resolve(output/'collection.json',mapper,xy,150,require_trained=False)
