"""Atomic stage graph with artifact verification and single-GPU process ownership."""
from pathlib import Path
from contextlib import contextmanager
import fcntl
import json
import os
import subprocess
import sys
import time
import numpy as np
from .contracts import save,canonical,sha,files_hashes,verify_files,load_study


class StageFailed(RuntimeError):
    def __init__(self,key,result):self.key,self.result=key,result;super().__init__(key+': '+str(result))


@contextmanager
def lock(root):
    with (root/'run.lock').open('a') as file:
        try:fcntl.flock(file,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('Study already has a coordinator') from None
        yield


class Coordinator:
    def __init__(self,root):
        self.root=Path(root).resolve();self.plan=load_study(self.root)
        path=self.root/'manifest.json'
        self.state=json.loads(path.read_text()) if path.exists() else dict(schema='hug-pipeline-state',version=1,plan_sha256=sha(self.root/'plan.json'),stages={},excluded=[],status='planned')
        if self.state['plan_sha256']!=sha(self.root/'plan.json'):raise ValueError('Study plan changed')

    def write(self):save(self.root/'manifest.json',self.state)

    def stage(self,key,kind,**kwargs):
        payload=dict(root=str(self.root),kind=kind,**kwargs)
        ident=canonical(payload);old=self.state['stages'].get(key)
        if old and old['input_sha256']!=ident:raise ValueError('Stage inputs changed: '+key)
        if old and old['status']=='complete':
            verify_files(Path(old['output']),old['artifacts']);return Path(old['output'])
        if old and old['status']=='failed':raise StageFailed(key,old['result'])
        # A killed child is never assumed finished or killed by an unrelated PID.
        # Resume training from a paired checkpoint; other interrupted stages use
        # a fresh attempt directory, keeping the partial evidence intact.
        if old and old.get('pid'):
            proc=Path('/proc')/str(old['pid'])/'cmdline'
            if proc.exists() and b'hug_pipeline.worker' in proc.read_bytes():
                raise RuntimeError('Previous worker still alive; inspect before resuming')
        attempt=old['attempt']+1 if old else 1
        output=self.root/'stages'/key/f'attempt-{attempt:03d}'
        output.parent.mkdir(parents=True,exist_ok=True)
        jobfile=output.parent/f'job-{attempt:03d}.json';resultfile=output.parent/f'result-{attempt:03d}.json'
        job=dict(payload,output=str(output),result=str(resultfile))
        if kind=='train' and old:
            checkpoint=self.latest_checkpoint(Path(old['output']))
            if checkpoint:
                job['resume']=str(checkpoint)
                side=json.loads(Path(str(checkpoint)+'.curriculum.json').read_text())
                # schedule step counts one vectorized PPO environment step.
                used=int(side['curriculum']['steps'])//16
                job['iterations']=max(0,self.plan['budgets']['iterations']-used)
        save(jobfile,job);started=time.time()
        record=dict(status='running',attempt=attempt,input_sha256=ident,output=str(output),started_unix=started,
            gpu=os.environ.get('CUDA_VISIBLE_DEVICES'),coordinator_pid=os.getpid())
        self.state['stages'][key]=record;self.state['status']='running';self.state['current_stage']=key;self.write()
        log=output.parent/f'attempt-{attempt:03d}.log'
        with log.open('w') as stream:
            child=subprocess.Popen([sys.executable,'-u','-m','workbench.research.experiments.hug_pipeline.worker',str(jobfile)],stdout=stream,stderr=subprocess.STDOUT)
            record['pid']=child.pid;self.write()
            try:code=child.wait()
            except BaseException:
                child.terminate()
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:child.kill();child.wait()
                record['status']='interrupted';self.state['status']='interrupted';self.write();raise
        result=json.loads(resultfile.read_text()) if resultfile.exists() else dict(accepted=False,status='worker_disappeared',returncode=code)
        record.update(elapsed_s=time.time()-started,result=result)
        if code==0 and result.get('accepted') is True:
            record.update(status='complete',artifacts=files_hashes(output));self.write();return output
        record['status']='failed' if result.get('status') not in ('execution_error','worker_disappeared') else 'interrupted'
        self.state['status']=record['status'];self.write();raise StageFailed(key,result)

    @staticmethod
    def latest_checkpoint(output):
        files=[]
        for p in output.rglob('model_*.pt'):
            side=Path(str(p)+'.curriculum.json')
            if side.exists():
                d=json.loads(side.read_text())
                if d.get('checkpoint_sha256')==sha(p):files.append((int(d['curriculum']['steps']),p))
        return max(files,key=lambda t:t[0])[1] if files else None

    def run(self,stop_after=None):
        from .planning import choose
        screen=self.stage('catalog-screen','screen')
        if stop_after=='screen':return
        for selection in range(len(self.state['excluded']),self.plan['budgets']['max_reselections']):
            try:
                pilot,cases=choose(self.root,self.plan,self.state['excluded'],screen/'report.json')
            except ValueError as exc:
                self.state.update(status='no_viable_triangle_at_resolution',error=str(exc));self.write();raise
            save(self.root/'selection.json',dict(pilot=pilot,cases=cases))
            self.state['selection']=selection;self.write();anchors=cases[:3];ready=[];retry=False
            for case in anchors:
                key=case['case_id']
                try:
                    reverse=self.stage(key+'/reverse','reverse',case=case)
                    if stop_after=='reverse':return
                except StageFailed as exc:
                    # Only exhausted geometry/reference searches justify selecting
                    # another anchor. Infrastructure errors remain resumable.
                    if exc.result.get('status') in ('initial_grasp_invalid','budget_exhausted','verification_failed'):
                        self.state['excluded'].append(case['q_deg']);self.write();retry=True;break
                    raise
                ppo=self.stage(key+'/ppo-plan','ppo-plan',case=case,reverse=str(reverse))
                self.stage(key+'/check','check',case=case,plan=str(ppo))
                if stop_after=='check':return
                if stop_after=='teachers':continue
                trained=self.stage(key+'/train','train',case=case,plan=str(ppo))
                if stop_after=='train':return
                policy=self.stage(key+'/policy','policy',case=case,policy=str(trained/'training/policy.npz'))
                original=self.stage(key+'/replay','replay',case=case,trajectory=str(policy/'rollout-1.npz'))
                ready.append(dict(case=case,policy=str(trained/'training/policy.npz'),trajectory=str(original/'rollout-1.npz')))
                if stop_after=='policy':return
            if retry:continue
            if stop_after=='teachers':return
            self.interpolate(ready,cases[3:]);return
        self.state['status']='selection_budget_exhausted';self.write()
        raise RuntimeError('Anchor selection budget exhausted; not an impossibility proof')

    def interpolate(self,anchors,held):
        from .evaluation import align,blend,alignment_events
        from workbench.research.experiments.hug_c.source import read_kernel,evaluate
        from .contracts import make_case
        paths=[]
        for a in anchors:
            with np.load(a['trajectory'],allow_pickle=False) as raw:paths.append({k:raw[k].copy() for k in ('commands','events')})
        aligned,events=align(paths,self.plan);folder=self.root/'alignment';folder.mkdir(exist_ok=True)
        save(folder/'alignment.json',dict(schema='hug-phase-alignment',version=2,
            boundaries=['start','first_grasp_or_clearance','both_grasp_and_clearance','final_hold','end'],
            common_events=events,physical_replay_required=True,
            anchors=[dict(case_id=a['case']['case_id'],raw_events=p['events'].tolist(),
                source_boundaries=alignment_events(p['events'],len(p['commands'])).tolist())
                for a,p in zip(anchors,paths)]))
        for a,commands in zip(anchors,aligned):
            file=folder/(a['case']['case_id']+'.npz')
            if not file.exists():np.savez_compressed(file,commands=commands,events=events)
            else:
                with np.load(file,allow_pickle=False) as raw:
                    if not np.array_equal(raw['commands'],commands):raise ValueError('Alignment changed')
            self.stage(a['case']['case_id']+'/aligned','aligned',case=a['case'],trajectory=str(file))
        vertices=np.array([a['case']['q_deg'] for a in anchors]);kernel,_=read_kernel(self.root/'mapper.html',require_preparation=True)
        probes=[(vertices[i]+vertices[(i+1)%3])/2 for i in range(3)]+[vertices.mean(0)]
        rows=evaluate(kernel,self.plan['diameter_mm'],[p.tolist() for p in probes],mu_max=self.plan['mu_max'])
        extras=[make_case(dict(r,split='probe'),self.plan['diameter_mm'],self.plan['home']) for r in rows if r]
        results=[]
        for target in [*extras,*held]:
            commands,w=blend(aligned,vertices,target['q_deg'],self.plan);file=folder/(target['case_id']+'.npz')
            if not file.exists():np.savez_compressed(file,commands=commands,weights=w,events=events)
            try:
                result=self.stage(target['case_id']+'/interpolation','interpolation',case=target,trajectory=str(file));accepted=True
            except StageFailed as exc:
                if exc.result.get('status') in ('execution_error','worker_disappeared'):raise
                accepted=False
            results.append(dict(case=target,accepted=accepted,weights=w.tolist(),trajectory=str(file)))
            save(self.root/'interpolation.json',dict(results=results,complete=False))
        accepted=all(r['accepted'] for r in results)
        save(self.root/'interpolation.json',dict(results=results,complete=True,accepted=accepted,
            invalid_geometric_probes=sum(r is None for r in rows),continuous_region_proven=False))
        for anchor in anchors:
            anchor['policy_sha256']=sha(anchor['policy'])
            anchor['aligned_sha256']=sha(folder/(anchor['case']['case_id']+'.npz'))
        publication=dict(schema='hug-pipeline-publication',version=1,ready=accepted,anchors=anchors,
            shared_context=self.plan['shared_context'],alignment=str(folder),events=events,
            diameter_mm=self.plan['diameter_mm'],evaluation_sha256=sha(self.root/'interpolation.json'),
            interpolation='phase-aligned timed commands; open-loop candidate',continuous_success_guaranteed=False)
        save(self.root/'publication.json',publication)
        self.state['publication_sha256']=sha(self.root/'publication.json')
        self.state['status']='published' if accepted else 'interpolation_failed';self.write()
