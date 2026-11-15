"""Bounded parent workflow. Ports own sensors/planners; tick alone starts motion."""
import uuid
import logging
from .contracts import Settings
from .ownership import Ownership


class Coordinator:
    TERMINAL={'idle','ready','completed','unverified','failed','cancelled'}
    def __init__(self,ports,*,settings=None,clock):
        self.ports=ports;self.s=settings or Settings();self.clock=clock
        self.ownership=Ownership();self.phase='idle';self.reason='';self.plan_id=''
        self.current=None;self.grant=None;self.generation=0;self.events=[]
        self.reobservations=0;self.corrections=0;self.searches=0
        self.cancel_reason=None
    @property
    def active(self):return self.phase not in self.TERMINAL
    def status(self):
        return dict(phase=self.phase,active=self.active,reason=self.reason,plan_id=self.plan_id,
            reobservations=self.reobservations,corrections=self.corrections,searches=self.searches,
            events=self.events[-24:],**self.ports.summary())
    def plan(self,target_id='target',task_profile='cylinder-d100'):
        if self.active:raise RuntimeError('mobile task is active')
        if task_profile!='cylinder-d100':raise ValueError('unsupported mobile task profile')
        self.ports.preflight(target_id)
        self.target_id=target_id;self.plan_id=uuid.uuid4().hex;self.phase='ready';self.reason=''
        self.route=self.ports.route();self.context=self.ports.context();self.prepared=self.clock()
        return self.status()
    def start(self,plan_id):
        if self.phase!='ready' or plan_id!=self.plan_id:raise RuntimeError('no matching mobile plan')
        if self.clock()-self.prepared>120 or self.route!=self.ports.route() or self.context!=self.ports.context():
            raise RuntimeError('mobile plan context expired')
        self.ports.preflight(self.target_id)
        self.cancel_reason=None
        self.ownership.acquire(self.route);self.generation+=1;self.started=self.clock()
        self.reobservations=self.corrections=self.searches=0;self.events=[]
        try:self._begin('acquire',{'arm'})
        except Exception as exc:
            self._end('failed',str(exc));raise
        return self.status()
    def _begin(self,phase,channels,**args):
        self.phase=phase;self.events.append(dict(phase=phase,time_s=self.clock()-self.started))
        self.grant=self.ownership.grant(phase,set(channels)|{'arm'})
        self.current=self.ports.begin(phase,self.grant,self.ownership,**args)
    def _next(self,result):
        phase=self.phase
        if phase in {'acquire','reacquire','post_scan_acquire'}:
            if result.get('visible'):
                if phase=='acquire':self._begin('plan_observation',set())
                elif phase=='post_scan_acquire':self._begin('plan_grasp',set())
                else:self._begin('check_grasp',set())
            elif phase=='reacquire':
                self.reobservations+=1
                if self.reobservations>self.s.max_reobservations:raise RuntimeError('final_target_reacquisition_failed')
                self._begin('plan_observation',set())
            elif self.searches<self.s.search_limit:
                self.searches+=1;self._begin('search',{'arm'},index=self.searches)
            else:raise RuntimeError('target_not_found')
        elif phase=='search':self._begin('acquire',{'arm'})
        elif phase=='plan_observation':
            if result.get('needs_mapping'):
                self.searches+=1
                if self.searches>self.s.search_limit:raise RuntimeError('no_observed_free_base_path')
                self._begin('map_search',{'arm'},index=self.searches)
            else:
                self.observation=result['goal'];self._begin('transport',{'arm'},purpose='observation')
        elif phase=='map_search':self._begin('acquire',{'arm'})
        elif phase=='transport':
            purpose=result['purpose']
            if purpose=='observation':self._begin('move_observation',{'arm','base'},goal=self.observation)
            else:self._begin('move_grasp',{'arm','base'},goal=self.grasp)
        elif phase=='move_observation':self._begin('aim_observation',{'arm'},goal=self.observation)
        elif phase=='aim_observation':self._begin('scan',{'arm'})
        elif phase=='scan':
            if not result['quality_ok']:
                self.reobservations+=1
                if self.reobservations>self.s.max_reobservations:raise RuntimeError('observation_quality_insufficient')
                self._begin('plan_observation',set())
            else:self._begin('post_scan_acquire',{'arm'})
        elif phase=='plan_grasp':
            self.grasp=result['goal'];self._begin('transport',{'arm'},purpose='grasp')
        elif phase=='move_grasp':self._begin('reacquire',{'arm'})
        elif phase=='check_grasp':
            if result['accepted']:self._begin('hug',{'arm'})
            else:
                self.corrections+=1
                if self.corrections>self.s.max_corrections:raise RuntimeError('grasp_localization_uncertain')
                self._begin('plan_grasp',set())
        elif phase=='hug':self._begin('verify',set())
        elif phase=='verify':
            self.phase='completed' if result.get('verified') else 'failed' if result.get('evaluated_failure') else 'unverified'
            self.reason=result.get('reason','');self.plan_id='';self.ports.stop('finished');self.ownership.release()
        else:raise RuntimeError('unknown mobile phase '+phase)
    def tick(self):
        if not self.active:return
        if self.cancel_reason is not None:
            reason,self.cancel_reason=self.cancel_reason,None
            self._end('cancelled',reason);return
        try:
            if self.route!=self.ports.route() or self.context!=self.ports.context():raise RuntimeError('mobile route/scene changed')
            if self.clock()-self.started>self.s.total_timeout_s:raise RuntimeError('mobile task timeout')
            self.ports.guard()
            self.ownership.require(self.grant,self.route,'arm')
            result=self.ports.poll(self.current)
            if result is None:return
            self.ownership.finish(self.grant);self.grant=None;self.current=None
            self._next(result)
        except Exception as exc:
            logging.getLogger(__name__).exception('Mobile manipulation failed during %s',self.phase)
            self._end('failed',str(exc))
    def _end(self,phase,reason):
        try:self.ports.stop(reason)
        except Exception as exc:reason=str(reason)+'; stop delivery: '+str(exc)
        finally:
            self.generation+=1;self.phase=phase;self.reason=str(reason)[:512];self.plan_id=''
            self.ownership.release();self.current=None;self.grant=None
        return self.status()
    def cancel(self,reason='operator_cancelled'):
        if self.phase=='idle':return self.status()
        return self._end('cancelled',reason)
    def request_cancel(self, reason='operator_cancelled'):
        self.cancel_reason=str(reason)
    def guard_operator(self,operation,name):
        if not self.active or operation=='view_snapshot':return
        if operation=='service_call' and name in {'cancel_mobile_manipulation','torque_off','reset_simulation'}:
            self.cancel(name);return
        raise RuntimeError('mobile manipulation owns motion; stop it before other changes')
    def allow_motion(self,payload):
        if not self.active:return True
        if payload.get('command') in {'estop','torque_off'}:
            self.cancel(payload['command']);return True
        return False
