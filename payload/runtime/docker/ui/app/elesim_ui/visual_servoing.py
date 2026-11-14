"""Presentation and explicit intents for the Visual Servoing workspace.

Pilot owns scanning/planning/motion. This object only reads operator caches and
submits existing operator or UI-Sim commands after an explicit user action.
It never advances a robot workflow from a render tick or a response callback.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time


STEPS = ('Scene', 'Scan', 'Reconstruction', 'Action')
COVERAGE_NOTE = 'View diversity is a proxy. Hidden surface completeness is unverified.'


@dataclass(frozen=True)
class ServoingView:
    endpoint: str
    role: str
    pilot_online: bool
    scan: dict
    sim_status: object
    mock_object: object
    busy: bool
    reconstructed: bool
    coverage_note: str = COVERAGE_NOTE
    grasp_verified: bool = False


class ServoingController:
    """Keep form drafts, pending acknowledgements and context-bound results."""

    def __init__(self, panel):
        self.panel = panel
        self.step = 0
        self.target_id, self.max_views = 'target', 12
        self.asset_id = 'demo_box'
        self.position, self.euler_deg = [.5, 0., .4], [0., 0., 0.]
        self.scene_reference = ''
        self.strategy = 0
        self.notice = self.error = ''
        self.solution = {}
        self.hug_plan = {}
        self._hug_context = None
        self._solution_context = ()
        self._pending = None
        self._serial = 0
        self.scene_outcome = ''
        self._applied = ()
        self._applied_epoch = -1

    @property
    def service(self):
        return self.panel.service

    @property
    def session(self):
        return getattr(getattr(self.panel, '_sim_view', None), 'session', None)

    def _descriptor(self):
        endpoint = self.service.active_endpoint
        return next((e for e in self.service.available_endpoints
                     if e.get('endpoint_id') == endpoint), {})

    def _context(self, *, geometry=False, scope='all'):
        session = self.session
        snap = getattr(session, 'snapshot', None)
        status = getattr(snap, 'status', None)
        obj = getattr(status, 'mock_object', None)
        motion = (self.service.active_endpoint, self._descriptor().get('instance_id', ''))
        presentation = (getattr(session, 'active_sim_id', ''),
                        getattr(session, 'active_sim_boot_id', ''), getattr(snap, 'session_id', ''))
        value = motion if scope == 'motion' else presentation if scope == 'scene' else motion+presentation
        if geometry:
            value += (getattr(status, 'epoch', None), getattr(obj, 'revision', None),
                      getattr(obj, 'sha256', ''), getattr(obj, 'asset_id', ''))
        return value

    def _scan_key(self):
        scan = self.service.object_scan
        return (*self._context(scope='scene'), scan.get('job_id'),
                scan.get('scene_reference'), scan.get('scene_revision'))

    @property
    def pending(self):
        return '' if self._pending is None else self._pending['action']

    def view(self):
        scan = self.service.object_scan
        status = getattr(getattr(self.session, 'snapshot', None), 'status', None)
        obj = getattr(status, 'mock_object', None)
        remote = self.service.session
        busy = bool(scan.get('active') or remote.service_value('arm_trajectory', {}).get('active')
                    or remote.service_value('wrap_grasp_running', False)
                    or remote.service_value('pick_e2e_running', False)
                    or any(getattr(self.panel.state, k, False) for k in
                           ('pick_running', 'gaze_running', 'ik_running'))
                    or getattr(obj, 'state', '') == 'executing')
        return ServoingView(self.service.active_endpoint, self._descriptor().get('role', ''),
            bool(getattr(remote.status, 'pilot_online', False)), scan, status, obj,
            busy, scan.get('phase') == 'completed' and not scan.get('active'))

    def _sim_reason(self):
        session = self.session
        snap = getattr(session, 'snapshot', None)
        if (session is None or not getattr(session, 'active_sim_id', '')
                or not getattr(session, 'active_sim_boot_id', '')
                or not getattr(snap, 'session_id', '') or getattr(snap, 'status', None) is None):
            return 'Open a live Sim camera session to prepare a virtual scene.'
        if getattr(snap, 'pending_commands', 0):
            return 'Waiting for the selected Sim to acknowledge its command.'
        return ''

    def _motion_reason(self, v):
        host = self.service.current_host_state()
        if not v.pilot_online:
            return 'Waiting for a fresh Pilot snapshot.'
        if not self._descriptor().get('instance_id') or not self.service.has_client():
            return 'Select a live control endpoint with motion authority.'
        if host is None or not getattr(host, 'connected', False):
            return 'Control endpoint is offline.'
        if getattr(host, 'safety_fault', ''):
            return str(host.safety_fault)
        if not getattr(host, 'torque_enabled', False):
            return 'Enable torque in Manual before starting motion.'
        return ''

    def reason(self, action):
        """Return a visible block reason; re-evaluate again when a button fires."""
        v = self.view()
        if action in ('gripper', 'embrace', 'verify'):
            return 'Mesh-based grasp / hold-verification adapter is not connected.'
        if action == 'cancel_scan':
            if self.pending == 'cancel_scan':
                return 'Scan cancellation is already pending.'
            return '' if v.scan.get('active') or self.pending == 'scan' else 'No scan to cancel.'
        if self.pending:
            return 'Waiting for '+self.pending.replace('_', ' ')+' acknowledgement.'
        if action == 'resume_sim':
            reason = self._sim_reason()
            return reason or ('' if v.sim_status.paused else 'Sim is already running.')
        if v.busy:
            return 'Stop the active workflow before changing the scene or starting another action.'
        if action in ('spawn', 'remove', 'detach', 'apply', 'load_scene', 'clear_scene'):
            reason = self._sim_reason()
            if reason:
                return reason
            obj = v.mock_object
            if action == 'spawn':
                if obj is None or self.asset_id not in obj.available_assets:
                    return 'Select an asset from the live Sim catalog.'
                if not all(math.isfinite(x) and abs(x) <= 10 for x in self.position):
                    return 'Position must be finite and within +/-10 m.'
                if not all(math.isfinite(x) for x in self.euler_deg):
                    return 'Rotation must be finite.'
            if action in ('remove', 'detach') and (obj is None or obj.state == 'empty'):
                return 'No virtual object is present.'
            if action == 'detach' and not obj.attached:
                return 'Object is already detached.'
            if action in ('apply', 'load_scene', 'clear_scene') and not self.session.supports_scene_rebuild:
                return 'Selected Sim does not support runtime scene loading.'
            if action == 'load_scene' and not self.scene_reference.strip():
                return 'Enter a staged scene bundle reference.'
            if action == 'apply':
                if not v.reconstructed or not v.scan.get('scene_reference') or not v.scan.get('scene_revision'):
                    return 'Complete a scan with a published scene bundle first.'
                if (v.scan.get('sim_id') != self.session.active_sim_id
                        or v.scan.get('sim_boot_id') != self.session.active_sim_boot_id):
                    return 'Select the same live Sim that produced this reconstruction.'
                if self._applied == self._scan_key() and v.sim_status.epoch <= self._applied_epoch:
                    return 'This reconstruction is already applied to the selected Sim.'
            return ''
        reason = self._motion_reason(v)
        if reason:
            return reason
        if action == 'scan':
            if v.role != 'sim':
                return 'Automatic live scanning currently supports Sim endpoints only.'
            if not self.target_id.strip() or len(self.target_id.strip()) > 64:
                return 'Enter a target instance ID of 1..64 characters.'
            if type(self.max_views) is not int or not 2 <= self.max_views <= 24:
                return 'Use 2..24 maximum views.'
            if not v.scan.get('can_start'):
                return str(v.scan.get('blocked_reason') or 'Waiting for Pilot scan preflight.')
            return ''
        if action in ('compute_hug', 'execute_hug'):
            if v.role != 'sim' or 'simulation.mock_hug.v1' not in self._descriptor().get('capabilities', ()):
                return 'Reference hug requires a capable Sim control endpoint.'
            reason = self._sim_reason()
            if reason:
                return reason
            if (self.session.active_sim_id != v.endpoint
                    or self.session.active_sim_boot_id != self._descriptor().get('instance_id')):
                return 'Camera session and control endpoint must refer to the same Sim boot.'
            if v.sim_status.paused:
                return 'Resume Sim before running reference hug.'
            if v.mock_object is None or v.mock_object.state == 'empty':
                return 'Spawn a virtual object first.'
            if action == 'execute_hug' and (not self.solution.get('solution_id')
                    or self._solution_context != self._context(geometry=True)):
                return 'Compute a fresh solution for this object and Sim session.'
            return ''
        if action == 'wrap':
            return ''
        if action in ('plan_reconstructed_hug', 'execute_reconstructed_hug'):
            if v.role != 'sim' or not v.reconstructed:
                return 'Complete a reconstruction on the selected Sim first.'
            if action == 'execute_reconstructed_hug' and (
                    not self.hug_plan.get('plan_id') or self._hug_context != self._scan_key()):
                return 'Prepare a fresh reconstructed hug first.'
            return ''
        return 'Unknown Visual Servoing action.'

    def refresh(self, *, now=None):
        """Consume cached ACKs and expire presentation state; never submit work."""
        status = getattr(getattr(self.session,'snapshot',None),'status',None)
        if self._applied and (self._applied != self._scan_key()
                or getattr(status,'epoch',-1) > self._applied_epoch):
            self._applied = ()
            self.scene_outcome = ''
        if self.solution and self._solution_context != self._context(geometry=True):
            self.solution = {}
            self.notice = 'Context changed; compute a fresh reference hug solution.'
        pending = self._pending
        if pending is None:
            return
        if pending['context'] != self._context(geometry=pending['geometry'],scope=pending['scope']):
            self._pending = None
            self.error = 'Target or session changed; previous response was discarded.'
            return
        if pending.get('request_id'):
            snap = self.session.snapshot
            result = snap.last_result
            if result is not None and result.request_id == pending['request_id']:
                self._pending = None
                if result.ok:
                    self.error, self.notice = '', str(result.reason)
                    if pending['action'] == 'apply':
                        self._applied = pending['scan_key']
                        # A scene rebuild increments the Sim epoch, even on failure.
                        self._applied_epoch = pending['epoch']+1
                        self.scene_outcome = 'applied'
                    elif pending['action'] in ('clear_scene','load_scene','spawn','remove'):
                        self._applied = ()
                        self.scene_outcome = 'cleared' if pending['action'] == 'clear_scene' else ''
                else:
                    self.error = str(result.reason)
                    self.scene_outcome = 'failed'
                return
        if (time.monotonic() if now is None else now) >= pending['deadline']:
            self._pending = None
            self.error = pending['action'].replace('_', ' ')+' acknowledgement timed out; check status before retrying.'

    def _callback(self, token, value, *, error=False):
        pending = self._pending
        if pending is None or pending['token'] != token:
            return
        if pending['context'] != self._context(geometry=pending['geometry'],scope=pending['scope']):
            self._pending = None
            return
        self._pending = None
        if error:
            self.error = str(value)
            return
        if value is False:
            self.error = 'Pilot declined the request. Check live workflow status before retrying.'
            self.notice = ''
            return
        self.error = ''
        self.notice = 'Request accepted; follow live status for completion.'
        if pending['action'] == 'plan_reconstructed_hug':
            self.hug_plan = dict(value) if isinstance(value, dict) else {}
            self._hug_context = pending['scan_key']
            self.notice = 'Interpolated path ready. Execute separately from Home; grasp success is unverified.'
        if pending['action'] == 'compute_hug':
            self.solution = dict(value) if isinstance(value, dict) else {}
            self._solution_context = pending['context']
            if not self.solution.get('solution_id'):
                self.error = 'Pilot returned no reference hug solution.'
            else:
                self.notice = 'Reference solution ready. Execution is a separate action.'

    def perform(self, action):
        self.refresh()
        reason = self.reason(action)
        if reason:
            self.error = reason
            return False
        self.error, self.notice = '', ''
        if action in ('scan','wrap'):
            self.solution = {}
        self._serial += 1
        token = self._serial
        geometry = action in ('compute_hug', 'execute_hug')
        scope = 'all' if geometry or action in ('plan_reconstructed_hug','execute_reconstructed_hug') else 'motion' if action in ('scan','cancel_scan','wrap') else 'scene'
        self._pending = dict(action=action,token=token,geometry=geometry,
            scope=scope,context=self._context(geometry=geometry,scope=scope),
            deadline=time.monotonic()+10.,scan_key=self._scan_key(),
            epoch=getattr(getattr(getattr(self.session,'snapshot',None),'status',None),'epoch',0))
        callbacks = dict(on_result=lambda value: self._callback(token,value),
                         on_error=lambda error: self._callback(token,error,error=True))
        try:
            if action in ('plan_reconstructed_hug', 'execute_reconstructed_hug'):
                method = 'plan_reconstructed_hug' if action == 'plan_reconstructed_hug' else 'execute_arm_trajectory'
                args = (self.target_id.strip(),) if action == 'plan_reconstructed_hug' else (self.hug_plan['plan_id'],)
                self.service.reconstructed_hug_async(method,*args,**callbacks)
                if action == 'execute_reconstructed_hug':self.hug_plan = {}
            elif action in ('scan', 'cancel_scan'):
                name, args = ('start_object_scan', (self.target_id.strip(),self.max_views)) if action == 'scan' else ('cancel_object_scan', ())
                self.service.object_scan_async(name,*args,**callbacks)
                self.step = 1
            elif action in ('compute_hug','execute_hug','wrap'):
                method = dict(compute_hug='compute_mock_hug',execute_hug='execute_mock_hug',wrap='start_wrap_grasp')[action]
                args = (self.solution['solution_id'],) if action == 'execute_hug' else ()
                getattr(self.service,method)(*args,**callbacks,request_timeout_s=5.)
                if action == 'execute_hug':
                    self.solution = {}  # Consume once; never execute the same UI draft twice.
            else:
                command, args = {
                    'spawn': ('spawn_mock_object',dict(asset_id=self.asset_id,
                        position=list(self.position),euler_deg=list(self.euler_deg))),
                    'remove': ('remove_mock_object',None), 'detach':('detach_mock_object',None),
                    'load_scene':('load_scene',{'manifest':self.scene_reference.strip()}),
                    'clear_scene':('clear_scene',None), 'apply':('load_scene',None),
                    'resume_sim':('resume',None),
                }[action]
                request = self.session.load_scan_result(self.service.object_scan) if action == 'apply' else self.session.send_command(command,args)
                if not request:
                    raise ValueError(self.session.snapshot.last_error or 'Sim did not accept this command.')
                self._pending['request_id'] = request
                self._pending['deadline'] = time.monotonic()+60.
                self.solution = {}
                self.notice = 'Waiting for Sim acknowledgement...'
                if action == 'apply':
                    self.scene_outcome = 'pending'
        except Exception as exc:
            self._pending = None
            self.error = str(exc)
            return False
        return True

    def stop(self):
        """Best-effort explicit stops remain reachable despite stale preflight."""
        v = self.view()
        operations = []
        paused_hug = False
        stopped_ik = False
        if v.scan.get('active') or self.pending == 'scan':
            operations.append(lambda: self.service.object_scan_async('cancel_object_scan',
                on_result=lambda _: None,on_error=lambda error: setattr(self,'error',str(error))))
        if self.service.session.service_value('arm_trajectory', {}).get('active'):
            operations.append(self.service.cancel_arm_trajectory)
        if (self.pending == 'wrap' or self.service.session.service_value('wrap_grasp_running',False)
                or self.service.session.service_value('pick_e2e_running',False)
                or getattr(self.panel.state,'pick_running',False)):
            operations.append(self.service.stop_pick_e2e)
        if getattr(self.panel.state,'gaze_running',False):
            operations.append(self.service.stop_gaze_stabilizer)
        if getattr(self.panel.state,'ik_running',False):
            # The existing IK solve has no cancellation RPC. Torque-off keeps a
            # late solver result from moving the endpoint while it finishes.
            operations.append(self.service.torque_off)
            stopped_ik = True
        if (self.pending == 'execute_hug' or getattr(v.mock_object,'state','') == 'executing') and self.session is not None:
            operations.append(lambda: self.session.send_command('pause'))
            paused_hug = True
        self.solution = {}
        for operation in operations:
            try:
                operation()
            except Exception as exc:
                self.error = str(exc)
        self.notice = ('Stop requested; follow live status.' if operations else 'No running task to stop.')
        if paused_hug:
            self.notice += ' Reference hug stops by pausing Sim. Resume explicitly when ready.'
        if stopped_ik:
            self.notice += ' IK solver has no cancellation adapter; torque-off requested while it finishes.'
