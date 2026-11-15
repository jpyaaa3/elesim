"""Evaluate installed UI -> Pilot -> Sim through public operator intents.

Only UI and protocol are imported. Fixture coordinates go to the Sim placement
intent; Pilot receives RGB-D and measured feedback, never this driver's fixture.
Use the registered UI's config/security/network and published image, with the
ordinary Pilot and Sim services running on the other installed host.
"""
import argparse
from dataclasses import asdict,is_dataclass
import importlib.util
import json
from pathlib import Path
import time
from types import SimpleNamespace


def plain(value):
    if is_dataclass(value):return plain(asdict(value))
    if isinstance(value,SimpleNamespace):return plain(vars(value))
    if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [plain(v) for v in value]
    if isinstance(value,float) and not __import__('math').isfinite(value):return None
    return value


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=Path('/opt/elesim/config/installed.yaml'))
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--position',type=float,nargs=3,default=(.7,.15,.55))
    p.add_argument('--timeout',type=float,default=1200.)
    p.add_argument('--home-timeout',type=float,default=120.)
    p.add_argument('--preflight-only',action='store_true')
    p.add_argument('--evaluation-mode',choices=('installed','development'),default='installed')
    p.add_argument('--reset-sim',action='store_true')
    p.add_argument('--video',action='store_true')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    for sibling in ('elesim_pilot','elesim_sim'):
        if importlib.util.find_spec(sibling) is not None:raise RuntimeError('UI image sees sibling '+sibling)
    from elesim_ui.config import load_config
    from elesim_ui.peer_runtime import UiPeerHub
    from elesim_ui.operator_session import OperatorSession
    from elesim_ui.operator import RemoteControlService,RemotePanelState
    from elesim_ui.sim_session import UiSimSession,SIMULATION_STREAMS
    from elesim_ui.visual_servoing import ServoingController
    from elesim_protocol import ControlU,control_u_to_sim_q
    import numpy as np
    cfg=load_config(a.config)
    hub=UiPeerHub(endpoint_id=cfg.endpoint_id,settings=cfg.dds)
    operator=OperatorSession(ui_id=cfg.endpoint_id,pilot_id=cfg.pilot_id,peer=hub.channel('operator'))
    service=RemoteControlService(operator,RemotePanelState(operator))
    sim=UiSimSession(ui_id=cfg.endpoint_id,sim_id=cfg.sim_id,peer=hub.channel('sim'))
    controller=ServoingController(SimpleNamespace(service=service,state=service.state,
        _sim_view=SimpleNamespace(session=sim)))
    report=dict(accepted=False,complete=False,started=time.time(),stage='discovery',events=[],
        scope='published installed UI backends; actual cross-host DDS/WebRTC and production Pilot/Sim; no native GUI input',
        fixture_sim_placement_only=list(a.position),sibling_imports_absent=True,
        endpoint=cfg.endpoint_id,system_id=cfg.dds.system_id,security_profile=cfg.dds.security_profile,
        evaluation_mode=a.evaluation_mode,check_kind='home_preflight' if a.preflight_only else 'full_mobile',
        reset_sim=a.reset_sim)
    if a.evaluation_mode=='development':
        report['scope']='development diagnosis with patched runtime; not installed-release acceptance'
    trace=(a.output/'telemetry.jsonl').open('w');last_log=last_save=0.;last_phase='';encoders={};video_frames={}
    video_start=None;last_video=0.
    def save():
        tmp=a.output/'report.tmp';tmp.write_text(json.dumps(plain(report),indent=2,allow_nan=False)+'\n')
        tmp.replace(a.output/'report.json')
    def tick():
        nonlocal last_log,last_save,last_phase,last_video
        service.poll();controller.refresh();now=time.monotonic()
        mobile=operator.service_value('mobile_manipulation',{})
        host=service.current_host_state()
        if now-last_save>.5:
            row=dict(wall=time.time(),mobile=mobile,scan=service.object_scan,host=plain(host),
                streams=list(sim.connected_streams),operator=plain(operator.status),sim_error=sim.last_error)
            trace.write(json.dumps(plain(row),allow_nan=False)+'\n');trace.flush()
            report.update(mobile=mobile,scan=service.object_scan,host=plain(host),
                streams=list(sim.connected_streams),ui_error=controller.error,sim_error=sim.last_error,
                control_u=plain(service.current_control_u()),offsets=service.state.offset_values(),
                command_q=[operator.state_value(k,None) for k in ('linear','roll','theta1','theta2')])
            save();last_save=now
        phase=mobile.get('phase','idle')
        if phase!=last_phase or now-last_log>15.:
            print('MOBILE_INSTALLED',report['stage'],'phase',phase,'reason',mobile.get('reason',''),
                'scan',service.object_scan.get('phase'),'frames',sim.connected_streams,flush=True)
            last_phase=phase;last_log=now
        if encoders and now-last_video>=.1:
            import av
            from fractions import Fraction
            for name,(container,stream) in encoders.items():
                frame=sim.frame(name)
                if frame is None:continue
                video=av.VideoFrame.from_ndarray(frame.copy(),format='bgr24')
                video.pts=round((now-video_start)*1000);video.time_base=Fraction(1,1000)
                for packet in stream.encode(video):container.mux(packet)
                video_frames[name]+=1
            last_video=now
    def wait(label,condition,timeout):
        report['stage']=label;save();deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            tick()
            if condition():
                report['events'].append(dict(event=label,wall=time.time()));save();return
            time.sleep(.02)
        raise TimeoutError(label)
    def call(name,*args,**kwargs):
        reply={}
        operator.submit('service_call',name,*args,**kwargs,request_timeout_s=10.,
            on_result=lambda v:reply.update(ok=True,value=v),on_error=lambda v:reply.update(ok=False,error=v))
        wait(name,lambda:bool(reply),12.)
        if not reply['ok']:raise RuntimeError(name+': '+str(reply['error']))
        return reply.get('value')
    def perform(name):
        if not controller.perform(name):raise RuntimeError(name+': '+controller.error)
        wait(name+' ACK',lambda:not controller.pending,65.)
        if controller.error:raise RuntimeError(name+': '+controller.error)
    try:
        wait('installed discovery and media',lambda:service.has_client() and sim.status is not None
            and all(sim.frame(s) is not None for s in SIMULATION_STREAMS),900.)
        report['peers']=service.available_endpoints
        if service.active_endpoint!=cfg.sim_id:call('select_endpoint',cfg.sim_id)
        wait('selected Sim authority',lambda:service.active_endpoint==cfg.sim_id and service.has_client(),30.)
        descriptor=next((d for d in service.available_endpoints if d.get('endpoint_id')==cfg.sim_id),{})
        if descriptor.get('role')!='sim' or sim.active_sim_id!=cfg.sim_id:
            raise RuntimeError('installed evaluation refuses non-Sim motion targets')
        if a.reset_sim:
            previous_epoch=sim.status.epoch;reset_id=sim.send_command('reset')
            def reset_done():
                result=sim.snapshot.last_result
                if result is None or result.request_id!=reset_id:return False
                if not result.ok:raise RuntimeError('Sim reset: '+result.reason)
                return sim.status.epoch>previous_epoch
            wait('independent Sim episode reset',reset_done,30.)
        if sim.status.paused:
            sim.send_command('resume');wait('Sim resumed',lambda:not sim.status.paused,15.)
        # Operator ACK means Pilot accepted the intent. Best-effort motion must
        # be confirmed in target feedback before sending a later command.
        for attempt in range(3):
            call('torque_on')
            try:
                wait('measured torque ON',lambda:bool(getattr(service.current_host_state(),'torque_enabled',False)),5.)
                break
            except TimeoutError:
                if attempt==2:raise
        call('send_claw_command',closed=False)
        wait('measured claws open',lambda:getattr(service.current_host_state(),'claw_closed',None) is False,10.)
        call('home_controls')
        def at_home():
            host=service.current_host_state();q=getattr(host,'q',None)
            # Home is a display preset and includes the installed operator's
            # calibrated offsets. Compare measurement to that physical target.
            offsets=service.state.offset_values()[:4]
            target=control_u_to_sim_q(ControlU(*(v+o for v,o in
                zip((180.,180.,10.,10.),offsets))),service.control_mapping())
            report['home_target_q']=plain(target)
            return (host is not None and host.torque_enabled and host.claw_closed is False and q is not None
                and np.all(np.abs(np.array([q.linear_m,q.roll_rad,q.theta1_rad,q.theta2_rad])-
                    [target.linear_m,target.roll_rad,target.theta1_rad,target.theta2_rad])<[.001,.015,.015,.015]))
        if a.video:
            import av
            from fractions import Fraction
            video_start=time.monotonic()
            for name in SIMULATION_STREAMS:
                frame=sim.frame(name);container=av.open(str(a.output/(name+'.mp4')),mode='w')
                stream=container.add_stream('libx264',rate=10)
                stream.height,stream.width=frame.shape[:2];stream.pix_fmt='yuv420p'
                stream.codec_context.thread_count=1;stream.codec_context.time_base=Fraction(1,1000)
                stream.options=dict(preset='veryfast',crf='25',tune='zerolatency')
                encoders[name]=container,stream;video_frames[name]=0
        wait('measured Home',at_home,a.home_timeout)
        if a.preflight_only:
            report.update(complete=True,accepted=True,stage='Home verified')
            return
        report['available_assets']=list(sim.status.mock_object.available_assets)
        controller.asset_id='cylinder_100mm.obj';controller.position=list(a.position)
        previous_revision=sim.status.mock_object.revision
        perform('spawn')
        wait('physical cylinder placed',lambda:sim.status.mock_object.asset_id=='cylinder_100mm'
            and sim.status.mock_object.revision>previous_revision,30.)
        perform('plan_mobile');report['plan']=controller.mobile_plan.copy();save()
        perform('start_mobile')
        wait('mobile task terminal',lambda:operator.service_value('mobile_manipulation',{}).get('phase')
            in {'completed','failed','unverified','cancelled'},a.timeout)
        mobile=operator.service_value('mobile_manipulation',{})
        report.update(complete=True,accepted=mobile.get('phase')=='completed',mobile=mobile,
            scan=service.object_scan,stage='completed' if mobile.get('phase')=='completed' else 'failed')
        if not report['accepted']:report['error']=mobile.get('reason','task did not verify the loaded hold')
    except KeyboardInterrupt:
        report.update(error='evaluation interrupted',stage='interrupted')
    except Exception as exc:
        report.update(error=repr(exc),stage='failed')
        print('MOBILE_INSTALLED_ERROR',repr(exc),flush=True)
    finally:
        if operator.service_value('mobile_manipulation',{}).get('active'):
            try:call('cancel_mobile_manipulation')
            except Exception as exc:report['cancel_error']=repr(exc)
        for name,(container,stream) in encoders.items():
            try:
                for packet in stream.encode():container.mux(packet)
                container.close()
            except Exception as exc:report['video_error']=repr(exc)
        report.update(finished=time.time(),video_frames=video_frames)
        save();trace.close();sim.close();operator.close();hub.close()
    return 0 if report['accepted'] else 1


if __name__=='__main__':raise SystemExit(main())
