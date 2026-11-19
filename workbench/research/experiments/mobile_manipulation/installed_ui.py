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


def cancel_mobile_after_scan(report, cancel_and_wait):
    """A passing observation must also release its mobile task cleanly."""
    previous_stage=report.get('stage')
    try:
        cancel_and_wait()
    except Exception as exc:
        report.update(accepted=False,stage='failed',cancel_error=repr(exc))
        if not report.get('error'):
            report['error']='Mobile task cancellation could not be confirmed'
    else:
        report['stage']=previous_stage


def scan_result(scan, previous_job, *, require_see=False, require_adapointr=False):
    if (not scan.get('job_id') or scan.get('job_id')==previous_job or
            scan.get('phase')!='completed' or scan.get('accepted_views',0)<2):
        return False,'Fresh multi-view reconstruction did not complete'
    if require_see and 'see' not in (scan.get('selected_view_policies') or []):
        return False,'No executed next view was selected by SEE'
    execution=scan.get('model_execution') or {}
    completion=(execution.get('models') or {}).get('completion') or {}
    if require_adapointr and (not completion.get('loaded') or not completion.get('applied') or
            'adapointr' not in str(completion.get('backend','')).lower() or
            scan.get('predicted_points',0)<=0):
        return False,'AdaPoinTr completion was not verified'
    return True,''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=Path('/opt/elesim/config/installed.yaml'))
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--position',type=float,nargs=3,default=(.7,.15,.55))
    p.add_argument('--timeout',type=float,default=1200.)
    p.add_argument('--home-timeout',type=float,default=120.)
    p.add_argument('--preflight-only',action='store_true')
    p.add_argument('--scan-only',action='store_true',help='stop after fresh multi-view reconstruction; does not verify grasp or hold')
    p.add_argument('--require-see',action='store_true',help='require a SEE-selected next view in a scan-only test')
    p.add_argument('--require-adapointr',action='store_true',help='require applied AdaPoinTr in a scan-only test')
    p.add_argument('--evaluation-mode',choices=('installed','development'),default='installed')
    p.add_argument('--reset-sim',action='store_true')
    p.add_argument('--video',action='store_true')
    p.add_argument('--gui',action='store_true',help='render the published panel and click real ImGui buttons')
    p.add_argument('--gui-backend',choices=('glfw','egl'),default='glfw')
    a=p.parse_args()
    if (a.require_see or a.require_adapointr) and not a.scan_only:
        p.error('--require-see and --require-adapointr require --scan-only')
    a.output.mkdir(parents=True,exist_ok=False)
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
    gui=None
    started_monotonic=time.monotonic()
    report=dict(accepted=False,complete=False,started=time.time(),stage='discovery',events=[],
        scope='published installed UI backends; actual cross-host DDS/WebRTC and production Pilot/Sim; no native GUI input',
        fixture_sim_placement_only=list(a.position),sibling_imports_absent=True,
        endpoint=cfg.endpoint_id,system_id=cfg.dds.system_id,security_profile=cfg.dds.security_profile,
        evaluation_mode=a.evaluation_mode,check_kind='home_preflight' if a.preflight_only else 'mobile_scan' if a.scan_only else 'full_mobile',
        reset_sim=a.reset_sim)
    if a.gui:
        report['scope']='published installed UI rendering and ImGui mouse clicks; actual cross-host DDS/WebRTC and production Pilot/Sim'
        report['gui']=dict(backend=a.gui_backend,renderer=None,clicks=[])
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
        if gui is not None:gui.draw()
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
                report['events'].append(dict(event=label,wall=time.time(),elapsed_s=time.monotonic()-started_monotonic));save();return
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
        # Match the real button's availability instead of clicking while a
        # fresh snapshot / scene ACK is still in flight during startup.
        wait(name+' available',lambda:not controller.reason(name),30.)
        if gui is None:
            if not controller.perform(name):raise RuntimeError(name+': '+controller.error)
        else:
            previous_serial=controller._serial
            gui.click(name,tick)
            # A fresh snapshot can expire between the availability check and
            # mouse release. Retry once only when the real controller reports
            # that admission changed; never repeat a submitted command.
            if controller._serial<=previous_serial and controller.reason(name):
                wait(name+' available after click race',lambda:not controller.reason(name),30.)
                gui.click(name,tick)
            if controller._serial<=previous_serial:raise RuntimeError(name+': GUI click did not submit an action')
        wait(name+' ACK',lambda:not controller.pending,65.)
        if controller.error:raise RuntimeError(name+': '+controller.error)
    try:
        if a.gui:
            if __package__:
                from .installed_gui import InstalledGui
            else:
                from installed_gui import InstalledGui
            gui=InstalledGui(service,sim,a.output,backend=a.gui_backend)
            controller=gui.controller
            report['gui'].update(renderer=gui.renderer_name,clicks=gui.clicks)
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
        previous_scan_job=service.object_scan.get('job_id')
        perform('start_mobile')
        if a.scan_only:
            def scan_terminal():
                scan=service.object_scan
                fresh=scan.get('job_id') and scan.get('job_id')!=previous_scan_job
                return (fresh and scan.get('phase')=='completed') or operator.service_value(
                    'mobile_manipulation',{}).get('phase') in {'failed','unverified','cancelled'}
            wait('mobile scan terminal',scan_terminal,a.timeout)
            scan=service.object_scan
            accepted,error=scan_result(scan,previous_scan_job,require_see=a.require_see,
                                       require_adapointr=a.require_adapointr)
            report.update(complete=True,accepted=accepted,scan=scan,stage='scan verified' if accepted else 'failed')
            if not accepted:report['error']=error
        else:
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
            def cancel_and_wait():
                call('cancel_mobile_manipulation')
                wait('mobile cancellation confirmed',lambda:operator.service_value(
                    'mobile_manipulation',{}).get('phase') in
                    {'idle','cancelled','failed','unverified','completed'},15.)
            cancel_mobile_after_scan(report,cancel_and_wait)
        for name,(container,stream) in encoders.items():
            try:
                for packet in stream.encode():container.mux(packet)
                container.close()
            except Exception as exc:report['video_error']=repr(exc)
        report.update(finished=time.time(),elapsed_s=time.monotonic()-started_monotonic,video_frames=video_frames)
        if gui is not None:
            try:
                gui.draw(force=True);gui.screenshot('final')
                report['gui'].update(frames=gui.frames)
            except Exception as exc:report['gui_error']=repr(exc)
            finally:
                try:gui.close()
                except Exception as exc:report['gui_close_error']=repr(exc)
        save();trace.close();sim.close();operator.close();hub.close()
    return 0 if report['accepted'] else 1


if __name__=='__main__':raise SystemExit(main())
