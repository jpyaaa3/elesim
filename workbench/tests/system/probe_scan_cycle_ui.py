"""UI-only headless driver using the same operator and Apply button backends.

Runs actual DDS requests and both WebRTC decoders in the isolated installed UI
image. It does not import Pilot/Sim or mount ground truth or reconstruction files.
"""
from __future__ import annotations

import argparse
import binascii
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import struct
import time
import traceback
import zlib


def check_poisson_surface(status):
    diagnostics = status.get('model_execution', {}).get('models', {}).get('mesher', {})
    assert diagnostics.get('surface_method') == 'screened_poisson', diagnostics
    assert diagnostics.get('remesh_resolution_m') == .004, diagnostics
    return diagnostics


class ServoingWidgets:
    """Real ImGui widgets and live UI services, without an OS window/GL renderer."""

    def __init__(self, service, sim):
        from types import SimpleNamespace
        import imgui
        from elesim_ui.visual_servoing import ServoingController
        self.imgui = imgui
        self.context = imgui.create_context()
        self.io = imgui.get_io()
        self.io.ini_file_name = None
        self.io.delta_time = 1/30
        self.io.display_size = 1280, 1200
        self.io.fonts.get_tex_data_as_rgba32()
        self.panel = SimpleNamespace(service=service, state=service.state,
            _sim_view=SimpleNamespace(session=sim))
        self.panel.servoing = ServoingController(self.panel)
        self.controller = self.panel.servoing
        self.rectangles, self.clicks = {}, []
        self.frames = 0

    def draw(self):
        from elesim_ui.panels.visual_servoing import draw_visual_servoing_panel
        imgui = self.imgui
        button = imgui.button
        self.rectangles.clear()
        def record(label, *args, **kwargs):
            result = button(label, *args, **kwargs)
            a, b = imgui.get_item_rect_min(), imgui.get_item_rect_max()
            self.rectangles[label] = ((a.x+b.x)/2, (a.y+b.y)/2)
            return result
        imgui.button = record
        try:
            imgui.new_frame()
            imgui.set_next_window_position(0, 0)
            imgui.set_next_window_size(1280, 1200)
            imgui.begin('Visual Servoing live probe')
            try:
                draw_visual_servoing_panel(self.panel)
            finally:
                imgui.end()
            imgui.render()
            self.frames += 1
        finally:
            imgui.button = button

    def click(self, label):
        self.draw()
        self.draw()
        point = self.rectangles[label]
        assert 0 < point[0] < 1280 and 0 < point[1] < 1200, (label, point)
        self.io.mouse_pos = point
        self.io.mouse_down[0] = True
        self.draw()
        self.io.mouse_down[0] = False
        self.draw()
        self.clicks.append(dict(label=label, time=time.time()))

    def close(self):
        self.imgui.destroy_context(self.context)


def write_png(path, bgr):
    """Write the decoded image with stdlib only; UI has no OpenCV dependency."""
    def chunk(kind, data):
        return struct.pack('!I',len(data))+kind+data+struct.pack('!I',binascii.crc32(kind+data)&0xffffffff)
    height, width, channels = bgr.shape
    assert channels == 3 and str(bgr.dtype) == 'uint8'
    pixels = b''.join(b'\0'+row[:,::-1].tobytes() for row in bgr)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))+
        chunk(b'IDAT',zlib.compress(pixels,3))+chunk(b'IEND',b''))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--system', required=True)
    parser.add_argument('--pilot', required=True)
    parser.add_argument('--sim', required=True)
    parser.add_argument('--ui', required=True)
    parser.add_argument('--domain', type=int, default=212)
    parser.add_argument('--view-budget', type=int, default=3)
    parser.add_argument('--peer', action='append', required=True)
    parser.add_argument('--record-video',action='store_true')
    parser.add_argument('--visual-servoing',action='store_true',
        help='Drive actual Visual Servoing ImGui buttons and controller instead of direct scan/Apply calls')
    parser.add_argument('--require-poisson',action='store_true')
    args = parser.parse_args()
    for sibling in ('elesim_pilot','elesim_sim'):
        assert importlib.util.find_spec(sibling) is None, sibling
    from elesim_protocol import DdsRuntimeSettings
    from elesim_ui.operator import RemoteControlService, RemotePanelState
    from elesim_ui.operator_session import OperatorSession
    from elesim_ui.sim_session import UiSimSession, SIMULATION_STREAMS
    import numpy as np

    output = args.evidence
    output.mkdir(parents=True, exist_ok=True)
    evidence = dict(complete=False, stage='connecting', started=time.time(),
        scope='installed UI backend; real DDS and decoded WebRTC; no native window automation',
        sibling_imports_absent=True, timeline=[])
    settings = DdsRuntimeSettings(system_id=args.system, domain_id=args.domain,
        discovery_mode='static', static_peers=tuple(args.peer),
        network_interface='eth0', security_profile='trusted-network')
    operator = OperatorSession(ui_id=args.ui+'-operator', pilot_id=args.pilot, settings=settings)
    service = RemoteControlService(operator, RemotePanelState(operator))
    sim = UiSimSession(ui_id=args.ui, sim_id=args.sim, settings=settings)
    recorder = None
    widgets = ServoingWidgets(service, sim) if args.visual_servoing else None
    controller = widgets.controller if widgets is not None else None
    if widgets is not None:
        evidence['scope'] = ('real Visual Servoing ImGui button mouse input and controller; '
            'actual DDS, Genesis and decoded WebRTC; no OS window/GL UI renderer')

    def save():
        (output/'ui.json').write_text(json.dumps(evidence, indent=2, allow_nan=False)+'\n')

    def wait(label, condition, timeout=120.):
        evidence['stage'] = label
        save()
        deadline, last_progress = time.monotonic()+timeout, 0.
        while time.monotonic() < deadline:
            service.poll()
            if widgets is not None:
                widgets.draw()
            if condition():
                evidence['timeline'].append(dict(event=label, time=time.time()))
                save()
                return
            if time.monotonic()-last_progress > 10:
                status = service.object_scan
                print('SCAN_CYCLE', label, 'scan=', status, 'operator=', operator.status,
                    'sim_error=', sim.last_error, 'streams=', sim.connected_streams, flush=True)
                last_progress = time.monotonic()
            time.sleep(.02)
        raise TimeoutError(f'{label}: operator={operator.status}; sim={sim.last_error}; scan={service.object_scan}')

    def call(name, *values, **kwargs):
        reply = {}
        operator.submit('service_call', name, *values, **kwargs, request_timeout_s=5.,
            on_result=lambda value:reply.update(ok=True,value=value),
            on_error=lambda error:reply.update(ok=False,error=error))
        wait(name, lambda:bool(reply), 8.)
        if not reply['ok']:
            raise RuntimeError(f'{name}: {reply["error"]}')
        return reply.get('value')

    def frames(label):
        rows = {}
        for stream in SIMULATION_STREAMS:
            frame = sim.frame(stream)
            assert frame is not None, stream
            frame = frame.copy()
            filename = f'{label}-{stream}.png'
            write_png(output/filename, frame)
            rows[stream] = dict(file=filename, version=sim.frame_version(stream),
                width=int(frame.shape[1]), height=int(frame.shape[0]))
        return rows

    def red_pixels(stream):
        frame = sim.frame(stream)
        if frame is None:
            return 1_000_000
        b,g,r = (frame[:,:,index].astype(np.int16) for index in range(3))
        return int(((r>80)&(r>g*1.5)&(r>b*1.5)).sum())

    save()
    try:
        wait('discovery and initial build', lambda:service.has_client() and sim.supports_scene_rebuild
            and sim.status is not None and all(sim.frame(name) is not None for name in SIMULATION_STREAMS), 900.)
        if args.record_video:
            from scan_cycle_recording import FrameRecorder
            recorder = FrameRecorder(sim,service,output,lambda:evidence['stage'])
            recorder.start()
            initial_view = time.monotonic()
            wait('demo initial view',lambda:time.monotonic()-initial_view >= 2.,5.)
        evidence['before'] = frames('before-scan')
        if widgets is not None:
            wait('Visual Servoing scene preflight', lambda:not controller.reason('spawn'), 30.)
            controller.position = [3., 3., 1.]
            widgets.click('Place object##vs-spawn')
            wait('Visual Servoing place ACK', lambda:not controller.pending, 30.)
            assert not controller.error, controller.error
            wait('Visual Servoing placed status', lambda:sim.status.mock_object.state != 'empty', 15.)
            widgets.click('Remove object##vs-remove')
            wait('Visual Servoing remove ACK', lambda:not controller.pending, 30.)
            assert not controller.error, controller.error
            wait('Visual Servoing empty status', lambda:sim.status.mock_object.state == 'empty', 15.)
        initial_step = service.current_host_state().sim_step_count
        wait('steady simulation feedback', lambda:service.current_host_state().sim_step_count >= initial_step+100
            and service.current_host_state().rx_age_s < .2, 60.)
        # A newly created volatile DDS motion writer may not yet match its
        # reader. Retry this explicit Sim-only intent against the exact boot;
        # record each attempt and require actual feedback before continuing.
        boot = sim.active_sim_boot_id
        for attempt in range(3):
            assert sim.active_sim_id == args.sim and sim.active_sim_boot_id == boot
            assert service.active_endpoint == args.sim
            call('torque_on')
            try:
                wait('Sim torque feedback',lambda:service.current_host_state().torque_enabled,1.5)
                break
            except TimeoutError:
                if attempt == 2:
                    raise
        call('send_claw_command', closed=False)
        wait('scan prerequisites', lambda:service.object_scan.get('can_start'), 60.)
        host = service.current_host_state()
        evidence['host_before_scan'] = {name:getattr(host,name) for name in (
            'connected','torque_enabled','claw_closed','rx_age_s','sim_step_count',
            'sim_realtime_factor','reply_ok','reply_reason')}
        if widgets is None:
            call('start_object_scan', 'target', args.view_budget)
        else:
            controller.target_id, controller.max_views = 'target', args.view_budget
            widgets.click('2  Scan##vs-step-1')
            wait('Visual Servoing scan preflight', lambda:not controller.reason('scan'), 30.)
            widgets.click('Start scan & reconstruction##vs-scan')
            assert controller.pending == 'scan', (controller.pending, controller.error)
            wait('Visual Servoing scan ACK', lambda:not controller.pending, 15.)
            assert not controller.error, controller.error
        wait('multi-view scan', lambda:service.object_scan.get('phase') in ('completed','failed','cancelled'), 620.)
        status = service.object_scan
        evidence['scan'] = status
        assert status['phase'] == 'completed', status
        if args.require_poisson:
            evidence['poisson_surface'] = check_poisson_surface(status)
        assert status['accepted_views'] >= 2 and status['observed_points'] > 0, status
        assert status['geometry_complete_claimed'] is False, status
        assert status['sim_id'] == sim.active_sim_id and status['sim_boot_id'] == sim.active_sim_boot_id
        # Hide only the original fixture through the normal existing operator
        # command, so the after image can demonstrate the imported mesh alone.
        stable_since = [None]
        def control_stable():
            host = service.current_host_state()
            ready = (host.connected and host.reply_ok and host.rx_age_s < .2
                and service.active_endpoint == args.sim and service.object_scan.get('can_start'))
            if not ready:
                stable_since[0] = None
                return False
            if stable_since[0] is None:
                stable_since[0] = time.monotonic()
            return time.monotonic()-stable_since[0] >= 1.
        wait('post-scan control authority',control_stable,30.)
        for attempt in range(3):
            assert sim.active_sim_boot_id == status['sim_boot_id'] and sim.active_sim_id == args.sim
            call('send_sim_target_xyz', -8., -8., -8.)
            try:
                wait('original fixture hidden',lambda:all(red_pixels(name)<10 for name in SIMULATION_STREAMS),3.)
                break
            except TimeoutError:
                if attempt == 2:
                    raise
        evidence['hidden_red_pixels'] = {name:red_pixels(name) for name in SIMULATION_STREAMS}
        evidence['hidden'] = frames('original-hidden')
        epoch = sim.status.epoch
        if widgets is None:
            request_id = sim.load_scan_result(status)
        else:
            widgets.click('3  Mesh##vs-step-2')
            wait('Visual Servoing apply preflight', lambda:not controller.reason('apply'), 30.)
            widgets.click('Apply reconstruction to Sim##vs-apply')
            assert controller.pending == 'apply', (controller.pending, controller.error)
            request_id = controller._pending['request_id']
        wait('apply reconstructed scene', lambda:sim.snapshot.last_result is not None
            and sim.snapshot.last_result.request_id == request_id, 120.)
        result = sim.snapshot.last_result
        evidence['application'] = asdict(result)
        assert result.ok, result
        if widgets is not None:
            assert not controller.pending and controller.scene_outcome == 'applied', controller.error
            assert controller.reason('apply'), 'same reconstruction must not be applied twice'
            evidence['visual_servoing_apply'] = dict(scene_outcome=controller.scene_outcome,
                duplicate_block_reason=controller.reason('apply'), request_id=request_id)
        ack_versions = {name:sim.frame_version(name) for name in SIMULATION_STREAMS}
        wait('fresh WebRTC after rebuild', lambda:sim.status.epoch > epoch and all(
            sim.frame(name) is not None and (sim.frame_version(name) or 0) >
            (ack_versions[name] or 0)+4 and np.ptp(sim.frame(name)) > 30
            for name in SIMULATION_STREAMS), 30.)
        evidence['after'] = frames('reconstructed')
        if widgets is not None:
            pause = sim.send_command('pause')
            wait('explicit Sim pause', lambda:sim.snapshot.last_result is not None
                and sim.snapshot.last_result.request_id == pause and sim.status.paused, 20.)
            assert sim.snapshot.last_result.ok
            widgets.click('Resume Sim##vs-resume_sim')
            wait('Visual Servoing resume ACK', lambda:not controller.pending and not sim.status.paused, 20.)
            assert not controller.error, controller.error
            widgets.click('4  Action##vs-step-3')
            assert all(controller.reason(action) for action in ('gripper','embrace','verify'))
        if recorder is not None:
            final_view = time.monotonic()
            wait('demo final view',lambda:time.monotonic()-final_view >= 3.,5.)
        evidence.update(complete=True, stage='complete', finished=time.time(),
            sim_boot_id=sim.active_sim_boot_id, scene_revision=status['scene_revision'])
        print('SCAN_CYCLE_COMPLETE', json.dumps(evidence), flush=True)
    except BaseException as exc:
        evidence.update(error=str(exc), traceback=traceback.format_exc(), finished=time.time())
        raise
    finally:
        if widgets is not None:
            evidence['visual_servoing_widgets'] = dict(frames=widgets.frames, clicks=widgets.clicks,
                os_window=False, gl_ui_renderer=False)
            widgets.close()
        if recorder is not None:
            evidence['video'] = recorder.close()
        save()
        service.close()
        sim.close()
    if args.record_video and not evidence.get('video',{}).get('complete'):
        raise RuntimeError('video recording failed: '+str(evidence.get('video')))


if __name__ == '__main__':
    main()
