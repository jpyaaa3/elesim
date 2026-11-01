"""Record the production desktop UI with live Sim data and X11 mouse input.

Run only in the isolated scan-cycle UI image on a private Xvfb display. No
sensor frames, operator responses, controller methods or rendered pixels are
replaced. The original Sim fixture remains visible when its mesh is applied.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import traceback
from unittest.mock import patch


class WindowDemo:
    def __init__(self, output):
        self.output = output
        self.panel = None
        self.rectangles = {}
        self.frame_rectangles = {}
        self.window_id = None
        self.frame_count = 0
        self.recording = None
        self.record_log = None
        self.cursor = None
        self.error = None
        self.evidence = dict(complete=False, stage='starting', started=time.time(),
            scope='production ControlPanel.run; XTest input; full native window X11 capture; actual DDS/WebRTC/Genesis',
            source_fixture='visible overlay; no backstage object hiding', timeline=[], clicks=[],
            widget_presses=[],mouse_edges=[])
        self.last_mouse = None

    def save(self):
        (self.output/'ui.json').write_text(json.dumps(self.evidence,indent=2,allow_nan=False)+'\n')

    def wait(self, label, condition, timeout=60.):
        self.evidence['stage'] = label
        self.save()
        deadline = time.monotonic()+timeout
        next_status = time.monotonic()+5.
        while time.monotonic() < deadline:
            if condition():
                self.evidence['timeline'].append(dict(event=label,time=time.time()))
                self.save()
                return
            if self.panel is not None and time.monotonic() >= next_status:
                sim = self.panel._sim_view.session
                self.evidence['readiness'] = dict(pilot_client=self.panel.service.has_client(),
                    sim_id=sim.active_sim_id,scene_rebuild=sim.supports_scene_rebuild,
                    status_received=sim.status is not None,streams=list(sim.connected_streams),
                    camera_source=self.panel._sim_view.state.camera_source,
                    scan=self.panel.service.object_scan.get('phase'))
                self.save()
                next_status = time.monotonic()+5.
            time.sleep(.05)
        raise TimeoutError(label)

    def xdo(self, *args):
        return subprocess.check_output(['xdotool',*map(str,args)],text=True,timeout=10).strip()

    def widget(self, label):
        return next((value for key,value in self.rectangles.items()
                     if key == label or key.endswith(label)), None)

    def click(self, label, *, scroll=False, control=False):
        self.wait('find '+label,lambda:self.widget(label) is not None,15.)
        for _ in range(14 if scroll else 1):
            item = self.widget(label)
            if item and item['visible'] and 0 < item['y'] < 1080:
                break
            self.xdo('mousemove','--window',self.window_id,250,800)
            self.xdo('click','--repeat',3,'--delay',80,5)
            time.sleep(.2)
        item = self.widget(label)
        assert item and item['visible'] and 0 < item['y'] < 1080,(label,item)
        self.xdo('mousemove','--sync','--window',self.window_id,round(item['x']),round(item['y']))
        time.sleep(.25)
        # Keep one XTest client alive for the whole gesture. Separate
        # xdotool invocations can release injected buttons when they exit.
        gesture = (['keydown','ctrl'] if control else [])
        gesture += ['mousedown',1,'sleep','.6','mouseup',1]
        if control:
            gesture += ['keyup','ctrl']
        self.xdo(*gesture)
        self.evidence['clicks'].append(dict(label=label,time=time.time(),x=item['x'],y=item['y']))
        time.sleep(.45)

    def navigate(self, step):
        self.click('##vs-step-'+str(step))
        self.wait('stage '+str(step),lambda:self.panel.servoing.step == step,10.)

    def capture(self, name):
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-n',
            '-f','x11grab','-video_size','1920x1080','-i',os.environ['DISPLAY']+'.0',
            '-frames:v','1',str(self.output/(name+'.png'))],check=True,timeout=20)

    def begin_recording(self):
        self.record_log = (self.output/'window-ffmpeg.log').open('w')
        self.recording = subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-n',
            '-thread_queue_size','512','-f','x11grab','-draw_mouse','1','-framerate','30',
            '-video_size','1920x1080','-i',os.environ['DISPLAY']+'.0',
            '-c:v','libx264','-preset','veryfast','-crf','18','-pix_fmt','yuv420p',
            '-movflags','+faststart',str(self.output/'window.mp4')],
            stdin=subprocess.PIPE,stdout=self.record_log,stderr=subprocess.STDOUT)
        self.evidence['recording_started'] = time.time()
        time.sleep(1.)
        assert self.recording.poll() is None,'X11 recording did not start'

    def end_recording(self):
        if self.recording is None:
            return
        try:
            self.recording.communicate(b'q\n',timeout=30)
        except BaseException:
            self.recording.terminate()
            self.recording.wait(timeout=10)
            raise
        finally:
            self.record_log.close()
        metadata = json.loads(subprocess.check_output(['ffprobe','-v','error',
            '-select_streams','v:0','-show_entries',
            'stream=width,height,codec_name,nb_frames,avg_frame_rate:format=duration',
            '-of','json',str(self.output/'window.mp4')],text=True,timeout=20))
        self.evidence['video'] = dict(complete=self.recording.returncode == 0,
            file='window.mp4',clock='actual X11 desktop wall time; no speed change',
            metadata=metadata,exit_code=self.recording.returncode)

    def automate(self):
        from probe_scan_cycle_ui import check_poisson_surface
        try:
            self.wait('native window ready',lambda:self.panel is not None
                and self.panel._glfw_window is not None and self.frame_count >= 3,60.)
            import glfw
            self.window_id = int(glfw.get_x11_window(self.panel._glfw_window))
            self.xdo('windowsize','--sync',self.window_id,1920,1080)
            self.xdo('windowmove','--sync',self.window_id,0,0)
            self.xdo('windowfocus','--sync',self.window_id)
            service = self.panel.service
            sim = self.panel._sim_view.session
            c = self.panel.servoing
            self.wait('two live cameras',lambda:service.has_client() and sim.status is not None
                and sim.supports_scene_rebuild and all(sim.frame(s) is not None
                    for s in ('observer','hand_eye_preview')),900.)
            self.begin_recording()
            time.sleep(2.)
            self.capture('window-before')
            self.click('Manual')
            self.wait('manual tab',lambda:self.panel._workspace_tab == 'Manual')
            if not service.current_host_state().torque_enabled:
                self.click('##hardware_torque_switch')
                self.wait('torque feedback',lambda:service.current_host_state().torque_enabled,15.)
            if service.current_host_state().claw_closed:
                self.click('##gripper_close_switch',scroll=True)
                self.wait('open gripper feedback',lambda:not service.current_host_state().claw_closed,15.)
            self.click('Visual Servoing')
            self.wait('visual servoing tab',lambda:self.panel._workspace_tab == 'Visual Servoing')
            self.navigate(1)
            self.wait('scan preflight',lambda:not c.reason('scan'),60.)
            self.click('View budget##vs-views',control=True)
            self.xdo('key','ctrl+a')
            self.xdo('type','--clearmodifiers','--delay',80,'3')
            self.xdo('key','Return')
            self.wait('three-view budget',lambda:c.max_views == 3,10.)
            time.sleep(1.)
            self.click('Start scan & reconstruction##vs-scan')
            self.wait('active scan',lambda:service.object_scan.get('active'),15.)
            self.wait('scan and Poisson completion',lambda:service.object_scan.get('phase')
                in ('completed','failed','cancelled'),620.)
            status = service.object_scan
            self.evidence['scan'] = status
            assert status['phase'] == 'completed',status
            assert status['accepted_views'] >= 2 and status['observed_points'] > 0
            self.evidence['poisson_surface'] = check_poisson_surface(status)
            self.wait('return of scan control',lambda:not c.view().busy,30.)
            self.navigate(2)
            self.wait('apply preflight',lambda:not c.reason('apply'),30.)
            time.sleep(2.)
            self.capture('window-reconstruction')
            before = {s:sim.frame_version(s) or 0 for s in ('observer','hand_eye_preview')}
            self.click('Apply reconstruction to Sim##vs-apply',scroll=True)
            self.wait('matching Apply acknowledgement',lambda:not c.pending
                and c.scene_outcome == 'applied',120.)
            assert not c.error,c.error
            self.evidence['application'] = asdict(sim.snapshot.last_result)
            assert self.evidence['application']['ok']
            self.wait('fresh cameras after apply',lambda:all((sim.frame_version(s) or 0)
                > before[s]+4 for s in before),30.)
            self.xdo('mousemove','--window',self.window_id,1500,950)
            self.capture('window-applied')
            time.sleep(4.)
            self.navigate(3)
            assert all(c.reason(action) for action in ('gripper','embrace','verify'))
            time.sleep(3.)
            self.navigate(2)
            time.sleep(3.)
            self.evidence.update(complete=True,stage='complete',finished=time.time(),
                scene_revision=status['scene_revision'],sim_boot_id=sim.active_sim_boot_id)
        except BaseException as error:
            self.error = error
            self.evidence.update(error=str(error),traceback=traceback.format_exc(),finished=time.time())
            try:
                self.capture('window-failed')
            except Exception:
                pass
        finally:
            try:
                self.end_recording()
            except BaseException as error:
                self.error = error
                self.evidence.update(complete=False,recording_error=str(error))
            self.evidence['rendered_ui_frames'] = self.frame_count
            self.save()
            if self.panel is not None:
                self.panel.stop()

    def run_panel(self, panel, original, **kwargs):
        self.panel = panel
        panel._sim_view.state.camera_source = 'sim'
        panel._workspace_controls_width = 480.
        worker = threading.Thread(target=self.automate,name='native-ui-demo',daemon=True)
        worker.start()
        original(panel,window_title='EleSim - Poisson Visual Servoing Demo')
        worker.join(timeout=40.)
        assert not worker.is_alive(),'native window automation did not stop'

    def draw(self, panel, original):
        import imgui
        io = imgui.get_io()
        mouse = tuple(bool(io.mouse_down[i]) for i in range(3))
        if mouse != self.last_mouse:
            self.evidence['mouse_edges'].append(dict(time=time.time(),buttons=mouse,
                position=list(io.mouse_pos),frame=self.frame_count))
            self.last_mouse = mouse
        self.frame_rectangles = {}
        result = original(panel)
        self.rectangles = self.frame_rectangles
        self.frame_count += 1
        if 'ui_renderer' not in self.evidence:
            import glfw
            self.cursor = glfw.create_standard_cursor(glfw.ARROW_CURSOR)
            if self.cursor:
                glfw.set_cursor(panel._glfw_window,self.cursor)
            from OpenGL.GL import glGetString,GL_RENDERER,GL_VENDOR
            self.evidence['ui_renderer'] = dict(renderer=glGetString(GL_RENDERER).decode(),
                vendor=glGetString(GL_VENDOR).decode())
        return result

    def observe(self, function):
        def wrapped(label,*args,**kwargs):
            import imgui
            result = function(label,*args,**kwargs)
            if type(result) is bool and result:
                self.evidence['widget_presses'].append(dict(label=label,time=time.time(),
                    frame=self.frame_count))
            a,b = imgui.get_item_rect_min(),imgui.get_item_rect_max()
            self.frame_rectangles[label] = dict(x=(a.x+b.x)/2,y=(a.y+b.y)/2,
                visible=imgui.is_item_visible())
            return result
        return wrapped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--config',default='/opt/elesim/config/installed.yaml')
    parser.add_argument('--pilot',required=True)
    parser.add_argument('--sim',required=True)
    args = parser.parse_args()
    for sibling in ('elesim_sim','elesim_pilot','elesim_robot'):
        assert importlib.util.find_spec(sibling) is None,sibling
    from contextlib import ExitStack
    import imgui
    from elesim_ui.control_panel import ControlPanel
    from elesim_ui.main import main as run_ui
    args.evidence.mkdir(parents=True,exist_ok=True)
    assert not (args.evidence/'ui.json').exists(),'preserve the previous attempt'
    demo = WindowDemo(args.evidence)
    original_run,original_draw = ControlPanel.run,ControlPanel._draw_controls_window
    with ExitStack() as stack:
        for name in ('button','invisible_button','begin_tab_item','slider_int'):
            stack.enter_context(patch.object(imgui,name,demo.observe(getattr(imgui,name))))
        stack.enter_context(patch.object(ControlPanel,'run',
            lambda panel,**kwargs:demo.run_panel(panel,original_run,**kwargs)))
        stack.enter_context(patch.object(ControlPanel,'_draw_controls_window',
            lambda panel:demo.draw(panel,original_draw)))
        sys.argv = ['elesim-ui','--config',args.config,'--pilot-id',args.pilot,'--sim-id',args.sim]
        run_ui()
    if demo.error is not None:
        raise RuntimeError('Native UI demonstration failed') from demo.error
    assert demo.evidence['complete'] and demo.evidence['video']['complete'],demo.evidence


if __name__ == '__main__':
    main()
