#!/usr/bin/env python3
"""Open or render the production operator UI with a local, inert data fixture.

PNG mode uses EGL/Mesa; interactive mode uses the desktop GLFW window.
No DDS participants, camera devices, Docker operations or motion executors
are created. Images are illustrative.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS


ROOT = Path(__file__).resolve().parents[2]


SCENARIOS = ('offline', 'ready', 'scanning', 'reconstructed', 'failed', 'robot', 'paused')


def preview_panel(*, robot_camera=False, scenario='offline'):
    import numpy as np
    import cv2
    from elesim_protocol import ControlU, SimMappingConfig, MockObjectStatePayload, SimulationStatusPayload
    from elesim_ui.control_panel import ControlPanel
    from elesim_ui.models import GazeStabilizerConfig, PickConfig
    from elesim_ui.operator import RemoteControlService, RemotePanelState

    class Cache:
        status = NS(pilot_online=False)

        def __init__(self):
            self.values, self.services, self.requests, self.callbacks = {}, {}, [], []

        def seed_state(self, values):
            self.values.update(values)

        def state_value(self, name, default=None):
            return self.values.get(name, default)

        def service_value(self, name, default=None):
            return self.services.get(name, default)

        def submit(self, *args, **kwargs):
            self.requests.append((args, kwargs))
            del self.requests[:-256]
            if args[0] == 'state_set':
                self.values[args[1]] = kwargs['value']
            elif args[0] == 'state_call' and args[1].startswith('set_'):
                field = args[1][4:]
                if field in self.values and len(args) == 3:
                    self.values[field] = args[2]
            if kwargs.get('on_error'):
                self.callbacks.append(kwargs['on_error'])
            return 'preview-request'

        def dispatch_callbacks(self):
            callbacks, self.callbacks = self.callbacks, []
            for callback in callbacks:
                callback('UI preview: no runtime is connected.')

        def request_snapshot(self):
            return 'preview-snapshot'

        def close(self):
            self.callbacks.clear()

    cache = Cache()
    cache.services.update(available_endpoints=[dict(endpoint_id='sim-preview', role='sim')],
        active_endpoint='sim-preview', current_control_u=ControlU(0., 0., 0., 0.),
        control_mapping=SimMappingConfig(), has_client=False, pick_config=PickConfig(),
        gaze_config=GazeStabilizerConfig(),
        object_scan=dict(phase='idle', active=False, can_start=False,
            blocked_reason='UI preview: no runtime is connected.',
            accepted_views=0, observed_points=0, predicted_points=0))
    state = RemotePanelState(cache)
    service = RemoteControlService(cache, state)

    class Camera:
        def __init__(self):
            self.active_sim_id = self.active_sim_boot_id = ''
            self.snapshot = NS(requested_sim_id='sim-preview', active_sim_id='',session_id='',
                connected_streams=('observer', 'hand_eye_preview'),last_error='',
                pending_commands=0,last_result=None,status=None)
            self.supports_scene_rebuild = False
            self.frames = {}
            for name, title, color in (('observer', 'OBSERVER / UI PREVIEW', (90, 135, 155)),
                                        ('hand_eye_preview', 'HAND-EYE / UI PREVIEW', (100, 125, 90))):
                frame = np.full((600, 800, 3), color, dtype=np.uint8)
                for x in range(0, 800, 50):
                    cv2.line(frame, (x, 0), (x, 599), (160, 170, 175), 1)
                for y in range(0, 600, 50):
                    cv2.line(frame, (0, y), (799, y), (160, 170, 175), 1)
                # Illustrative jar silhouette, never a synthetic sensor capture.
                contour = np.array([(350,200),(450,200),(450,225),(465,245),
                    (475,295),(465,330),(475,370),(465,425),(335,425),
                    (325,370),(335,330),(325,295),(335,245),(350,225)])
                cv2.fillPoly(frame,[contour],(70,80,190))
                cv2.ellipse(frame,(400,202),(50,12),0,0,360,(115,125,225),-1)
                cv2.putText(frame, title, (30, 65), cv2.FONT_HERSHEY_SIMPLEX, .8, (245, 245, 245), 2)
                if name == 'hand_eye_preview':
                    frame = cv2.flip(frame, -1)  # The operator renderer compensates the mount roll.
                self.frames[name] = frame

        def frame(self, name):
            return self.frames.get(name)

        def frame_version(self, name):
            return 1

        def sync_arm_ghost(self, value):
            pass

        def sync_ik_target(self, *args):
            pass

        def send_command(self, *args):
            cache.requests.append(args)
            del cache.requests[:-256]
            self.snapshot.last_error = 'UI preview: no runtime is connected.'
            return ''

        def load_scan_result(self, _status):
            return self.send_command('load_scene')

        def switch_target(self, _target):
            pass

    camera = Camera()
    previews = {}
    if robot_camera:
        for key in ('zed_preview', 'zed_right_preview'):
            previews[key] = NS(frame=camera.frames['observer'], frame_version=1,
                               status='LIVE', source_id='robot-preview', last_error='')
    panel = ControlPanel(state, service, sim_session=camera, use_go2=True, **previews)
    if scenario not in SCENARIOS:
        raise ValueError('Unknown UI preview scenario: '+scenario)
    if scenario != 'offline':
        cache.status = NS(pilot_online=True)
        cache.services.update(has_client=True,current_host_state=NS(connected=True,
            torque_enabled=True,safety_fault='',device='',ports=(),tx_seq=0,rx_age_s=0.,
            reply_ok=True,reply_reason='',motor_positions_raw={},motor_positions_deg={},
            motor_currents_ma={},actual_tip_xyz=None,actual_tip_dir=None,
            perceived_timestamp_s=0.,perceived_center_uv=None,perceived_scale=None,
            perceived_object_label=''),
            available_endpoints=[dict(endpoint_id='sim-preview',role='sim',instance_id='boot-preview',
                capabilities=['simulation.mock_hug.v1'])])
        cache.services['object_scan'].update(can_start=True,blocked_reason='')
        camera.active_sim_id,camera.active_sim_boot_id = 'sim-preview','boot-preview'
        camera.supports_scene_rebuild = True
        camera.snapshot.active_sim_id,camera.snapshot.session_id = 'sim-preview','preview-session'
        camera.snapshot.status = SimulationStatusPayload(1,False,1.,False,12.,
            mock_object=MockObjectStatePayload(available_assets=('jar','demo_box','demo_cylinder'),
                state='spawned',asset_id='jar',revision=1))
        panel.servoing.asset_id = 'jar'
        if scenario in ('scanning','reconstructed','failed'):
            cache.services['object_scan'].update(job_id='preview-job',accepted_views=6,
                observed_points=18462,predicted_points=16384,
                coverage=dict(fraction=.68,unobserved_directions=[[0.,0.,-1.],[0.,-1.,0.]]))
            panel.servoing.step = 1
        if scenario == 'scanning':
            cache.services['object_scan'].update(active=True,can_start=False,phase='capturing',
                stage='choosing_view',view=6,views=12)
        elif scenario == 'reconstructed':
            panel.servoing.step = 2
            cache.services['object_scan'].update(phase='completed',sim_id='sim-preview',
                sim_boot_id='boot-preview',scene_reference='scan-preview/scene.json',
                scene_revision='preview-revision',needs_more_observation=True,
                stop_reason='view_budget_exhausted',
                model_execution={'scope':'Illustrative fixture; no models executed'})
        elif scenario == 'failed':
            cache.services['object_scan'].update(phase='failed',
                reason='Illustrative failure: completion model assets unavailable on Pilot.')
        elif scenario == 'robot':
            cache.services.update(active_endpoint='robot-preview',available_endpoints=[
                dict(endpoint_id='robot-preview',role='robot',instance_id='robot-boot-preview')])
            cache.services['object_scan'].update(can_start=False,
                blocked_reason='Automatic live scanning currently supports Sim endpoints only.')
            panel.servoing.step = 1
        elif scenario == 'paused':
            from dataclasses import replace
            camera.snapshot.status = replace(camera.snapshot.status,paused=True)
            cache.services['object_scan'].update(can_start=False,
                blocked_reason='Resume Sim before starting a scan.')
            panel.servoing.step = 1
    return panel, cache


def configure_panel(tab, layout, controls, source, scale, scenario='offline', stage=None):
    panel, cache = preview_panel(robot_camera=source == 'robot',scenario=scenario)
    panel._workspace_tab_request = tab
    panel._workspace_controls_visible = controls
    panel._sim_view.state.main_stream = 'hand_eye_preview' if layout == 'hand-eye' else 'observer'
    panel._sim_view.state.preview_stream = 'observer' if layout == 'hand-eye' else 'hand_eye_preview'
    panel._sim_view.state.layout = 'split' if layout == 'split' else 'pip'
    panel._ui_resolution_requested_scale = panel._ui_resolution_scale = scale
    if stage is not None:
        panel.servoing.step = ('scene','scan','reconstruction','action').index(stage)
    panel._preview_scenario = scenario
    return panel, cache


def interactive(tab, layout, controls, source, scale, scenario='offline', stage=None):
    panel, _cache = configure_panel(tab, layout, controls, source, scale,scenario,stage)
    print('[ui-preview] Opening local mock UI; no runtime or hardware is connected.', flush=True)
    try:
        panel.run(window_title='EleSim - Mock UI')
    except KeyboardInterrupt:
        pass
    finally:
        panel.service.close()


def render(output: Path, width: int, height: int, tab: str, layout: str, controls: bool,
           source: str, scale: float, scenario='offline', stage=None):
    import cv2
    import numpy as np
    import imgui
    from OpenGL import EGL, GL
    from imgui.integrations.opengl import ProgrammablePipelineRenderer

    display = EGL.eglGetDisplay(EGL.EGL_DEFAULT_DISPLAY)
    major, minor = ctypes.c_int(), ctypes.c_int()
    EGL.eglInitialize(display, major, minor)
    config = (EGL.EGLConfig*1)()
    count = ctypes.c_int()
    attributes = (ctypes.c_int*13)(EGL.EGL_SURFACE_TYPE, EGL.EGL_PBUFFER_BIT,
        EGL.EGL_RENDERABLE_TYPE, EGL.EGL_OPENGL_BIT, EGL.EGL_RED_SIZE, 8,
        EGL.EGL_GREEN_SIZE, 8, EGL.EGL_BLUE_SIZE, 8, EGL.EGL_ALPHA_SIZE, 8, EGL.EGL_NONE)
    EGL.eglChooseConfig(display, attributes, config, 1, count)
    if not count.value:
        raise RuntimeError('EGL OpenGL pbuffer is unavailable')
    EGL.eglBindAPI(EGL.EGL_OPENGL_API)
    surface = EGL.eglCreatePbufferSurface(display, config[0],
        (ctypes.c_int*5)(EGL.EGL_WIDTH, width, EGL.EGL_HEIGHT, height, EGL.EGL_NONE))
    context = EGL.eglCreateContext(display, config[0], EGL.EGL_NO_CONTEXT,
                                  (ctypes.c_int*1)(EGL.EGL_NONE))
    EGL.eglMakeCurrent(display, surface, surface, context)
    ui_context = imgui.create_context()
    io = imgui.get_io()
    io.ini_file_name = None
    io.display_size = width, height
    io.delta_time = 1/30
    panel, cache = configure_panel(tab, layout, controls, source, scale,scenario,stage)
    panel._install_ui_font()
    panel._install_ui_style()
    panel.set_ui_resolution_scale(scale)
    renderer = ProgrammablePipelineRenderer()
    try:
        for _ in range(4):
            GL.glViewport(0, 0, width, height)
            GL.glClearColor(.94, .95, .96, 1.)
            GL.glClear(GL.GL_COLOR_BUFFER_BIT)
            imgui.new_frame()
            panel._draw_controls_window()
            imgui.render()
            renderer.render(imgui.get_draw_data())
        pixels = GL.glReadPixels(0, 0, width, height, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE)
        pixels = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, 4)[::-1]
        output.parent.mkdir(parents=True, exist_ok=True)
        if output.exists():
            raise FileExistsError(output)
        if not cv2.imwrite(str(output), cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGRA)):
            raise RuntimeError('Could not save preview PNG')
        return dict(output=str(output), width=width, height=height, tab=tab, layout=layout,
                    source=source, scale=scale, controls=controls,scenario=scenario,stage=stage,
                    renderer=GL.glGetString(GL.GL_RENDERER).decode(),
                    requests=len(cache.requests), scope='inert UI fixture; no runtime or hardware')
    finally:
        panel._sim_view.close()
        renderer.shutdown()
        imgui.destroy_context(ui_context)
        EGL.eglMakeCurrent(display, EGL.EGL_NO_SURFACE, EGL.EGL_NO_SURFACE, EGL.EGL_NO_CONTEXT)
        EGL.eglDestroyContext(display, context)
        EGL.eglDestroySurface(display, surface)
        EGL.eglTerminate(display)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--output', type=Path, help='save an offscreen PNG')
    mode.add_argument('--interactive', action='store_true', help='open the native mock UI window')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=800)
    parser.add_argument('--tab', choices=('Visual Servoing', 'Manual', 'Status'), default='Visual Servoing')
    parser.add_argument('--scenario',choices=SCENARIOS,default='offline',help='inert workflow status fixture')
    parser.add_argument('--stage',choices=('scene','scan','reconstruction','action'),help='open a workflow step')
    parser.add_argument('--layout', choices=('observer', 'hand-eye', 'split'), default='observer')
    parser.add_argument('--source', choices=('sim', 'robot'), default='sim')
    parser.add_argument('--scale', type=float, choices=(.8, 1., 1.2), default=1.)
    parser.add_argument('--camera-only', action='store_true')
    args = parser.parse_args()
    if not 640 <= args.width <= 3840 or not 480 <= args.height <= 2160:
        parser.error('dimensions must be 640..3840 by 480..2160')
    sys.path[:0] = [str(ROOT/'payload/runtime/docker/ui/app'), str(ROOT/'payload/runtime/common/protocol')]
    if args.interactive:
        if sys.platform.startswith('linux') and os.environ.get('DISPLAY'):
            # WSLg exposes WAYLAND_DISPLAY as well, so PyOpenGL otherwise
            # auto-selects EGL while the installed GLFW library uses X11/GLX.
            os.environ['PYOPENGL_PLATFORM'] = 'glx'
        interactive(args.tab, args.layout, not args.camera_only, args.source, args.scale,args.scenario,args.stage)
        return
    os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
    os.environ.setdefault('EGL_PLATFORM', 'surfaceless')
    os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
    print(json.dumps(render(args.output, args.width, args.height, args.tab,
                            args.layout, not args.camera_only, args.source, args.scale,args.scenario,args.stage)))


if __name__ == '__main__':
    main()
