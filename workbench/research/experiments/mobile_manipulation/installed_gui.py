"""Render the published operator panel and exercise its real ImGui buttons.

A hidden GLFW window uses the installed UI's normal graphics stack. An optional
EGL surface supports headless development fixtures. Input stays inside ImGui;
no desktop mouse events are sent. Button observation records item rectangles
only; the original return value and controller decide whether an action is sent.
"""
from __future__ import annotations

import ctypes
import os
import time


class InstalledGui:
    def __init__(self, service, sim, output, *, backend='glfw'):
        if backend not in {'glfw', 'egl'}:
            raise ValueError('Unknown installed UI graphics backend')
        self.backend = backend
        os.environ['PYOPENGL_PLATFORM'] = 'egl' if backend == 'egl' else 'glx'
        if backend == 'egl':
            os.environ.setdefault('EGL_PLATFORM', 'surfaceless')
            os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
        import imgui
        from OpenGL import GL
        from imgui.integrations.opengl import ProgrammablePipelineRenderer
        from elesim_ui.control_panel import ControlPanel

        self.imgui, self.gl = imgui, GL
        self.output = output
        self.width, self.height = 1600, 1200
        if backend == 'egl':
            self._init_egl()
        else:
            import glfw
            self.glfw = glfw
            if not glfw.init():
                raise RuntimeError('Installed UI GLFW initialization failed')
            glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
            glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
            glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
            glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
            self.window = glfw.create_window(self.width, self.height, 'EleSim UI verification', None, None)
            if not self.window:
                glfw.terminate()
                raise RuntimeError('Installed UI GLFW window creation failed')
            glfw.make_context_current(self.window)
        self.ui_context = imgui.create_context()
        self.io = imgui.get_io()
        self.io.ini_file_name = None
        self.io.display_size = self.width, self.height
        self.io.delta_time = 1 / 30
        self.panel = ControlPanel(service.state, service, sim_session=sim, use_go2=True)
        self.panel._workspace_tab_request = 'Visual Servoing'
        self.panel._workspace_controls_visible = True
        self.panel._install_ui_font()
        self.panel._install_ui_style()
        self.renderer = ProgrammablePipelineRenderer()
        self.buttons, self.clicks = {}, []
        self.frames = 0
        self.last_frame = 0.
        self.renderer_name = GL.glGetString(GL.GL_RENDERER).decode()

    def _init_egl(self):
        from OpenGL import EGL
        self.egl = EGL
        self.display = EGL.eglGetDisplay(EGL.EGL_DEFAULT_DISPLAY)
        major, minor = ctypes.c_int(), ctypes.c_int()
        if not EGL.eglInitialize(self.display, major, minor):
            raise RuntimeError('Installed UI EGL initialization failed')
        configs, count = (EGL.EGLConfig * 1)(), ctypes.c_int()
        attributes = (ctypes.c_int * 13)(EGL.EGL_SURFACE_TYPE, EGL.EGL_PBUFFER_BIT,
            EGL.EGL_RENDERABLE_TYPE, EGL.EGL_OPENGL_BIT, EGL.EGL_RED_SIZE, 8,
            EGL.EGL_GREEN_SIZE, 8, EGL.EGL_BLUE_SIZE, 8, EGL.EGL_ALPHA_SIZE, 8, EGL.EGL_NONE)
        EGL.eglChooseConfig(self.display, attributes, configs, 1, count)
        if not count.value:
            raise RuntimeError('Installed UI EGL pbuffer unavailable')
        EGL.eglBindAPI(EGL.EGL_OPENGL_API)
        self.surface = EGL.eglCreatePbufferSurface(self.display, configs[0],
            (ctypes.c_int * 5)(EGL.EGL_WIDTH, self.width, EGL.EGL_HEIGHT, self.height, EGL.EGL_NONE))
        self.context = EGL.eglCreateContext(self.display, configs[0], EGL.EGL_NO_CONTEXT,
            (ctypes.c_int * 1)(EGL.EGL_NONE))
        EGL.eglMakeCurrent(self.display, self.surface, self.surface, self.context)

    @property
    def controller(self):
        return self.panel.servoing

    def draw(self, *, force=False):
        now = time.monotonic()
        if not force and now - self.last_frame < .1:
            return
        if self.backend == 'glfw':
            self.glfw.poll_events()
        self.panel._host_state = self.panel.service.current_host_state()
        self.panel.sync_offset_drafts()
        imgui, GL = self.imgui, self.gl
        original = imgui.button
        self.buttons.clear()

        def observe_button(label, *args, **kwargs):
            pressed = original(label, *args, **kwargs)
            if '##vs-' in label and imgui.is_item_visible():
                lo, hi = imgui.get_item_rect_min(), imgui.get_item_rect_max()
                self.buttons[label.split('##vs-', 1)[1]] = (tuple(lo), tuple(hi))
            return pressed

        imgui.button = observe_button
        try:
            GL.glViewport(0, 0, self.width, self.height)
            GL.glClearColor(.94, .95, .96, 1.)
            GL.glClear(GL.GL_COLOR_BUFFER_BIT)
            imgui.new_frame()
            self.panel._draw_controls_window()
            imgui.render()
            self.renderer.render(imgui.get_draw_data())
        finally:
            imgui.button = original
        self.frames += 1
        self.last_frame = now

    def click(self, action, tick):
        self.controller.step = 0 if action == 'spawn' else 3
        for _ in range(3):
            self.draw(force=True)
        if action not in self.buttons:
            self.screenshot('missing-' + action)
            raise RuntimeError('Installed UI button is not visible: ' + action)
        lo, hi = self.buttons[action]
        point = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2)
        if not (0 <= point[0] < self.width and 0 <= point[1] < self.height):
            raise RuntimeError('Installed UI button is outside the viewport: ' + action)
        self.screenshot('before-' + action)
        self.io.mouse_pos = point
        self.draw(force=True)
        self.io.mouse_down[0] = True
        self.draw(force=True)
        self.io.mouse_down[0] = False
        self.draw(force=True)
        tick()
        self.clicks.append(dict(action=action, position=list(point), wall=time.time()))
        self.screenshot('clicked-' + action)

    def screenshot(self, name):
        import av
        import numpy as np
        from fractions import Fraction
        pixels = self.gl.glReadPixels(0, 0, self.width, self.height,
            self.gl.GL_RGBA, self.gl.GL_UNSIGNED_BYTE)
        pixels = np.frombuffer(pixels, dtype=np.uint8).reshape(self.height, self.width, 4)[::-1]
        path = self.output / ('ui-' + name + '.png')
        codec = av.CodecContext.create('png', 'w')
        codec.width, codec.height = self.width, self.height
        codec.pix_fmt, codec.time_base = 'rgba', Fraction(1, 1)
        frame = av.VideoFrame.from_ndarray(pixels.copy(), format='rgba')
        frame.pts = 0
        packets = codec.encode(frame) + codec.encode(None)
        path.write_bytes(b''.join(bytes(packet) for packet in packets))

    def close(self):
        self.panel._sim_view.close()
        self.renderer.shutdown()
        self.imgui.destroy_context(self.ui_context)
        if self.backend == 'egl':
            EGL = self.egl
            EGL.eglMakeCurrent(self.display, EGL.EGL_NO_SURFACE, EGL.EGL_NO_SURFACE, EGL.EGL_NO_CONTEXT)
            EGL.eglDestroyContext(self.display, self.context)
            EGL.eglDestroySurface(self.display, self.surface)
            EGL.eglTerminate(self.display)
        else:
            self.glfw.destroy_window(self.window)
            self.glfw.terminate()
