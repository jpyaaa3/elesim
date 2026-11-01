"""Workspace visibility, sizing and video focus without a runtime participant."""
from types import SimpleNamespace as NS

import imgui
import numpy as np
import pytest

from elesim_ui import control_panel, sim_view, workspace
from elesim_ui.sim_view import SimView, _pip_rect
from workbench.tools.ui_preview import preview_panel


@pytest.fixture
def ui_context():
    context = imgui.create_context()
    io = imgui.get_io()
    io.ini_file_name = None
    io.delta_time = 1/30
    io.fonts.get_tex_data_as_rgba32()
    yield io
    imgui.destroy_context(context)


@pytest.mark.parametrize('size', [(1920,1080), (1920,1200), (1366,768), (1280,800), (800,600), (640,480)])
@pytest.mark.parametrize('tab', ['Scan', 'Manual', 'Legacy', 'Status'])
def test_only_selected_work_controls_draw_and_camera_stays_in_window(monkeypatch, ui_context, size, tab):
    panel, cache = preview_panel()
    panel._sim_view = None
    panel._workspace_tab_request = tab
    calls, regions = [], []
    monkeypatch.setattr(workspace, '_TABS', {
        name: (lambda _panel, name=name: calls.append(name),) for name in workspace._TABS})

    def camera():
        position = imgui.get_cursor_screen_pos()
        available = imgui.get_content_region_available()
        regions.append((position.x, position.y, available.x, available.y))

    panel._draw_camera_contents = camera
    ui_context.display_size = size
    before = list(cache.requests)
    for _ in range(4):
        imgui.new_frame()
        panel._draw_controls_window()
        imgui.render()
    # ImGui applies SET_SELECTED after registering the new tab on its first frame.
    assert len(calls) == 4 and calls[-3:] == [tab]*3
    x, y, width, height = regions[-1]
    assert 0 <= x < size[0] and 0 <= y < size[1]
    assert width > 100 and height > 50
    assert x+width <= size[0] and y+height <= size[1]
    assert cache.requests == before  # Viewing a tab never starts a workflow.


def test_hidden_manual_pad_releases_velocity_once_and_final_slider_draft():
    panel, cache = preview_panel()
    panel._go2_was_active = True
    panel.update_control_u_draft({'s1': 12.})
    before = len(cache.requests)
    workspace.release_manual_input(panel)
    workspace.release_manual_input(panel)
    requests = [request[0] for request in cache.requests[before:]]
    assert [request[1] for request in requests] == ['send_go2_velocity', 'apply_partial_control_u']
    assert cache.requests[before][1] == dict(vx=0., vy=0., wz=0.)
    assert panel._go2_respawn_blocked and not panel._go2_was_active
    assert not panel._control_u_dragging and not panel._control_u_dirty


def test_mock_ui_poll_dispatches_local_rejection_and_bounds_request_history():
    panel, cache = preview_panel()
    errors = []
    panel.service.object_scan_async('start_object_scan', 'target', 8,
        on_result=lambda _: pytest.fail('mock started a real scan'), on_error=errors.append)
    panel.service.poll()
    panel.service.poll()
    assert errors == ['UI preview: no runtime is connected.']
    assert not panel.service.object_scan['active']
    for _ in range(300):
        panel.state.visual_target_uv_u = .1
    assert len(cache.requests) == 256
    panel.service.close()


def test_interactive_mock_selects_glx_before_loading_ui_on_wslg(monkeypatch):
    from workbench.tools import ui_preview
    import os
    seen = []
    monkeypatch.setattr(ui_preview.sys, 'argv', ['ui_preview', '--interactive'])
    monkeypatch.setattr(ui_preview.sys, 'path', list(ui_preview.sys.path))
    monkeypatch.setattr(ui_preview.sys, 'platform', 'linux')
    monkeypatch.setenv('DISPLAY', ':0')
    monkeypatch.setenv('WAYLAND_DISPLAY', 'wayland-0')
    monkeypatch.setenv('PYOPENGL_PLATFORM', 'egl')
    monkeypatch.setattr(ui_preview, 'interactive', lambda *args: seen.append(os.environ['PYOPENGL_PLATFORM']))
    ui_preview.main()
    assert seen == ['glx']


def test_tab_bar_stays_visible_while_legacy_controls_scroll(monkeypatch, ui_context):
    panel, _ = preview_panel()
    panel._sim_view = None
    panel._workspace_tab_request = 'Legacy'
    panel._draw_camera_contents = lambda: None
    tab_y, scroll_y = [], []
    begin_tab = imgui.begin_tab_item

    def tab(*args, **kwargs):
        result = begin_tab(*args, **kwargs)
        if args[0] == 'Scan':
            tab_y.append(imgui.get_item_rect_min().y)
        return result

    def long_controls(_panel):
        scroll_y.append(imgui.get_scroll_y())
        for row in range(100):
            imgui.text('Legacy setting '+str(row))

    monkeypatch.setattr(workspace.imgui, 'begin_tab_item', tab)
    monkeypatch.setattr(workspace, '_TABS', dict(Scan=(), Manual=(), Legacy=(long_controls,), Status=()))
    ui_context.display_size = 1280, 800
    ui_context.mouse_pos = 200, 300
    for index in range(7):
        if index == 4:
            ui_context.mouse_wheel = -5.
        imgui.new_frame()
        panel._draw_controls_window()
        imgui.render()
    assert max(scroll_y) > 0
    assert len(set(tab_y[-4:])) == 1


def test_scale_changes_text_without_resizing_native_window(monkeypatch, ui_context):
    panel, _ = preview_panel()
    panel._glfw_window = object()
    monkeypatch.setattr(control_panel.glfw, 'set_window_size', lambda *_: pytest.fail('window resized'))
    panel.set_ui_resolution_scale(.8)
    assert panel.ui_resolution_scale() == .8
    assert ui_context.font_global_scale == pytest.approx(.8)


def test_camera_window_is_created_only_on_explicit_detach_and_reused(monkeypatch):
    panel, _ = preview_panel()
    assert panel._camera_window is None and not panel._camera_visible
    events = []
    panel._create_camera_window = lambda: (events.append('create'), setattr(panel, '_camera_window', 'camera'))
    for name in ('show_window', 'focus_window', 'hide_window', 'set_window_should_close'):
        monkeypatch.setattr(control_panel.glfw, name, lambda *args, name=name: events.append(name))
    panel._show_camera_window()
    panel._hide_camera_window()
    assert not panel._camera_visible
    panel._show_camera_window()
    assert panel._camera_visible and events.count('create') == 1


def test_failed_detach_keeps_embedded_video_and_restores_context(monkeypatch, ui_context):
    panel, _ = preview_panel()
    panel._glfw_window = 'main'
    current, switches = imgui.get_current_context(), []
    monkeypatch.setattr(control_panel.glfw, 'window_hint', lambda *_: None)
    monkeypatch.setattr(control_panel.glfw, 'create_window', lambda *_: None)
    monkeypatch.setattr(control_panel.glfw, 'make_context_current', switches.append)
    panel._show_camera_window()
    assert not panel._camera_visible and panel._camera_window is None
    assert imgui.get_current_context() == current
    assert switches == ['main']
    assert panel._workspace_error


def test_detach_initializes_its_context_and_restores_main(monkeypatch, ui_context):
    panel, _ = preview_panel()
    panel._glfw_window = 'main'
    main, fonts = imgui.get_current_context(), []
    for name in ('window_hint', 'set_window_size_limits', 'make_context_current',
                 'swap_interval', 'show_window', 'focus_window'):
        monkeypatch.setattr(control_panel.glfw, name, lambda *_: None)
    monkeypatch.setattr(control_panel.glfw, 'create_window', lambda *_: 'camera')
    monkeypatch.setattr(control_panel, 'GlfwRenderer', lambda _: NS(shutdown=lambda: None))
    panel._install_ui_font = lambda **kwargs: fonts.append((imgui.get_current_context(), kwargs))
    panel._install_ui_style = lambda: None
    panel._install_respawn_shortcut = lambda *_: None
    panel._show_camera_window()
    try:
        assert panel._camera_visible
        assert panel._camera_imgui_context != main
        assert fonts == [(panel._camera_imgui_context, dict(install_header=False))]
        assert imgui.get_current_context() == main
    finally:
        imgui.destroy_context(panel._camera_imgui_context)
        imgui.set_current_context(main)


def test_detached_video_is_not_drawn_again_in_main_workspace(ui_context):
    panel, _ = preview_panel()
    drawn = []
    panel._workspace_controls_visible = False
    panel._camera_visible = True
    panel._draw_camera_contents = lambda: drawn.append('video')
    ui_context.display_size = 1280, 800
    for detached in (True, False):
        panel._camera_visible = detached
        imgui.new_frame()
        panel._draw_controls_window()
        imgui.render()
    assert drawn == ['video']


@pytest.mark.parametrize('layout,main,expected', [
    ('pip', 'observer', ['observer']),
    ('pip', 'hand_eye_preview', ['hand_eye_preview']),
    ('split', 'observer', ['observer', 'hand_eye_preview']),
])
def test_video_layout_uses_same_session_and_can_focus_hand_eye(monkeypatch, ui_context, layout, main, expected):
    session = NS(snapshot=NS(last_error='', connected_streams=('observer','hand_eye_preview')))
    view = SimView(session)
    view.state.layout, view.state.main_stream = layout, main
    view.state.preview_stream = 'observer' if main == 'hand_eye_preview' else 'hand_eye_preview'
    drawn, inset = [], []
    view._draw_stream = lambda name, **kwargs: (drawn.append(name), (20.,30.,600.,450.))
    view._draw_pip = lambda rect, **kwargs: inset.append(kwargs['stream'])
    ui_context.display_size = 1000, 700
    imgui.new_frame()
    imgui.begin('video')
    view.draw()
    imgui.end()
    imgui.render()
    assert view.session is session
    assert drawn == expected
    assert inset == ([] if layout == 'split' else [view.state.preview_stream])


def test_missing_primary_track_still_draws_secondary(ui_context):
    view = SimView(NS(snapshot=NS(last_error='', connected_streams=('hand_eye_preview',))))
    drawn = []
    view._draw_stream = lambda name, **kwargs: (drawn.append(name), None)
    ui_context.display_size = 1000, 700
    imgui.new_frame()
    imgui.begin('video')
    view.draw()
    imgui.end()
    imgui.render()
    assert drawn == ['observer', 'hand_eye_preview']


def test_inset_does_not_forward_pointer_gestures_to_observer(monkeypatch, ui_context):
    frame = np.zeros((600, 800, 3), dtype=np.uint8)
    view = SimView(NS(frame=lambda _: frame))
    gestures = []
    monkeypatch.setattr(sim_view, 'GL', object())
    view._upload = lambda *args, **kwargs: 1
    view._handle_observer_input = lambda **kwargs: gestures.append(kwargs)
    ui_context.display_size = 1000, 700

    def draw():
        imgui.new_frame()
        imgui.set_next_window_position(0, 0, imgui.ALWAYS)
        imgui.set_next_window_size(900, 650, imgui.ALWAYS)
        imgui.begin('video')
        _, rect = view._draw_stream('observer', width=600, max_height=450, connected=True,
                                    pip_stream='hand_eye_preview')
        imgui.end()
        imgui.render()
        return rect

    draw()
    rect = draw()
    ui_context.mouse_pos = rect[0]+10, rect[1]+10
    draw()
    assert gestures
    gestures.clear()
    x0, y0, x1, y1 = _pip_rect(rect, source_aspect=4/3)
    ui_context.mouse_pos = (x0+x1)*.5, (y0+y1)*.5
    draw()
    assert not gestures
