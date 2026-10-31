"""Operator controls for the Pilot-owned Sim scan session."""
import imgui

from elesim_ui.helpers import panel_header


def draw_object_scan_panel(panel):
    status = getattr(panel.service, 'object_scan', {})
    if not status or not panel_header('Object Scan (Sim)', visible=True)[0]:
        return
    imgui.text('Phase: '+str(status.get('phase', 'idle')))
    imgui.text_wrapped('Keep object and base stationary. Aim the hand-eye camera at the target first.')
    active = bool(status.get('active'))
    if not active:
        _, target = imgui.input_text('Scan instance ID', getattr(panel, '_scan_target', 'target'), 65)
        panel._scan_target = target
        _, views = imgui.slider_int('Maximum views', getattr(panel, '_scan_views', 12), 2, 24)
        panel._scan_views = views
        if status.get('can_start'):
            if imgui.button('Start object scan'):
                _submit(panel, 'start_object_scan', target, views)
        elif status.get('blocked_reason'):
            imgui.text_wrapped(str(status['blocked_reason']))
    elif imgui.button('Cancel object scan'):
        _submit(panel, 'cancel_object_scan')
    if status.get('stage'):
        imgui.text('Stage: '+str(status['stage']))
    imgui.text(f"Accepted views: {status.get('accepted_views', 0)} | Observed points: {status.get('observed_points', 0)}")
    imgui.text(f"Predicted points: {status.get('predicted_points', 0)}")
    coverage = status.get('coverage', {})
    if coverage:
        imgui.text(f"View-direction coverage: {100*coverage.get('fraction', 0):.1f}%")
        imgui.text(f"Unobserved direction probes: {len(coverage.get('unobserved_directions', []))}")
        counts = {name: 0 for name in ('+X', '-X', '+Y', '-Y', '+Z', '-Z')}
        for vector in coverage.get('unobserved_directions', []):
            axis = max(range(3), key=lambda i: abs(vector[i]))
            counts[('+' if vector[axis] >= 0 else '-')+'XYZ'[axis]] += 1
        imgui.text('World directions: '+' '.join(f'{key}:{value}' for key, value in counts.items()))
        imgui.text_wrapped('View diversity is a proxy. Hidden surface completeness is unverified.')
    for name in ('stop_reason', 'reason', 'scene_manifest'):
        if status.get(name):
            imgui.text_wrapped(name+': '+str(status[name]))
    view = getattr(panel, '_sim_view', None)
    session = getattr(view, 'session', None)
    if status.get('phase') == 'completed' and status.get('scene_reference') and session is not None:
        if session.supports_scene_rebuild and not session.snapshot.pending_commands:
            if imgui.button('Apply reconstructed scene'):
                try:
                    request_id = session.load_scan_result(status)
                    panel._scan_scene_request_id = request_id
                    panel._scan_error = '' if request_id else session.snapshot.last_error
                except ValueError as error:
                    panel._scan_error = str(error)
        result = session.snapshot.last_result
        if result is not None and result.request_id == getattr(panel, '_scan_scene_request_id', ''):
            imgui.text_wrapped(result.reason)
        elif getattr(panel, '_scan_scene_request_id', '') and session.snapshot.pending_commands:
            imgui.text_disabled('Applying reconstructed scene...')
    if getattr(panel, '_scan_error', ''):
        imgui.text_wrapped(panel._scan_error)


def _submit(panel, name, *args):
    panel._scan_error = ''
    panel.service.object_scan_async(name, *args, on_result=lambda value: None,
        on_error=lambda value: setattr(panel, '_scan_error', str(value)))
