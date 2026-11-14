"""One task-oriented surface for scene preparation, observation and action."""
from __future__ import annotations

import json

import imgui

from elesim_ui.helpers import begin_disabled_ui, end_disabled_ui, scaled, section_title
from elesim_ui.visual_servoing import STEPS


_BLUE = (.12, .36, .62, 1.)
_GREEN = (.12, .46, .28)
_AMBER = (.66, .36, .08)


def _title(value, description=''):
    section_title(value)
    if description:
        imgui.text_wrapped(description)
    imgui.spacing()


def _action(c, label, action, *, primary=False, explain=False):
    reason = c.reason(action)
    token = begin_disabled_ui(bool(reason))
    if primary:
        imgui.push_style_color(imgui.COLOR_BUTTON, *(_BLUE if not reason else (.68,.71,.75,1.)))
        imgui.push_style_color(imgui.COLOR_TEXT, 1.,1.,1.,1.)
    try:
        pressed = imgui.button(label+'##vs-'+action, -1. if primary else 0., scaled(c.panel,32) if primary else 0.)
        if pressed and not reason:
            c.perform(action)
    finally:
        if primary:
            imgui.pop_style_color(2)
        end_disabled_ui(token)
    if reason and explain:
        imgui.text_wrapped(reason)
    elif reason and imgui.is_item_hovered():
        imgui.set_tooltip(reason)


def _next(c, step, label):
    imgui.spacing()
    imgui.separator()
    if imgui.button(label+'##vs-next', -1., scaled(c.panel,30)):
        c.step = step


def _scene(c, v):
    _title('Prepare the scene', 'Use a real object with Robot, or place a virtual object in Sim.')
    imgui.text_wrapped('Control: '+(v.endpoint or 'not selected')+' / '+(v.role or 'unknown'))
    imgui.text_wrapped('Sim view: '+(getattr(c.session,'active_sim_id','') or 'not connected'))
    imgui.spacing()
    obj = v.mock_object
    available = () if obj is None else obj.available_assets
    if not available:
        imgui.text_wrapped('Virtual object catalog unavailable. Connect a Sim view to use scene preparation.')
    else:
        index = available.index(c.asset_id) if c.asset_id in available else 0
        _, index = imgui.combo('Object##vs-asset',index,list(available))
        c.asset_id = available[index]
        if 'cylinder_100mm.obj' in available and imgui.button('100 mm cylinder preset##vs-cylinder'):
            c.asset_id = 'cylinder_100mm.obj'
            c.position, c.euler_deg = [.42, .11, .55], [0., 0., 0.]
        imgui.text_wrapped('Object: '+(obj.asset_id or 'empty')+' / '+obj.state)
        if obj.attached:
            imgui.text_colored('Sim attachment active; physical hold is unverified.', *_AMBER)
        if imgui.tree_node('Placement (metres / degrees)##vs-placement'):
            try:
                for values, axes, fmt, suffix in ((c.position,'XYZ','%.3f','m'),
                        (c.euler_deg,('Roll','Pitch','Yaw'),'%.1f','deg')):
                    for i, axis in enumerate(axes):
                        changed, value = imgui.input_float(f'{axis} {suffix}##vs-placement-{suffix}-{i}',
                            values[i],0.,0.,format=fmt)
                        if changed:
                            values[i] = value
            finally:
                imgui.tree_pop()
        _action(c,'Place object','spawn',primary=True,explain=True)
        _action(c,'Remove object','remove')
        imgui.same_line()
        _action(c,'Detach','detach')
        if obj.reason:
            imgui.text_wrapped(obj.reason)
    imgui.spacing()
    imgui.text_wrapped('Next: aim the hand-eye camera at the target. Keep the object and robot base stationary during the scan.')
    _next(c,1,'Continue to observation >')


def _progress(c, v):
    s = v.scan
    stage = str(s.get('stage') or s.get('phase') or 'idle')
    phase = s.get('phase','idle')
    color = ((.74,.18,.16) if phase == 'failed' else _AMBER if phase == 'cancelled'
             else _BLUE[:3] if s.get('active') else _GREEN if phase == 'completed' else (.38,.42,.48))
    imgui.text_colored(stage.replace('_',' ').capitalize(), *color)
    total = max(1,int(s.get('views',c.max_views)))
    if s.get('active'):
        # This is a view budget, not a promised wall-clock completion percentage.
        used = int(s.get('view',0))
        imgui.progress_bar(min(1.,max(0.,used/total)),(-1.,0.),f'View budget {used}/{total}')
    imgui.text(f"Accepted views: {s.get('accepted_views',0)}")
    imgui.text(f"Observed points: {s.get('observed_points',0):,}")
    imgui.text(f"Generated points: {s.get('predicted_points',0):,}")
    coverage = s.get('coverage',{})
    if coverage:
        fraction = float(coverage.get('fraction',0.))
        imgui.text(f'View-direction coverage: {100*fraction:.1f}%')
        directions = coverage.get('unobserved_directions',[])
        imgui.text(f'Unobserved direction probes: {len(directions)}')
        if directions and imgui.tree_node('Missing viewing directions##vs-directions'):
            try:
                counts = {name:0 for name in ('+X','-X','+Y','-Y','+Z','-Z')}
                for vector in directions:
                    axis = max(range(3),key=lambda i:abs(vector[i]))
                    counts[('+' if vector[axis]>=0 else '-')+'XYZ'[axis]] += 1
                for key,count in counts.items():
                    imgui.text(f'{key}: {count}')
            finally:
                imgui.tree_pop()
        imgui.text_wrapped(v.coverage_note)
    for key in ('stop_reason','reason'):
        if s.get(key):
            imgui.text_wrapped(str(s[key]))


def _scan(c, v):
    _title('Observe & reconstruct', 'Pilot runs floor correction, segmentation, active multiview scanning, completion and surface meshing with its configured model profile.')
    imgui.text_wrapped('The instance ID names the reconstruction output. The current detector adapter selects a single object mask; this field does not select a detected instance.')
    token = begin_disabled_ui(bool(v.scan.get('active') or c.pending))
    try:
        changed, value = imgui.input_text('Instance ID##vs-target',c.target_id,65)
        if changed:
            c.target_id = value
        changed, value = imgui.slider_int('View budget##vs-views',c.max_views,2,24)
        if changed:
            c.max_views = value
    finally:
        end_disabled_ui(token)
    if v.scan.get('active') or c.pending in ('scan','cancel_scan'):
        _action(c,'Cancel scan','cancel_scan',primary=True)
    else:
        _action(c,'Start scan & reconstruction','scan',primary=True,explain=True)
    imgui.spacing()
    _progress(c,v)
    if v.reconstructed:
        _next(c,2,'Review reconstruction >')
    if v.role == 'robot':
        imgui.text_wrapped('Robot live scanning is not connected yet. This UI preserves the Pilot preflight gate.')


def _reconstruction(c, v):
    _title('Review the reconstruction', 'Measured geometry and model completion remain separate. Review the result before using it.')
    if getattr(v.mock_object, 'asset_id', '') == 'cylinder_100mm':
        imgui.text_wrapped('For reconstructed hug, keep the original physical cylinder. Continue to Action without applying a duplicate fixed scene mesh. Return the arm to Home before preparing the path.')
    if not v.reconstructed:
        imgui.text_wrapped('No completed reconstruction is available from Pilot.')
        _progress(c,v)
        _next(c,1,'Return to observation >')
    else:
        _progress(c,v)
        if v.scan.get('needs_more_observation'):
            imgui.text_colored('More observation requested', *_AMBER)
            imgui.text_wrapped('Completion adds a hypothesis; it does not verify hidden surfaces.')
        imgui.spacing()
        imgui.text_wrapped('Result source: '+str(v.scan.get('sim_id') or 'Pilot'))
        if v.scan.get('scene_reference'):
            imgui.text_wrapped('Published bundle: '+str(v.scan['scene_reference']))
        imgui.text_wrapped('Sim preview is optional presentation. Applying a mesh here does not create a grasp plan.')
        _action(c,'Apply reconstruction to Sim','apply',primary=True,explain=True)
        if c.scene_outcome == 'applied':
            imgui.text_colored('Applied: matching Sim acknowledgement received.', *_GREEN)
        _next(c,3,'Continue to action >')
    if imgui.tree_node('Result details / model execution##vs-provenance'):
        try:
            for name in ('job_id','scene_manifest','scene_revision'):
                if v.scan.get(name):
                    imgui.text_wrapped(name+': '+str(v.scan[name]))
            execution = v.scan.get('model_execution')
            if execution:
                imgui.text_wrapped(json.dumps(execution,ensure_ascii=False,indent=2))
            else:
                imgui.text_wrapped('No model execution report received. Configured profiles are owned by Pilot.')
        finally:
            imgui.tree_pop()
    if imgui.tree_node('Staged scene tools##vs-scene-tools'):
        try:
            imgui.text_wrapped('Bundle reference relative to the Sim import directory, e.g. scan-1/scene.json.')
            _, c.scene_reference = imgui.input_text('Bundle##vs-bundle',c.scene_reference,513)
            _action(c,'Load staged bundle','load_scene',explain=True)
            imgui.same_line()
            _action(c,'Clear imported scene','clear_scene')
        finally:
            imgui.tree_pop()


def _actions(c, v):
    _title('Approach, grasp & verify', 'Choose the motion strategy after reconstructing the target.')
    imgui.text('Reconstructed cylinder / interpolated C-hug')
    imgui.text_wrapped('100 mm upright cylinder, Sim only. Fit the reconstructed mesh, solve q3/q4 and blend three learned paths. Start at Home.')
    _action(c, 'Prepare reconstructed hug', 'plan_reconstructed_hug', primary=True, explain=True)
    if c.hug_plan:
        fit = c.hug_plan.get('fit', {})
        imgui.text(f"Fitted diameter: {fit.get('diameter_mm', 0):.2f} mm")
        imgui.text('Solver XY (mm): '+str(c.hug_plan.get('xy_mm', [])))
        imgui.text('q3/q4 total bend (deg): '+str(c.hug_plan.get('q_deg', [])))
        imgui.text('Blend weights: '+str(c.hug_plan.get('weights', [])))
        imgui.text_wrapped('20 s command path including final hold. Execution completion does not certify grasp success.')
    _action(c, 'Execute reconstructed hug', 'execute_reconstructed_hug', explain=True)
    imgui.separator()
    _, c.strategy = imgui.combo('Strategy##vs-strategy',c.strategy,['Gripper + visual alignment','Arm embrace'])
    action = 'gripper' if c.strategy == 0 else 'embrace'
    imgui.text_wrapped('Input: reconstructed target mesh + measured robot state.')
    imgui.text_wrapped('Execution adapter: not connected')
    _action(c,'Plan & execute grasp',action,primary=True,explain=True)
    imgui.spacing()
    imgui.text_wrapped('Hold verification: unverified. A completed scan, finished command or Sim attachment is not proof of a physical grasp.')
    imgui.separator()
    if imgui.tree_node('Simulator reference hug##vs-reference-hug'):
        try:
            imgui.text_wrapped('Input: Sim object ground truth, not the camera reconstruction. Open-space reference geometry test; hidden obstacles and physical hold are unverified.')
            _action(c,'Compute reference solution','compute_hug',explain=True)
            if c.solution:
                imgui.text_wrapped('Solution: '+str(c.solution.get('solution_id','')))
                imgui.text_wrapped('Clearance: '+str(c.solution.get('clearance_mode','unreported')))
            _action(c,'Execute reference hug','execute_hug',primary=True,explain=True)
            if getattr(v.mock_object,'attached',False):
                _action(c,'Detach reference object','detach')
        finally:
            imgui.tree_pop()
    if imgui.tree_node('Configured pole policy bench##vs-wrap-bench'):
        try:
            imgui.text_wrapped('Input: the pole geometry in Pilot configuration. This policy does not consume the reconstructed mesh. Check its trained geometry range before use.')
            _action(c,'Run configured wrap policy','wrap',primary=True,explain=True)
            result = c.service.session.service_value('wrap_grasp_result','')
            if result:
                imgui.text_wrapped(str(result))
        finally:
            imgui.tree_pop()


def draw_visual_servoing_panel(panel):
    c = panel.servoing
    c.refresh()
    v = c.view()
    imgui.text_colored('OBSERVE  /  RECONSTRUCT  /  ACT', *_BLUE[:3])
    phase = str(v.scan.get('stage') or v.scan.get('phase') or 'waiting for Pilot')
    imgui.text_wrapped(('Scan: '+phase.replace('_',' ')) if v.pilot_online else 'Pilot offline - connect to begin')
    if getattr(v.sim_status,'paused',False):
        imgui.text_colored('Sim view paused',*_AMBER)
        imgui.same_line()
        _action(c,'Resume Sim','resume_sim')
    # Stage navigation and the global workspace stop remain outside the scroller.
    available = imgui.get_content_region_available_width()
    columns = 4 if available >= scaled(panel,500) else 2
    width = max(1.,(available-imgui.get_style().item_spacing.x*(columns-1))/columns)
    for i,label in enumerate(STEPS):
        if i%columns:
            imgui.same_line()
        if columns == 4 and i == 2:
            label = 'Mesh'
        active = c.step == i
        if active:
            imgui.push_style_color(imgui.COLOR_BUTTON,*_BLUE)
            imgui.push_style_color(imgui.COLOR_TEXT,1.,1.,1.,1.)
        if imgui.button(f'{i+1}  {label}##vs-step-{i}',width,scaled(panel,30)):
            c.step = i
        if active:
            imgui.pop_style_color(2)
    imgui.separator()
    imgui.begin_child('vs-stage-'+str(c.step),0.,0.,False)
    try:
        imgui.push_item_width(max(scaled(panel,90),imgui.get_content_region_available_width()*.50))
        try:
            (_scene,_scan,_reconstruction,_actions)[c.step](c,v)
            if c.error:
                imgui.spacing()
                imgui.text_colored('Request needs attention',.72,.18,.16)
                imgui.text_wrapped(c.error)
            elif c.notice:
                imgui.spacing()
                imgui.text_wrapped(c.notice)
        finally:
            imgui.pop_item_width()
    finally:
        imgui.end_child()
