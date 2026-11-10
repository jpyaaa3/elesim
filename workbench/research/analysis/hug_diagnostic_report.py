"""Dependency-free offline SVG/HTML viewer for measured hug diagnostics."""
from __future__ import annotations

import argparse
import hashlib
from html import escape
import json
import math
from pathlib import Path


def arc(start, heading, turn, length, count=100):
    points = []
    for index in range(count+1):
        t = index/count
        if abs(turn) < 1e-10:
            dx,dy = length*t*math.cos(heading),length*t*math.sin(heading)
        else:
            dx = length/turn*(math.sin(heading+turn*t)-math.sin(heading))
            dy = length/turn*(math.cos(heading)-math.cos(heading+turn*t))
        points.append([start[0]+dx,start[1]+dy])
    return points


def plot_data(vertices, pose, settings):
    t1,t2 = pose['turns'];length = pose['length_m']
    first = arc([0,0],math.pi/2,t1,length)
    second = arc(first[-1],math.pi/2+t1,t2,length)
    heading = math.pi/2+t1+t2
    tip = [second[-1][i]+pose['terminal_ratio']*length*f
           for i,f in enumerate((math.cos(heading),math.sin(heading)))]
    co,si = math.cos(pose['rotation']),math.sin(pose['rotation'])
    def transform(p):
        return [co*p[0]-si*p[1]+pose['translation'][0],
                si*p[0]+co*p[1]+pose['translation'][1]]
    half = settings['body_width_m']/2
    body = [[-half,0],[half,0],[half,-settings['body_length_m']],[-half,-settings['body_length_m']]]
    return dict(target=list(map(transform,vertices)),first=first,second=second,
        stick=[second[-1],tip],body=body,contacts=list(map(transform,pose['contact_points_local'])))


def best_summary(row):
    fixed = row['fixed'];best = next(iter(fixed['candidates']),None)
    return dict(case=row['case'],terminal_ratio=row['terminal_ratio'],
        terminal_mm=1000*row['terminal_ratio']*row['section_length_m'],
        candidates=fixed['candidate_count'],legacy_gate_pass=fixed['legacy_gate_pass_count'],
        forward_rejected=fixed['forward_rejected_count'],
        endpoint_non_tangent=fixed['endpoint_non_tangent_count'],solves=fixed['solves'],
        search_s=fixed['elapsed_s'],status=fixed['status'],
        best_headroom_deg=best['metrics']['section_angle_margins']['minimum_deg'] if best else None,
        best_centroid_margin_mm=1000*best['metrics']['centroid_margin_m'] if best else None,
        sampled_recovery=best['local_recovery']['successes'] if best else None,
        best_candidate=(dict(turns_deg=list(map(math.degrees,best['turns'])),
            first_contact_a=best['a'],second_contact_mode=best['mode'],
            metrics=best['metrics'],legacy_gate=best['legacy_gate'],
            contact_proxies=best['contact_proxies'],local_recovery=best['local_recovery']) if best else None),
        legacy_variable_lengths_mm=[1000*s['length_m'] for s in row['legacy']['solutions']],
        hold_verified=None,motor_load_margin=None,compression_margin=None)


def svg_panel(data, x, y, width, height, title, subtitle,
              missing_message='Not found in bounded search'):
    scale = min((width-24)/.76,(height-65)/.72)
    def point(p):
        return f'{x+width/2+p[0]*scale:.2f},{y+55+(.55-p[1])*scale:.2f}'
    def shape(points,color,fill='none',dash=''):
        return f'<polyline points="{" ".join(map(point,points))}" stroke="{color}" fill="{fill}" stroke-width="2.2" {dash}/>'
    result = [f'<rect x="{x+2}" y="{y+2}" width="{width-4}" height="{height-4}" rx="9" fill="#fff" stroke="#ccd6e0"/>',
        f'<text x="{x+14}" y="{y+23}" font-size="15" font-weight="600">{escape(title)}</text>',
        f'<text x="{x+14}" y="{y+43}" font-size="11" fill="#526173">{escape(subtitle)}</text>']
    if data is None:
        result.append(f'<text x="{x+14}" y="{y+height/2}" font-size="14" fill="#526173">{escape(missing_message)}</text>')
    else:
        result += [shape(data['target']+[data['target'][0]],'#7a8798','#e1e7ed'),
            shape(data['body']+[data['body'][0]],'#58677a','#c9d4df'),
            shape(data['contacts'],'#8a5cc5',dash='stroke-dasharray="5 4"'),
            shape(data['first'],'#1876b9'),shape(data['second'],'#db7019'),shape(data['stick'],'#c42643')]
        for p in data['contacts']:
            cx,cy = point(p).split(',')
            result.append(f'<circle cx="{cx}" cy="{cy}" r="3" fill="#202b38"/>')
    return ''.join(result)


def comparison_svg(report):
    rows = report['rows'];names = list(dict.fromkeys(row['case'] for row in rows))
    panels = []
    for i,name in enumerate(names):
        row = next(r for r in rows if r['case']==name)
        for j,label in enumerate(('old exact gate','diagnostic geometry')):
            choices = [p for p in row['fixed']['candidates'] if j or p['legacy_gate']['pass_']]
            best = next(iter(choices),None)
            data = plot_data(row['target_vertices'],best,report['settings']) if best else None
            summary = best_summary(row)
            subtitle = f'{len(choices)} sampled candidates'
            if best:
                angle = best['metrics']['section_angle_margins']['minimum_deg']
                subtitle += f' | angle headroom {angle:.1f} deg'
            panels.append(svg_panel(data,j*440,105+i*340,440,340,
                f'{name} | {label}',subtitle,missing_message=(
                    'No sampled candidate passes old exact' if j==0 else 'Not found in bounded search')))
    height = 125+len(names)*340
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="880" height="{height}" viewBox="0 0 880 {height}">
<rect width="880" height="{height}" fill="#eef3f8"/>
<g font-family="DejaVu Sans,sans-serif" fill="#172839">
<text x="16" y="29" font-size="22" font-weight="600">Hug: attached gripper and 250 mm sections fixed</text>
<text x="16" y="53" font-size="13">Blue / orange: sections | red: terminal | dashed purple: virtual contact chord</text>
<text x="16" y="75" font-size="13">Bounded geometry only. Holding, actuator load and compression: unverified.</text>
{''.join(panels)}</g></svg>'''


PAGE = r'''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Hug 판정 진단: 그리퍼 고정</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#eef3f8;color:#182a3c;font:15px/1.6 system-ui,sans-serif}
main{max-width:1280px;margin:auto;padding:24px}h1{font-size:26px;margin:0}h2{font-size:19px}
.card{background:white;border:1px solid #ced8e2;border-radius:12px;padding:20px;margin:18px 0}
.muted{color:#536579}.grid{display:grid;grid-template-columns:minmax(300px,1.2fr) minmax(290px,1fr);gap:24px}
label{display:inline-block;margin:0 16px 12px 0}select{padding:7px;border:1px solid #b3c2d1;border-radius:5px;max-width:240px}
svg{width:100%;background:#f8fafc;border-radius:8px}dt{font-weight:600}dd{margin:0 0 10px}
table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:8px 10px;border-bottom:1px solid #dce4ec;text-align:left;white-space:nowrap}
th{background:#f3f6fa}.scroll{overflow:auto}@media(max-width:780px){.grid{grid-template-columns:1fr}main{padding:12px}}
</style><main><h1>현재 그리퍼를 고정하고 hug 판정을 검토한다</h1>
<p>두 구간을 각각 <strong>250 mm</strong>로 고정하고 기존 그리퍼를 부착한 평면 기하 진단.
그리퍼 길이·유무 조합을 탐색하지 않는다. 후보 수는 발견한 수이며 작업 공간의 부피나 파지 성공률이 아니다.</p>
<p class="muted">그리퍼는 hug.py의 고정 직선 근사다. 실제 CAD/jaw 접촉이나 ID5 집기를 재현하지 않는다.</p>
<div class="card"><strong>판정 경계</strong><p>접촉·비관통·몸통 간섭·가상 포획 영역을 검사했다.
보라색 점선은 실제 부재가 없는 가상 닫힘선이다. 각도 여유는 기하학적 굽힘 한계까지의 거리다.
힘·마찰·물컹한 물체의 압축·실제 ID3/ID4 부하와 유지 성공은 <strong>미검증</strong>이다.</p>
<p class="muted">미발견은 정해진 탐색에서 못 찾았다는 뜻이다. 기구적으로 불가능하다는 증명이 아니다.
구판 가변 길이 결과와 고정 길이 결과를 별도로 표시하며 runtime 판정·제어는 변경하지 않았다.</p></div>
<div class="card"><label>형상 <select id="case"></select></label>
<label>후보 <select id="candidate"></select></label><div class="grid"><div>
<svg id="plot" viewBox="0 0 520 500" role="img" aria-label="팔과 대상의 평면 배치"></svg>
<p class="muted">파랑: 구간 1 · 주황: 구간 2 · 빨강: 꼬다리 · 회색: 대상/몸통</p></div><div id="metrics"></div></div></div>
<div class="card"><h2>같은 예산의 탐색 결과</h2><div class="scroll"><table id="table"></table></div>
<p class="muted">±1 mm 위치 / ±1° 회전의 여섯 오차에 두 굽힘을 ±5° 이내로 보정한 국소 수치 탐색이다.
팔의 접촉 재료 좌표는 고정했다. 초기 오차부터 최종 자세까지의 연속 비관통 경로·회수 동작은 검증하지 않았다.</p></div>
<div class="card"><h2>재현 조건</h2><pre id="settings" style="white-space:pre-wrap"></pre></div>
</main><script id="data" type="application/json">__DATA__</script><script>
const D=JSON.parse(document.querySelector('#data').textContent),$=s=>document.querySelector(s);
const fmt=(v,n=2)=>v==null?'미검증':Number(v).toFixed(n);
const esc=v=>String(v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const cases=[...new Set(D.rows.map(r=>r.case))].sort((a,b)=>
 Number(D.rows.find(r=>r.case===b).fixed.candidate_count>0)-Number(D.rows.find(r=>r.case===a).fixed.candidate_count>0));
$('#case').innerHTML=cases.map(c=>'<option>'+esc(c)+'</option>').join('');
function populate(){populateCandidates()}
function selectedRow(){return D.rows.find(r=>r.case==$('#case').value)}
function populateCandidates(){let r=selectedRow();$('#candidate').innerHTML=r.fixed.candidates.map((c,i)=>'<option value="'+i+'">'+(i+1)+': '+c.search_modes.join('/')+' · 여유 '+fmt(c.metrics.section_angle_margins.minimum_deg,1)+'°</option>').join('')||'<option>미발견</option>';render()}
function draw(data){const x=p=>260+p[0]*590,y=p=>350-p[1]*590;
 const path=(ps,color,fill='none',dash='')=>'<polyline points="'+ps.map(p=>[x(p),y(p)].join(',')).join(' ')+'" fill="'+fill+'" stroke="'+color+'" stroke-width="3" '+dash+'/>';
 if(!data)return '<text x="30" y="220" fill="#536579">탐색 예산 안에서 후보 미발견</text>';
 return path([...data.target,data.target[0]],'#7a8798','#e1e7ed')+path([...data.body,data.body[0]],'#58677a','#c9d4df')+
 path(data.contacts,'#8a5cc5','none','stroke-dasharray="7 5"')+path(data.first,'#1876b9')+path(data.second,'#db7019')+path(data.stick,'#c42643')+
 data.contacts.map((p,i)=>'<circle cx="'+x(p)+'" cy="'+y(p)+'" r="5" fill="#182a3c"/><text x="'+(x(p)+8)+'" y="'+(y(p)-8)+'" font-size="14">P'+(i+1)+'</text>').join('')}
function render(){const r=selectedRow(),p=r.fixed.candidates[+$('#candidate').value];$('#plot').innerHTML=draw(p?.plot);
 if(!p){$('#metrics').innerHTML='<p>후보 미발견 · '+r.fixed.solves+'회 수치 탐색 / '+fmt(r.fixed.elapsed_s)+'초.</p><p>불가능 판정이 아니다.</p>';return}
 const m=p.metrics,rec=p.local_recovery,pairs=[['기하 후보',r.fixed.candidate_count+'개 · 구판 exact 통과 '+r.fixed.legacy_gate_pass_count+'개'],
 ['굽힘 각도',p.turns.map(x=>fmt(x*180/Math.PI,1)+'°').join(' / ')],['최소 각도 여유',fmt(m.section_angle_margins.minimum_deg,1)+'°'],
 ['면적 중심 여유',fmt(m.centroid_margin_m*1000)+' mm · 실제 질량 중심 미측정'],['몸통-대상 거리',fmt(m.body_target_clearance_m*1000)+' mm'],
 ['구판 P2 전방 조건',(m.legacy_forward_pass?'통과':'탈락')+' · 투영 '+fmt(m.forward_projection_m*1000)+' mm'],
 ['구판 exact 판정',p.legacy_gate.pass_?'통과':p.legacy_gate.violations.join(', ')],
 ['국소 보정',rec?rec.successes+'/'+rec.trials+' · 물리적 회복 경로 증명 아님':'이 후보는 계산하지 않음'],
 ['힘 접촉 가정',p.contact_proxies?p.contact_proxies.by_friction.map(f=>'μ='+f.mu+': 평면 force closure '+(f.closure?'성립':'미성립')+', 정규화 +Y 지지 '+fmt(f.task_support_per_assumed_normal_unit)).join('<br>'):'이 후보는 계산하지 않음'],
 ['실제 부하 / 압축 / 유지','미검증 · ID3/ID4 동역학과 재료 보정 필요']];
 $('#metrics').innerHTML='<dl>'+pairs.map(kv=>'<dt>'+kv[0]+'</dt><dd>'+kv[1]+'</dd>').join('')+'</dl>'}
$('#case').onchange=populate;$('#candidate').onchange=render;
$('#table').innerHTML='<tr>'+['형상','꼬다리 mm','후보','구판 exact','P2 전방 탈락','끝점 비접선','각도 여유°','보정/6','구판 L mm'].map(s=>'<th>'+s+'</th>').join('')+'</tr>'+
 D.summaries.map(s=>'<tr>'+[s.case,fmt(s.terminal_mm),s.candidates,s.legacy_gate_pass,s.forward_rejected,s.endpoint_non_tangent,
 s.best_headroom_deg==null?'미발견':fmt(s.best_headroom_deg,1),s.sampled_recovery??'—',s.legacy_variable_lengths_mm.map(x=>fmt(x,1)).join(', ')||'미발견'].map(v=>'<td>'+esc(v)+'</td>').join('')+'</tr>').join('');
$('#settings').textContent=JSON.stringify({settings:D.settings,gripper:D.gripper,reference:D.reference,report_sha256:D.report_sha256,limitations:D.limitations},null,2);populate();
</script></html>'''


def build_report(report, report_sha256):
    if report.get('schema_version') != 1 or not report.get('complete') or not report.get('rows'):
        raise ValueError('a complete nonempty schema-1 diagnostic is required')
    gripper = report.get('gripper',{})
    ratios = {row['terminal_ratio'] for row in report['rows']}
    if (gripper.get('fixed') is not True or gripper.get('present') is not True
            or ratios != {gripper.get('ratio')} or gripper['ratio'] <= 0):
        raise ValueError('the existing attached gripper must remain fixed; length sweeps are not this report')
    data = dict(report,report_sha256=report_sha256,
        summaries=[best_summary(row) for row in report['rows']])
    data = json.loads(json.dumps(data,allow_nan=False))
    for row in data['rows']:
        for pose in row['fixed']['candidates']:
            pose['plot'] = plot_data(row['target_vertices'],pose,data['settings'])
    witness = data.get('analytic_witness')
    if witness:
        witness['pose']['plot'] = plot_data(witness['target_vertices'],witness['pose'],data['settings'])
    encoded = json.dumps(data,ensure_ascii=False,allow_nan=False).replace('<','\\u003c')
    summary = {k:v for k,v in data.items() if k not in ('rows','analytic_witness')}
    if witness:
        summary['analytic_witness'] = {k:v for k,v in witness['pose'].items() if k not in
            ('plot','contact_points_local','inward_normals_local')}
    return PAGE.replace('__DATA__',encoded),comparison_svg(data),summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    raw = args.input.read_bytes();report = json.loads(raw)
    page,svg,summary = build_report(report,hashlib.sha256(raw).hexdigest())
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'viewer.html').write_text(page)
    (args.output/'comparison.svg').write_text(svg)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),rows=len(report['rows']))))


if __name__ == '__main__':
    main()
