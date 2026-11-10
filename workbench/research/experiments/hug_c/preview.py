"""Small standalone SVG review of the function-derived q solution triangle."""
from pathlib import Path


def write_preview(path, rows, vertices, selection):
    x=lambda q:80+(q+180)*3
    y=lambda q:590-(q+180)*3
    step=selection['resolution_total_deg'];mu=selection['mu_max']
    parts=['<!doctype html><meta charset="utf-8"><title>C-hug training triangle</title>',
           '<style>body{font:14px system-ui;max-width:850px;margin:20px auto}svg{width:100%;background:white}td,th{padding:5px 12px;text-align:right}</style>',
           '<h2>C-hug: three function-derived training poses</h2>',
           f'<p>D={selection["diameter_mm"]:g} mm · μ≤{mu:g} · coverage {100*selection["coverage"]:.2f}% of the preparation-clear solution area · raster step {step:g}° total bend. Best found; continuous global optimum unproven.</p>',
           '<svg viewBox="0 0 780 680" role="img" aria-label="q3 q4 solution region and training triangle">']
    for q in range(-180,1,30):
        parts.append(f'<path d="M{x(q)} 50V590 M80 {y(q)}H620" stroke="#ddd" fill="none"/><text x="{x(q)-10}" y="615">{q}</text><text x="38" y="{y(q)+4}">{q}</text>')
    for r in rows:
        a,b=r['q_deg'];hue=240*(1-min(1,r['mu_min']/mu))
        parts.append(f'<rect x="{x(a)-step*1.5}" y="{y(b)-step*1.5}" width="{step*3}" height="{step*3}" fill="hsl({hue} 80% 50%)"/>')
    points=' '.join(f'{x(a)},{y(b)}' for a,b in vertices)
    parts.append(f'<polygon points="{points}" fill="#e0000014" stroke="#222" stroke-width="2"/>')
    for i,(a,b) in enumerate(vertices):
        parts.append(f'<circle cx="{x(a)}" cy="{y(b)}" r="5" fill="#111"/><text x="{x(a)+8}" y="{y(b)-8}">A{i+1}</text>')
    parts.append('<text x="280" y="650">q3 · seg1 total bend (°)</text><text x="80" y="25">q4 · seg2 total bend (°)</text></svg>')
    domain='final-pose CAD and the Home → Extend Arm envelope filter'
    if selection.get('static_catalog_screened'):domain+=' and actual Home/goal catalog screening'
    parts.append(f'<p>All colored cells pass {domain}. Colors: low μ blue, high μ red. The triangle may contain excluded gaps; those positions are rejected during held-out generation and XY lookup. Full roll, closing and lifting remain unverified.</p>')
    parts.append('<table><tr><th>Anchor</th><th>q3 (°)</th><th>q4 (°)</th><th>μ</th><th>Preparation margin (mm)</th></tr>')
    for i,vertex in enumerate(vertices):
        row=next(r for r in rows if list(r['q_deg'])==list(vertex))
        parts.append(f'<tr><td>A{i+1}</td><td>{vertex[0]:.2f}</td><td>{vertex[1]:.2f}</td><td>{row["mu_min"]:.4f}</td><td>{row.get("preparation_margin_mm",0):.3f}</td></tr>')
    parts.append('</table><p>PPO still starts at the real Home with four free actions. Preparation screening does not impose a timed deployment on the policy. Subsequent roll, closing and lifting remain untested.</p>')
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as stream:stream.write('\n'.join(parts))


def write_collection_preview(output,plan):
    parts=['<!doctype html><meta charset="utf-8"><title>C-hug prepared triangles</title>',
           '<style>body{font:15px system-ui;max-width:900px;margin:24px auto}td,th{padding:8px 16px;text-align:right}</style>',
           '<h2>Three C-hug anchors per diameter</h2>',
           '<p>Selection area: final-pose CAD + Home → Extend Arm envelope clear. Maximize area(S ∩ triangle); excluded interior gaps remain excluded. Each anchor starts independently from Home with four free policy actions. No RL has run for this collection.</p>',
           '<table><tr><th>Diameter</th><th>Eligible q cells</th><th>Covered area</th><th>Review</th></tr>']
    for profile in plan['profiles']:
        link=str(Path(profile['pilot']).with_name('triangle.html'))
        parts.append(f'<tr><td>{profile["diameter_mm"]:g} mm</td><td>{profile["feasible_samples"]}</td><td>{100*profile["geometric_coverage"]:.2f}%</td><td><a href="{link}">q3–q4 triangle and anchors</a></td></tr>')
    parts.append('</table><p>Geometry only. Physical Home/Extend/goal checks are required before training. Roll, closing, lift and path interpolation remain unverified.</p>')
    (Path(output)/'index.html').write_text('\n'.join(parts))
