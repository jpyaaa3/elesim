"""Controlled band-1 roll/bend comparison with the identical contact catalog."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
from pathlib import Path
import time
from workbench.research.analysis import hugmap_algebraic_inverse as inv
from workbench.research.analysis import hugmap_band_functions as functions
from workbench.research.support import hugmap as h


def legacy(job):
    x,y,D=job;center=h.local_center(x,y);radius=D/2000
    if h.exclusion(center,radius):return x,y,None
    result=inv.solve_catalog(center,radius,functions.CATALOG,math.inf)
    values=[s['mu_min'] for s in result['solutions'] if s['seg2_deg']>0]
    return x,y,min(values) if values else None


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('new_report',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    raw=args.source.read_bytes();source=json.loads(raw);new=json.loads(args.new_report.read_text())
    if new['source_sha256']!=hashlib.sha256(raw).hexdigest():raise ValueError('Source mismatch')
    D=150;step=new['xy_step_mm'];entry=next(m for m in source['maps'] if m['diameter_mm']==D);new_map=next(m for m in new['maps'] if m['D']==D)
    new_values={(r[0],r[1]):r[2][0] if r[2] else None for r in new_map['values']}
    jobs=[(r['x_mm'],r['y_mm'],D) for r in entry['results'] if r['x_mm']%step==0 and r['y_mm']%step==0]
    rows=[];start=time.perf_counter()
    with ProcessPoolExecutor(max_workers=4) as pool:
        for x,y,mu in pool.map(legacy,jobs,chunksize=24):rows.append(dict(x=x,y=y,old_mu=mu,new_mu=new_values.get((x,y))))
    old=lambda r:r['old_mu'] is not None and r['old_mu']<.5
    new_ok=lambda r:r['new_mu'] is not None and r['new_mu']<.5
    report=dict(source_sha256=new['source_sha256'],D=D,grid_step_mm=step,catalog_size=len(functions.CATALOG),
        old_policy='roll +90, q2>0',new_policy='roll -90, q2<0',same='polynomial root method, 52 feature pairs, collision geometry, friction cutoff .5',
        old_count=sum(old(r) for r in rows),new_count=sum(new_ok(r) for r in rows),both=sum(old(r) and new_ok(r) for r in rows),
        new_only=sum(not old(r) and new_ok(r) for r in rows),old_only=sum(old(r) and not new_ok(r) for r in rows),
        elapsed_s=time.perf_counter()-start,rows=rows)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,separators=(',',':'))+'\n')
    print({k:v for k,v in report.items() if k!='rows'},flush=True)

if __name__=='__main__':main()
