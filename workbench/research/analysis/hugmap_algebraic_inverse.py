"""Finite trigonometric-polynomial inverse for fixed contact features.

First contact on seg1 determines q1 independently; second determines q2.
The tan-half-angle substitution gives polynomials of degree at most ten.
Roots are numerical, not a radicals-only closed-form or certified root isolation.
"""
import argparse
from functools import lru_cache
from collections import Counter
import json
import hashlib
import math
from pathlib import Path
import time

import numpy as np
from numpy.polynomial import polynomial as poly
from workbench.research.analysis import hugmap_contact_geometry as geom
from workbench.research.support import hugmap as h


def residual(q, center, radius, spec):
    f=geom.feature(q,*spec)
    delta=center-f['point']
    return float(delta@delta-radius**2) if f['kind']=='v' else float(f['n']@delta-radius)


def tan_polynomial(function, harmonics):
    """Exact finite Fourier interpolation in exact arithmetic (not data fitting).

    function(a) has frequencies -m..m. Evaluate it at 2m+1 fixed angles;
    substitute exp(i*a)=(1+i*u)/(1-i*u) and clear (1+u*u)**m.
    """
    m=harmonics;n=2*m+1
    values=np.array([function(2*np.pi*j/n) for j in range(n)])
    fourier=np.fft.fft(values)/n
    coefficients=np.zeros(2*m+1,complex)
    for k in range(-m,m+1):
        term=poly.polymul(poly.polypow([1,1j],m+k),poly.polypow([1,-1j],m-k))
        coefficients[:len(term)]+=fourier[k%n]*term
    if np.max(np.abs(coefficients.imag))>1e-8:raise ArithmeticError('Non-real polynomial')
    return coefficients.real


def polynomial_angles(coefficients):
    scale=float(np.max(np.abs(coefficients)))
    if scale<1e-13:
        raise ArithmeticError('Degenerate contact equation: continuous family')
    normalized=coefficients/scale
    while len(normalized)>1 and abs(normalized[-1])<1e-12:normalized=normalized[:-1]
    candidates=[]
    for u in poly.polyroots(normalized):
        if abs(u.imag)>1e-7:continue
        angle=10*math.atan(float(u.real))
        if abs(angle)>np.pi+1e-8:continue
        angle=float(np.clip(angle,-np.pi,np.pi))
        if not any(abs(angle-other)<1e-7 for other in candidates):candidates.append(angle)
    return sorted(candidates)


def angle_roots(function,harmonics):
    return [q for q in polynomial_angles(tan_polynomial(function,harmonics))
            if abs(function(q/5))<=1e-8]


@lru_cache(maxsize=None)
def feature_polynomials(spec):
    part=h.PART_NAMES.index(spec[0]);first=part<5
    m=part+1 if first else min(part-4,5)
    def coefficients(center):
        return tan_polynomial(lambda a:residual([5*a,0] if first else [0,5*a],center,0,spec),m)
    constant=coefficients(np.zeros(2));denominator=poly.polypow([1.,0.,1.],m)
    xterm=coefficients(np.array([1.,0.]))-constant
    yterm=coefficients(np.array([0.,1.]))-constant
    if spec[1]=='v':xterm-=denominator;yterm-=denominator
    return constant,xterm,yterm,denominator


def feature_angles(center,radius,spec):
    constant,xterm,yterm,denominator=feature_polynomials(tuple(spec));x,y=center
    term=x*x+y*y-radius*radius if spec[1]=='v' else -radius
    return polynomial_angles(constant+x*xterm+y*yterm+term*denominator)


def second_center(q1,center):
    # Express the target in the second-segment frame; reuse q1=0 coefficients.
    return np.array([0.,.25])+geom.rotate(center-geom.chain_sum(5,q1/5),-q1)


def inverse_candidates(center,radius,specs):
    first=h.PART_NAMES.index(specs[0][0]);second=h.PART_NAMES.index(specs[1][0])
    if first>=5 or second<5:raise ValueError('Expected seg1 first, seg2/gripper second')
    q1s=feature_angles(center,radius,specs[0])
    out=[]
    for q1 in q1s:
        q2s=feature_angles(second_center(q1,center),radius,specs[1])
        for q2 in q2s:
            q=np.array([q1,q2])
            if geom.on_features(q,specs,center,radius):out.append(q)
    return out


def solve_features(center,radius,specs,mu_allow=.5):
    solutions=[]
    for q in inverse_candidates(center,radius,specs):
        result=h.inspect_pose(q,center,mu_allow,radius)
        if result is not None:solutions.append(result)
    return sorted(solutions,key=lambda s:s['mu_min'])


def solve_catalog(center,radius,catalog,mu_allow=.5,q2_negative_only=False):
    first_angles={};second_angles={};checked=set();solutions=[];degeneracies=[]
    for specs in catalog:
        try:
            first,second=map(tuple,specs)
            if first not in first_angles:first_angles[first]=feature_angles(center,radius,first)
            for q1 in first_angles[first]:
                key=(q1,second)
                if key not in second_angles:second_angles[key]=feature_angles(second_center(q1,center),radius,second)
                for q2 in second_angles[key]:
                    if q2_negative_only and q2>=0:continue
                    q=np.array([q1,q2])
                    if not geom.on_features(q,specs,center,radius):continue
                    pose=(round(q1,8),round(q2,8))
                    if pose in checked:continue
                    checked.add(pose)
                    result=h.inspect_pose(q,center,mu_allow,radius)
                    if result is not None:solutions.append(result)
        except ArithmeticError as exc:degeneracies.append(dict(features=specs,error=str(exc)))
    return dict(solutions=sorted(solutions,key=lambda s:s['mu_min']),degeneracies=degeneracies)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    raw=args.source.read_bytes();doc=json.loads(raw)
    mu_allow=doc['metadata']['config']['mu_allow']
    if doc['metadata']['geometry']!=h.REFERENCE:raise ValueError('Geometry mismatch')
    # Stored angles are used ONLY to identify the contact features and audit errors.
    # The inverse itself receives center, radius and feature identifiers, no angles.
    groups={}
    for m in doc['maps']:
        for r in m['results']:
            if r['status']!='found':continue
            q=np.radians([r['seg1_deg'],r['seg2_deg']])
            specs=tuple(geom.identify(q,n,np.array(p)) for n,p in zip(r['contact_parts'],r['contacts_node0_m']))
            key=(1 if r['seg2_deg']>0 else 2,specs)
            groups.setdefault(key,[]).append((m['diameter_mm'],r))
    selected=[]
    for key,rows in groups.items():
        for i in np.unique(np.linspace(0,len(rows)-1,min(12,len(rows)),dtype=int)):
            selected.append((key,rows[i]))
    report=dict(source_sha256=hashlib.sha256(raw).hexdigest(),mu_allow=mu_allow,method='fixed-feature sequential polynomial roots; first <= degree 10, second <= degree 10',
        note='Stratified sample, features identified from saved poses; no saved joint-angle seed in inverse',cases=[])
    start=time.perf_counter()
    for index,((band,specs),(diameter,row)) in enumerate(selected):
        center=h.local_center(row['x_mm'],row['y_mm']);radius=diameter/2000;t=time.perf_counter()
        try:
            qs=inverse_candidates(center,radius,specs);error=None
            valid=[h.inspect_pose(q,center,mu_allow,radius) for q in qs];valid=[v for v in valid if v is not None]
        except ArithmeticError as exc:qs=[];valid=[];error=str(exc)
        match=min((float(np.max(np.abs(np.degrees(q)-[row['seg1_deg'],row['seg2_deg']]))) for q in qs),default=None)
        report['cases'].append(dict(band=band,D=diameter,x=row['x_mm'],y=row['y_mm'],features=specs,
            candidates=len(qs),valid=len(valid),nearest_saved_angle_error_deg=match,elapsed_s=time.perf_counter()-t,error=error))
        if (index+1)%50==0:print(index+1,'/',len(selected),flush=True)
    report['summary']=dict(cases=len(selected),valid=sum(c['valid']>0 for c in report['cases']),
        candidate_recovered=sum(c['candidates']>0 for c in report['cases']),elapsed_s=time.perf_counter()-start,
        angle_error_deg=geom.percentiles([c['nearest_saved_angle_error_deg'] for c in report['cases'] if c['nearest_saved_angle_error_deg'] is not None]))
    catalog=sorted({specs for (band,specs),rows in groups.items() if any(d%10==0 for d,r in rows)})
    blind=[];rng=np.random.default_rng(906)
    for band in (1,2):
        available=[(d,r) for (k,specs),rows in groups.items() if k==band for d,r in rows if d%10==5]
        for index in rng.choice(len(available),min(30,len(available)),replace=False):
            d,r=available[index];blind.append((band,d,r))
    holes=[]
    for m in doc['maps']:
        if m['diameter_mm']%10!=5:continue
        found={(r['x_mm'],r['y_mm']) for r in m['results'] if r['status']=='found'}
        for r in m['results']:
            x,y=r['x_mm'],r['y_mm']
            if r['status']=='not_found' and (((x-5,y) in found and (x+5,y) in found) or ((x,y-5) in found and (x,y+5) in found)):
                holes.append((0,m['diameter_mm'],r))
    blind.extend(holes[i] for i in rng.choice(len(holes),min(60,len(holes)),replace=False))
    blind_results=[];start=time.perf_counter()
    for expected_band,d,r in blind:
        t=time.perf_counter();result=solve_catalog(h.local_center(r['x_mm'],r['y_mm']),d/2000,catalog,mu_allow)
        blind_results.append(dict(expected_band=expected_band,D=d,x=r['x_mm'],y=r['y_mm'],
            solutions=result['solutions'],degeneracies=result['degeneracies'],elapsed_s=time.perf_counter()-t))
    report['blind_catalog']=dict(catalog=catalog,training_diameters=list(range(100,201,10)),
        heldout_diameters=list(range(105,200,10)),cases=blind_results,elapsed_s=time.perf_counter()-start,
        found_cases=sum(c['expected_band']!=0 for c in blind_results),
        recovered_found=sum(c['expected_band']!=0 and bool(c['solutions']) for c in blind_results),
        recovered_same_band=sum(c['expected_band']!=0 and any((1 if sol['seg2_deg']>0 else 2)==c['expected_band'] for sol in c['solutions']) for c in blind_results),
        hole_cases=sum(c['expected_band']==0 for c in blind_results),
        recovered_holes=sum(c['expected_band']==0 and bool(c['solutions']) for c in blind_results))
    print('BLIND', {k:v for k,v in report['blind_catalog'].items() if k not in ('catalog','cases')},flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(report['summary'],flush=True)

if __name__=='__main__':main()
