"""Fit compact implicit band functions to a circle Hugmap (NumPy/SciPy only).

The labels describe recorded solver successes, not certified feasibility.
Diameter holdouts are used to select a family; separate diameters audit it.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial.distance import cdist


def features(x, model):
    z = (x - model['origin']) / model['scale']
    if model['kind'] == 'polynomial':
        return np.prod(z[:, None, :] ** np.asarray(model['powers'])[None, :, :], axis=2)
    return np.column_stack((np.ones(len(z)), np.exp(-.5 * cdist(z, model['centers'], 'sqeuclidean'))))


def predict(x, model):
    weights = np.asarray(model['weights'])
    return np.concatenate([features(chunk, model) @ weights for chunk in np.array_split(x, max(1, (len(x)+4095)//4096))])


def metrics(y, p):
    out=[]
    for k in range(2):
        tp=int(np.sum(y[:,k] & p[:,k]));fp=int(np.sum(~y[:,k] & p[:,k]));fn=int(np.sum(y[:,k] & ~p[:,k]))
        out.append(dict(tp=tp,fp=fp,fn=fn,iou=tp/max(1,tp+fp+fn),precision=tp/max(1,tp+fp),recall=tp/max(1,tp+fn)))
    return out


def centers_farthest(z, count):
    # Deterministic space-covering centers, independent of target value fitting.
    chosen=[];distance=np.full(len(z),np.inf);index=0
    for _ in range(count):
        chosen.append(z[index]);distance=np.minimum(distance,np.sum((z-z[index])**2,axis=1));index=int(np.argmax(distance))
    return np.array(chosen)


def fit(x, target, spec, rng):
    near = np.max(target, axis=1) > -.999
    indices=np.concatenate((np.flatnonzero(near),rng.choice(np.flatnonzero(~near),min(3000,np.sum(~near)),replace=False)))
    if len(indices)>24000:indices=rng.choice(indices,24000,replace=False)
    xx=x[indices];yy=target[indices]
    model=dict(spec)
    if model['kind']=='rbf':
        z=(x[near]-model['origin'])/model['scale']
        if len(z)>14000:z=z[rng.choice(len(z),14000,replace=False)]
        model['centers']=centers_farthest(z,model.pop('count')).tolist()
    a=features(xx,model)
    # Two distance fields, with emphasis on their respective near-boundary samples.
    weights=[]
    for k in range(2):
        w=np.where(yy[:,k]>-.999,4.,1.)
        gram=a.T@(a*w[:,None]);penalty=1e-3 if model['kind']=='rbf' else 1e-4
        weights.append(np.linalg.solve(gram+penalty*np.eye(a.shape[1]),a.T@(yy[:,k]*w)))
    model['weights']=np.array(weights).T.tolist()
    return model


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--embed-reader',type=Path);args=ap.parse_args()
    raw=args.source.read_bytes();doc=json.loads(raw)
    if doc.get('summary',{}).get('status')!='complete':raise ValueError('A completed table is required')
    cfg=doc['metadata']['config']
    if cfg['dx_mm']!=5 or cfg['dy_mm']!=5 or doc['metadata'].get('diameters_mm')!=list(range(100,201,5)):
        raise ValueError('This experiment requires a 5 mm grid and diameters 100..200 in 5 mm steps')
    if doc['metadata']['constraints'].get('interest_region')!='left_forward_quadrant':raise ValueError('Expected original left-quadrant coordinates')
    xs=[];ys=[];targets=[]
    for m in doc['maps']:
        rows=m['results'];grid=np.zeros((101,101,2),bool)
        for r in rows:
            if r['status']=='found':grid[round(-r['x_mm']/5),round(r['y_mm']/5),0 if r['seg2_deg']>0 else 1]=True
        fields=np.stack([np.clip((distance_transform_edt(grid[:,:,k])-distance_transform_edt(~grid[:,:,k]))*5/20,-1,1) for k in range(2)],axis=-1)
        for r in rows:
            ix=round(-r['x_mm']/5);iy=round(r['y_mm']/5)
            xs.append([r['x_mm'],r['y_mm'],m['diameter_mm']]);ys.append(grid[ix,iy]);targets.append(fields[ix,iy])
    x=np.array(xs);y=np.array(ys);target=np.array(targets)
    train=(x[:,2]%10)==0;validation=(x[:,2]%20)==5;test=(x[:,2]%20)==15
    specs=[]
    for degree in [2,4,6]:
        powers=[[i,j,k] for i in range(degree+1) for j in range(degree+1-i) for k in range(degree+1-i-j)]
        specs.append(dict(kind='polynomial',degree=degree,origin=[-250,250,150],scale=[250,250,50],powers=powers))
    for n,width in [(128,25),(256,20),(512,15),(768,12),(1024,10),(1536,8)]:
        specs.append(dict(kind='rbf',count=n,origin=[0,0,150],scale=[width,width,15]))
    trials=[];models=[]
    for spec in specs:
        model=fit(x[train],target[train],spec,np.random.default_rng(831))
        pv=predict(x[validation],model);threshold=[]
        for k in range(2):
            ts=np.linspace(-.3,.3,61);scores=[metrics(y[validation,k:k+1].repeat(2,axis=1),(pv[:,k:k+1]>t).repeat(2,axis=1))[0]['iou'] for t in ts]
            threshold.append(float(ts[np.argmax(scores)]))
        model['threshold']=threshold
        val=metrics(y[validation],pv>threshold)
        record=dict(spec=spec,validation=val,mean_iou=float(np.mean([v['iou'] for v in val])))
        print(json.dumps(record),flush=True);trials.append(record);models.append(model)
    best=int(np.argmax([r['mean_iou'] for r in trials]));selected=models[best]
    audit=metrics(y[test],predict(x[test],selected)>selected['threshold'])
    final=fit(x,target,specs[best],np.random.default_rng(831));final['threshold']=selected['threshold']
    reconstruction=metrics(y,predict(x,final)>final['threshold'])
    report=dict(schema='hugmap-band-fit',version=1,source_sha256=hashlib.sha256(raw).hexdigest(),source_name=args.source.name,
        config=doc['metadata']['config'],geometry=doc['metadata']['geometry'],domain=dict(x_mm=[-500,0],y_mm=[0,500],diameter_mm=[100,200],radius_mm=500),
        bands=['Band 1: seg2 > 0','Band 2: seg2 < 0'],formula='f_k(x,y,D) = w_0k + sum_j w_jk exp(-0.5 sum_i ((v_i-o_i)/s_i-c_ji)^2) - threshold_k; band iff f_k >= 0',
        train_diameters=sorted(set(x[train,2])),validation_diameters=sorted(set(x[validation,2])),test_diameters=sorted(set(x[test,2])),
        trials=trials,selected_trial=best,heldout_test=audit,reconstruction=reconstruction,model=final,
        semantics='Empirical recorded-success regions, not a grasp solver or proof that unobserved positions are impossible. No extrapolation.')
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,separators=(',',':'))+'\n')
    if args.embed_reader:
        html=args.embed_reader.read_text()
        payload=json.dumps(report,separators=(',',':')).replace('<','\\u003c')
        html,count=re.subn(r'(<script id="bandFitModel" type="application/json">).*?(</script>)',lambda m:m[1]+payload+m[2],html,flags=re.S)
        if count!=1:raise ValueError('Expected one embedded band-fit model slot')
        args.embed_reader.write_text(html)
    print('SELECTED',best,'AUDIT',audit,'RECONSTRUCTION',reconstruction,flush=True)

if __name__=='__main__':main()
