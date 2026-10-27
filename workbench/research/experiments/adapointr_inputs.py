"""Prepare individual observed objects from sensor/prediction records, no truth."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def prepare(sensors,predictions,output,*,max_captures=12):
    sensors,predictions,output=map(Path,(sensors,predictions,output))
    output.mkdir(parents=True,exist_ok=False)
    recording=json.loads((predictions/'result.json').read_text())
    inputs=[];captures=[]
    for frame in recording['frames']:
        if not frame['name'].endswith('-2.npz'):continue
        if len(captures)>=max_captures:break
        captures.append({'capture':frame['name'],'floor_valid':frame['floor_valid'],'objects':0})
        if not frame['floor_valid']:continue
        with np.load(sensors/frame['name'],allow_pickle=False) as packet:
            color=packet['color_bgr']
        hsv=cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
        mask=(((hsv[:,:,0]<=12)|(hsv[:,:,0]>=168))&(hsv[:,:,1]>=100)&(hsv[:,:,2]>=60)).astype(np.uint8)
        _,labels=cv2.connectedComponents(mask,connectivity=8)
        with np.load(predictions/frame['name'],allow_pickle=False) as packet:
            points=packet['points_after'];pixels=packet['pixels_uv']
        if len(points)!=len(pixels):raise ValueError('incoherent corrected cloud')
        point_labels=labels[pixels[:,1],pixels[:,0]]
        for component in sorted(set(point_labels)-{0}):
            observed=points[point_labels==component]
            if len(observed)<128:continue
            name=Path(frame['name']).stem+f'-component-{component:02d}'
            path=output/(name+'.npy');np.save(path,observed,allow_pickle=False)
            inputs.append({'id':name,'file':path.name,'capture':frame['name'],
                           'component':int(component),'observed_points':len(observed),
                           'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
            captures[-1]['objects']+=1
    result={'schema_version':1,'inputs':inputs,'captures':captures,
            'source':'sensor RGB red connected components and nominal-FK/plane-corrected depth points',
            'selection':'first capture IDs; all third stability frames count; >=128 observed points per component',
            'truth_access':False,'object_catalog_access':False,'alignment':'none'}
    (output/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sensors',type=Path);parser.add_argument('predictions',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--captures',type=int,default=12)
    args=parser.parse_args();result=prepare(args.sensors,args.predictions,args.output,max_captures=args.captures)
    print(json.dumps({'captures':len(result['captures']),'objects':len(result['inputs'])}))


if __name__=='__main__':main()
