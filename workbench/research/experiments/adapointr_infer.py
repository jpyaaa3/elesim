"""Complete recorded partial objects with strict pretrained AdaPoinTr weights."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from workbench.research.experiments.adapointr import normalize_partial,restore_completion,sample_input,pcn_axes


def main():
    import torch
    from workbench.research.support.adapointr_backend import load_model
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--vendor',type=Path,required=True);parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--checkpoint-sha256',required=True);parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--radius',type=float,default=.5)
    parser.add_argument('--axis-order',choices=['world-z-up','model-native'],default='world-z-up')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    assert torch.cuda.is_available() and torch.cuda.device_count()==1
    torch.set_num_threads(2);torch.manual_seed(17)
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    manifest=json.loads((args.inputs/'manifest.json').read_text())
    model,metadata=load_model(args.vendor,args.config,args.checkpoint,checkpoint_sha256=args.checkpoint_sha256)
    rows=[]
    with torch.inference_mode():
        for entry in manifest['inputs']:
            path=args.inputs/entry['file']
            if hashlib.sha256(path.read_bytes()).hexdigest()!=entry['sha256']:raise ValueError('input digest mismatch')
            observed=np.load(path,allow_pickle=False)
            normalized,frame=normalize_partial(observed,radius=args.radius)
            prepared=sample_input(pcn_axes(normalized) if args.axis_order=='world-z-up' else normalized)
            tensor=torch.from_numpy(prepared[None]).cuda()
            start=time.perf_counter();prediction=model(tensor)[-1];torch.cuda.synchronize()
            first=(time.perf_counter()-start)*1000
            durations=[]
            for _ in range(5):
                start=time.perf_counter();model(tensor);torch.cuda.synchronize();durations.append((time.perf_counter()-start)*1000)
            generated=prediction[0].cpu().numpy()
            restored=restore_completion(pcn_axes(generated) if args.axis_order=='world-z-up' else generated,frame)
            if len(restored)!=16384 or not np.all(np.isfinite(restored)):raise ValueError('invalid model output')
            destination=args.output/(entry['id']+'.npz')
            np.savez_compressed(destination,observed=observed,predicted=restored,
                                center=frame.center,meters_per_unit=frame.meters_per_unit)
            with np.load(destination,allow_pickle=False) as saved:np.testing.assert_array_equal(saved['observed'],observed)
            row={**entry,'output_file':destination.name,'output_sha256':hashlib.sha256(destination.read_bytes()).hexdigest(),
                 'output_points':len(restored),'observed_unchanged':True,'first_forward_ms':first,
                 'forward_wall_ms_median':float(np.median(durations)),
                 'normalization':'observed centroid; observed max radius mapped to '+str(args.radius),
                 'axis_order':args.axis_order,
                 'meters_per_unit':frame.meters_per_unit}
            rows.append(row);print(json.dumps({'completed':len(rows),'objects':len(manifest['inputs']),'last_ms':row['forward_wall_ms_median']}),flush=True)
    result={'schema_version':1,'complete':True,'model':metadata,'objects':rows,'inputs':manifest,
            'gpu':torch.cuda.get_device_name(0),'gpu_count':torch.cuda.device_count(),
            'max_allocated_bytes':torch.cuda.max_memory_allocated(),'tf32':False,'truth_access':False,
            'normalization_radius':args.radius,'alignment':'none; sensor-derived inverse transform only'}
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
