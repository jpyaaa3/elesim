"""Complete recorded partial objects with strict pretrained AdaPoinTr weights."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from workbench.research.experiments.adapointr import sample_input,pcn_axes
from workbench.research.support.adapointr_profiles import prepare_cloud,validate_profile,restore_prediction


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--vendor',type=Path,required=True);parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--checkpoint-sha256',required=True);parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--radius',type=float,default=.5)
    parser.add_argument('--profile',choices=['pcn','shapenet55','projected55'],default='pcn')
    parser.add_argument('--normalization',choices=['observed-radius','center-only','native'],default='observed-radius')
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--axis-order',choices=['world-z-up','model-native'],default='world-z-up')
    from workbench.research.support.output import fresh_directory
    args=parser.parse_args();fresh_directory(args.output)
    import torch
    import yaml
    from workbench.research.support.adapointr_backend import load_model
    assert torch.cuda.is_available() and torch.cuda.device_count()==1
    torch.set_num_threads(2);torch.manual_seed(17)
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    manifest=json.loads((args.inputs/'manifest.json').read_text())
    profile=validate_profile(yaml.safe_load(args.config.read_text()),args.profile)
    model,metadata=load_model(args.vendor,args.config,args.checkpoint,checkpoint_sha256=args.checkpoint_sha256)
    rows=[]
    with torch.inference_mode():
        for entry in manifest['inputs']:
            path=args.inputs/entry['file']
            if hashlib.sha256(path.read_bytes()).hexdigest()!=entry['sha256']:raise ValueError('input digest mismatch')
            observed=np.load(path,allow_pickle=False)
            normalized,frame=prepare_cloud(observed,args.normalization,radius=args.radius)
            prepared=sample_input(pcn_axes(normalized) if args.axis_order=='world-z-up' else normalized,seed=args.seed)
            tensor=torch.from_numpy(prepared[None]).cuda()
            start=time.perf_counter();prediction=model(tensor)[-1];torch.cuda.synchronize()
            first=(time.perf_counter()-start)*1000
            durations=[]
            for _ in range(5):
                start=time.perf_counter();model(tensor);torch.cuda.synchronize();durations.append((time.perf_counter()-start)*1000)
            generated=prediction[0].cpu().numpy()
            restored=restore_prediction(generated,frame,args.axis_order,expected_points=profile['output_points'])
            destination=args.output/(entry['id']+'.npz')
            np.savez_compressed(destination,observed=observed,predicted=restored,
                                center=frame.center,meters_per_unit=frame.meters_per_unit)
            with np.load(destination,allow_pickle=False) as saved:np.testing.assert_array_equal(saved['observed'],observed)
            row={**entry,'output_file':destination.name,'output_sha256':hashlib.sha256(destination.read_bytes()).hexdigest(),
                 'output_points':len(restored),'observed_unchanged':True,'first_forward_ms':first,
                 'forward_wall_ms_median':float(np.median(durations)),
                 'normalization':args.normalization,
                 'radius':args.radius if args.normalization=='observed-radius' else None,
                 'axis_order':args.axis_order,
                 'sampling_seed':args.seed,
                 'model_input_near_zero_points':int(np.sum(np.sum(prepared*prepared,axis=1)<=1e-3)),
                 'meters_per_unit':frame.meters_per_unit}
            rows.append(row);print(json.dumps({'completed':len(rows),'objects':len(manifest['inputs']),'last_ms':row['forward_wall_ms_median']}),flush=True)
    result={'schema_version':1,'complete':True,'model':metadata,'objects':rows,'inputs':manifest,
            'gpu':torch.cuda.get_device_name(0),'gpu_count':torch.cuda.device_count(),
            'max_allocated_bytes':torch.cuda.max_memory_allocated(),'tf32':False,'truth_access':False,
            'normalization_radius':args.radius if args.normalization=='observed-radius' else None,
            'normalization':args.normalization,'model_profile':profile,'sampling_seed':args.seed,
            'official_affine_matched':bool(
                (profile['official_affine']=='none' and args.normalization=='native') or
                (profile['official_affine']=='observed-unit-radius' and args.normalization=='observed-radius' and args.radius==1.)),
            'sampling_policy':'deterministic full-cloud random selection or observed-point repetition; not upstream UpSamplePoints',
            'alignment':'none; sensor-derived inverse transform only'}
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
