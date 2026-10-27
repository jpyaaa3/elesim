"""Verify inference operators and original pretrained AdaPoinTr on one GPU."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from workbench.research.experiments.adapointr import fps_reference,sample_input


def verify_operators():
    import torch
    from workbench.research.support.adapointr_ops import furthest_point_sample,gather_operation
    results=[]
    for seed in (1,7):
        for n,m in ((32,16),(513,128),(2048,512)):
            points=np.random.default_rng(seed).normal(size=(n,3)).astype(np.float32)
            points[2:5]=0
            expected=fps_reference(points,m)
            got=furthest_point_sample(torch.from_numpy(points[None]).cuda(),m).cpu().numpy()[0]
            if not np.array_equal(got,expected):
                raise AssertionError({'fps_mismatch':{'seed':seed,'n':n,'indices':np.flatnonzero(got!=expected)[:10].tolist()}})
            results.append({'seed':seed,'n':n,'m':m,'indices_equal':True})
    for points in (np.zeros((513,3),np.float32),np.tile(np.eye(3,dtype=np.float32),(171,1))):
        expected=fps_reference(points,64)
        got=furthest_point_sample(torch.from_numpy(points[None]).cuda(),64).cpu().numpy()[0]
        np.testing.assert_array_equal(got,expected)
        results.append({'degenerate_or_ties':True,'indices_equal':True})
    features=np.arange(2*7*100,dtype=np.float32).reshape(2,7,100)
    indices=np.array([[99,1,4,1],[13,5,0,99]],dtype=np.int32)
    actual=gather_operation(torch.from_numpy(features).cuda(),torch.from_numpy(indices).cuda()).cpu().numpy()
    expected=np.stack([features[b][:,indices[b]] for b in range(2)])
    np.testing.assert_array_equal(actual,expected)
    return {'fps_cases':results,'gather_equal':True}


def read_demo(path):
    with Path(path).open() as stream:
        for line in stream:
            if line.startswith('DATA'):
                if line.strip()!='DATA ascii':raise ValueError('unsupported demo format')
                break
        return np.loadtxt(stream,dtype=np.float32)


def main():
    import torch
    from workbench.research.support.adapointr_backend import load_model
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor',type=Path,required=True)
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--checkpoint-sha256',required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    assert torch.cuda.is_available() and torch.cuda.device_count()==1
    torch.set_num_threads(2);torch.manual_seed(17)
    torch.backends.cuda.matmul.allow_tf32=False
    torch.backends.cudnn.allow_tf32=False
    began=time.perf_counter();operators=verify_operators()
    (args.output/'operators.json').write_text(json.dumps(operators,indent=2)+'\n')
    print(json.dumps({'stage':'operators_passed','wall_s':time.perf_counter()-began}),flush=True)
    model,metadata=load_model(args.vendor,args.config,args.checkpoint,checkpoint_sha256=args.checkpoint_sha256)
    results=[]
    with torch.inference_mode():
        for name in ('airplane','sofa'):
            points=read_demo(args.vendor/'demo'/(name+'.pcd'))
            prepared=torch.from_numpy(sample_input(points)[None]).cuda()
            start=time.perf_counter();first=model(prepared)[-1];torch.cuda.synchronize()
            first_s=time.perf_counter()-start
            repeat=model(prepared)[-1];torch.cuda.synchronize()
            difference=float(torch.max(torch.abs(first-repeat)).item())
            if first.shape!=(1,16384,3) or not torch.isfinite(first).all():raise AssertionError('invalid model output')
            durations=[]
            for _ in range(12):
                start=time.perf_counter();model(prepared);torch.cuda.synchronize();durations.append((time.perf_counter()-start)*1000)
            np.savez_compressed(args.output/(name+'.npz'),observed=points,predicted=first[0].cpu().numpy())
            results.append({'name':name,'input_points':len(points),'model_input_points':2048,'output_points':16384,
                            'first_forward_s':first_s,'repeat_max_abs':difference,
                            'forward_wall_ms':{'median':float(np.median(durations)),'p95':float(np.percentile(durations,95))}})
            print(json.dumps(results[-1]),flush=True)
    result={'complete':True,'operators':operators,'model':metadata,'demos':results,
            'torch':torch.__version__,'cuda':torch.version.cuda,'gpu':torch.cuda.get_device_name(0),
            'gpu_count':torch.cuda.device_count(),'max_allocated_bytes':torch.cuda.max_memory_allocated(),
            'tf32':False,'scope':'batch-one isolated FP32 inference; demo inputs have no complete ground truth'}
    (args.output/'smoke.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
