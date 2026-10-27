"""Inference operators for AdaPoinTr on existing Torch/Triton CUDA installs.

FPS follows PointNet2_PyTorch sampling_gpu.cu (MIT): initial index zero,
near-zero exclusion, 512-lane tie priority and sequential furthest sampling.
Upstream: https://github.com/erikwijmans/Pointnet2_PyTorch
No nvcc, CPU copy, model parameter or network architecture change is required.
"""
import torch
import triton
import triton.language as tl


@triton.jit
def _fps(X,Out,N:tl.constexpr,M:tl.constexpr,Block:tl.constexpr,Lanes:tl.constexpr):
    batch=tl.program_id(0)
    index=tl.arange(0,Block)
    x=tl.load(X+batch*N*3+index*3,mask=index<N,other=0)
    y=tl.load(X+batch*N*3+index*3+1,mask=index<N,other=0)
    z=tl.load(X+batch*N*3+index*3+2,mask=index<N,other=0)
    valid=(index<N)&(((x*x+y*y)+z*z)>1e-3)
    distance=tl.full((Block,),1e10,tl.float32)
    old=tl.full((),0,tl.int32)
    tl.store(Out+batch*M,old)
    for j in range(1,M):
        a=tl.load(X+batch*N*3+old*3)
        b=tl.load(X+batch*N*3+old*3+1)
        c=tl.load(X+batch*N*3+old*3+2)
        d=((x-a)*(x-a)+(y-b)*(y-b))+(z-c)*(z-c)
        distance=tl.minimum(distance,d)
        candidates=tl.where(valid,distance,-1.)
        best=tl.max(candidates,0)
        lane=tl.min(tl.where(valid&(candidates==best),index%Lanes,2147483647),0)
        chosen=tl.min(tl.where(valid&(candidates==best)&(index%Lanes==lane),index,2147483647),0)
        old=tl.where(best>=0,chosen,0)
        tl.store(Out+batch*M+j,old)


def furthest_point_sample(points,count):
    if (not points.is_cuda or points.dtype!=torch.float32 or points.ndim!=3
            or points.shape[2]!=3 or not points.is_contiguous()
            or not 1<=count<=points.shape[1] or points.shape[1]>8192):
        raise ValueError('unsupported FPS input')
    n=points.shape[1]
    result=torch.empty((points.shape[0],count),device=points.device,dtype=torch.int32)
    _fps[(points.shape[0],)](points,result,n,count,triton.next_power_of_2(n),min(512,2**(n.bit_length()-1)),num_warps=8,enable_fp_fusion=False)
    return result


def gather_operation(features,indices):
    return torch.gather(features,2,indices.long().unsqueeze(1).expand(-1,features.shape[1],-1))


def unsupported_deformable_operation(*_args,**_kwargs):
    raise RuntimeError('This inference backend supports the graph AdaPoinTr configs only')
