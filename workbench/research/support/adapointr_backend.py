"""Strictly load pinned upstream AdaPoinTr for isolated inference trials.

Only AdaPoinTr modules are imported: unrelated model/training CUDA extensions
are not needed. DropPath/truncated-normal use equivalent native Torch operations;
the training loss is unavailable. Existing runtime packages are not modified.
"""
from pathlib import Path
import hashlib
import importlib
import sys
import types


class AttrDict(dict):
    def __getattr__(self,name):
        try:return self[name]
        except KeyError:raise AttributeError(name) from None


def _attributes(value):
    if isinstance(value,dict):return AttrDict({k:_attributes(v) for k,v in value.items()})
    if isinstance(value,list):return [_attributes(v) for v in value]
    return value


def _namespace(name,path=None):
    module=types.ModuleType(name)
    if path is not None:module.__path__=[str(path)]
    sys.modules[name]=module
    return module


def load_model(vendor,config_path,checkpoint_path,*,checkpoint_sha256):
    import torch
    import yaml
    from workbench.research.support import adapointr_ops as ops

    checkpoint=Path(checkpoint_path)
    digest=hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    if digest!=checkpoint_sha256:raise ValueError('checkpoint digest mismatch')
    config=_attributes(yaml.safe_load(Path(config_path).read_text())['model'])
    if config.NAME!='AdaPoinTr' or config.encoder_type!='graph':
        raise ValueError('unsupported model config')
    vendor=Path(vendor)
    _namespace('models',vendor/'models');_namespace('utils',vendor/'utils')
    _namespace('extensions',vendor/'extensions')
    loss=_namespace('extensions.chamfer_dist')
    class InferenceOnlyLoss(torch.nn.Module):
        def forward(self,*_args):raise RuntimeError('training loss is unavailable in inference trial')
    loss.ChamferDistanceL1=InferenceOnlyLoss
    pointnet=_namespace('pointnet2_ops')
    pointnet.pointnet2_utils=types.SimpleNamespace(
        furthest_point_sample=ops.furthest_point_sample,gather_operation=ops.gather_operation,
        three_nn=ops.unsupported_deformable_operation,three_interpolate=ops.unsupported_deformable_operation)
    _namespace('timm');_namespace('timm.models');layers=_namespace('timm.models.layers')
    class DropPath(torch.nn.Module):
        def __init__(self,drop_prob=0.):super().__init__();self.drop_prob=drop_prob
        def forward(self,x):
            if not self.training or not self.drop_prob:return x
            keep=1-self.drop_prob
            random=keep+torch.rand((x.shape[0],)+(1,)*(x.ndim-1),device=x.device,dtype=x.dtype)
            return x/keep*random.floor()
    layers.DropPath=DropPath;layers.trunc_normal_=torch.nn.init.trunc_normal_
    architecture=importlib.import_module('models.AdaPoinTr')
    model=architecture.AdaPoinTr(config)
    raw=torch.load(checkpoint,map_location='cpu',weights_only=True)
    state=raw['base_model']
    state={key.removeprefix('module.'):value for key,value in state.items()}
    loaded=model.load_state_dict(state,strict=True)
    model=model.eval().cuda()
    return model,{'checkpoint_sha256':digest,'parameters':sum(p.numel() for p in model.parameters()),
                  'missing_keys':loaded.missing_keys,'unexpected_keys':loaded.unexpected_keys,
                  'config':dict(config),'backend':'original upstream forward; Triton FPS and Torch gather; inference-only imports'}
