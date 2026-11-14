"""Pinned external completion assets, materialized into immutable Pilot data."""
from pathlib import Path
import hashlib
import os
import shutil
import tempfile
from urllib.request import urlopen

ASSETS = {
 'pointr-source.tar.gz': ('https://codeload.github.com/yuxumin/PoinTr/tar.gz/4603257ed3db9e7dad349b712e1b2fe0da207015', 'ab52d68e84bf79e7d7969a826c93672c2ff4d997b714a571b722bcd5f4be668e'),
 'ap_ps55.pth': ('https://huggingface.co/robotic-vt-grasp-project/SGA-GSN/resolve/133836b26c3af5806e2594882c2d445f0e0bda24/checkpoints/shape_completion/ap_ps55.pth', '66e22112aa43e74cd0716012201ea40bda0492e8163897e6e45102e86cb781b7'),
}
MAX_BYTES = 256*1024*1024


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def supply(destination, *, cache=None, opener=urlopen):
    destination=Path(destination)
    cache=Path(cache or os.environ.get('ELESIM_COMPLETION_ASSET_CACHE',
        str(Path(os.environ.get('XDG_CACHE_HOME',Path.home()/'.cache'))/'elesim/model-assets')))
    cache.mkdir(parents=True,exist_ok=True);destination.mkdir(parents=True,exist_ok=True)
    for name,(url,expected) in ASSETS.items():
        target=destination/name
        if target.is_file() and digest(target)==expected:continue
        cached=cache/(expected+'-'+name)
        if not cached.is_file() or digest(cached)!=expected:
            print(f'[models] downloading verified completion asset: {name}',flush=True)
            with tempfile.NamedTemporaryFile(dir=cache,delete=False) as f:
                temporary=Path(f.name)
                try:
                    with opener(url,timeout=60) as response:
                        total=0
                        while chunk:=response.read(1024*1024):
                            total+=len(chunk)
                            if total>MAX_BYTES:raise ValueError('Completion asset exceeds size limit')
                            f.write(chunk)
                    f.flush()
                    if digest(temporary)!=expected:raise ValueError(f'Completion asset checksum mismatch: {name}')
                    os.replace(temporary,cached)
                finally:temporary.unlink(missing_ok=True)
        with tempfile.NamedTemporaryFile(dir=destination,delete=False) as f:temporary=Path(f.name)
        try:
            shutil.copyfile(cached,temporary)
            if digest(temporary)!=expected:raise ValueError('Completion asset copy failed verification')
            temporary.chmod(0o644);os.replace(temporary,target)
        finally:temporary.unlink(missing_ok=True)
