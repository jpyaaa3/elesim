import hashlib
import io
import pytest
from elesim_setup import completion_assets as a


def test_download_verify_cache_and_publish(tmp_path,monkeypatch):
    data=b'fixed-model';sha=hashlib.sha256(data).hexdigest()
    monkeypatch.setattr(a,'ASSETS',{'model.pth':('https://example.test/model',sha)})
    calls=[]
    def open_(url,timeout):calls.append(url);return io.BytesIO(data)
    a.supply(tmp_path/'one',cache=tmp_path/'cache',opener=open_)
    a.supply(tmp_path/'two',cache=tmp_path/'cache',opener=open_)
    assert len(calls)==1
    assert (tmp_path/'two/model.pth').read_bytes()==data


def test_bad_download_never_published(tmp_path,monkeypatch):
    monkeypatch.setattr(a,'ASSETS',{'model.pth':('https://example.test/model','0'*64)})
    with pytest.raises(ValueError,match='checksum'):
        a.supply(tmp_path/'out',cache=tmp_path/'cache',opener=lambda *a,**k:io.BytesIO(b'bad'))
    assert not (tmp_path/'out/model.pth').exists()
    assert not list((tmp_path/'cache').iterdir())
