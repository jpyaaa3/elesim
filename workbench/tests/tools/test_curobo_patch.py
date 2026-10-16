import hashlib

import pytest

from workbench.tools.patch_curobo import _patch_file


def digest(value):
    return hashlib.sha256(value).hexdigest()


def test_patch_is_exact_and_idempotent(tmp_path):
    path = tmp_path / "source"
    before, after = b"prefix old suffix", b"prefix new suffix"
    path.write_bytes(before)
    args = (path, digest(before), digest(after), [(b"old", b"new")])
    _patch_file(*args)
    assert path.read_bytes() == after
    _patch_file(*args)
    assert path.read_bytes() == after


@pytest.mark.parametrize("source,expected,target", [
    (b"unknown source", b"expected source", b"expected target"),
    (b"old", b"old", b"wrong target digest"),
    (b"old old", b"old old", b"new old"),
])
def test_patch_refuses_unknown_or_ambiguous_source_without_writing(tmp_path, source, expected, target):
    path = tmp_path / "source"
    path.write_bytes(source)
    with pytest.raises(RuntimeError):
        _patch_file(path, digest(expected), digest(target), [(b"old", b"new")])
    assert path.read_bytes() == source
