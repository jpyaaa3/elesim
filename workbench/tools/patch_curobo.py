"""Developer entry point for the patch shipped with Pilot build contexts."""
from pathlib import Path
import argparse
import runpy

_patch = runpy.run_path(str(Path(__file__).resolve().parents[2] / "payload/runtime/docker/shared/patch_curobo.py"))
_patch_file = _patch["_patch_file"]
patch = _patch["patch"]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    patch(parser.parse_args().source)
