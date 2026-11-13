#!/bin/bash
# Research-only V2 environment. Invoke explicitly on the CUDA server.
set -euo pipefail
if [ "$#" -ne 3 ]; then
    echo "Usage: bash setup_v2.sh EXISTING_TORCH_PYTHON NEW_VENV NEW_CUROBO_SOURCE" >&2
    exit 2
fi
# Preserve a selected venv's interpreter path; resolving its symlink would
# inspect the base interpreter's site-packages instead of that environment.
torch_python=$(realpath -s "$1")
v2_environment=$(realpath -m "$2")
v2_source=$(realpath -m "$3")
test ! -e "$v2_environment"
test ! -e "$v2_source"
"$torch_python" -m venv "$v2_environment"
# Existing Torch is read-only fallback; newly installed V2 packages take priority.
v2_site=$("$v2_environment/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')
"$torch_python" -c 'import sys; print("\n".join(p for p in sys.path if p.endswith("site-packages")))' > "$v2_site/existing_torch.pth"
GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 --branch v0.9.0 https://github.com/NVlabs/curobo.git "$v2_source"
test "$(git -C "$v2_source" rev-parse HEAD)" = 42498cf4e7d0d6fbd4ef9fbcec1243c00c888a83
"$v2_environment/bin/python" -m pip install "$v2_source[cu13]" \
    warp-lang==1.19.0 cuda-core==1.3.0 cuda-bindings==13.5.0
"$v2_environment/bin/python" -m pip freeze > "$v2_environment/requirements-resolved.txt"
"$v2_environment/bin/python" -c 'import importlib.metadata; assert importlib.metadata.version("nvidia-curobo") == "0.9.0"'
