#!/usr/bin/env bash
# Optional Pilot planning environment; never starts EleSim or opens a device.
set -euo pipefail
planning_root="${1:-$HOME/.cache/elesim/curobo-integration}"
if [[ "$(uname -m)" != aarch64 || ! -f /etc/nv_tegra_release ]]; then
    echo "Run this on the Jetson with an existing CUDA-enabled PyTorch installation." >&2
    exit 1
fi
python3 -c 'import sys, torch; assert sys.version_info[:2] == (3, 10), "JetPack Python 3.10 required"; assert torch.cuda.is_available(); assert torch.cuda.get_device_capability() == (8, 7), "Orin required"; print(torch.__version__, torch.version.cuda)'
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}"
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST=8.7
export MAX_JOBS="${MAX_JOBS:-2}"
export CFLAGS="${CFLAGS:-} -fPIC"
export CXXFLAGS="${CXXFLAGS:-} -fPIC"
export SETUPTOOLS_USE_DISTUTILS=stdlib
nvcc --version
mkdir -p "$planning_root"
python3 -m venv --system-site-packages "$planning_root/venv"
planning_python="$planning_root/venv/bin/python"
export PATH="$planning_root/venv/bin:$PATH"
if [[ ! -d "$planning_root/curobo" ]]; then
    git clone --depth 1 --branch v0.7.8 https://github.com/NVlabs/curobo.git "$planning_root/curobo"
fi
if [[ "$(git -C "$planning_root/curobo" rev-parse HEAD)" != d64c4b005459db10c5dd867d8b30a87d5bda9bdb ]]; then
    echo "Expected cuRobo v0.7.8 commit d64c4b0; refusing to modify this checkout." >&2
    exit 1
fi
planning_changes="$(git -C "$planning_root/curobo" diff --name-only HEAD)"
while IFS= read -r planning_file; do
    case "$planning_file" in
        ""|src/curobo/cuda_robot_model/cuda_robot_generator.py|src/curobo/curobolib/cpp/self_collision_kernel.cu) ;;
        *) echo "Unexpected changes in cuRobo checkout: $planning_file" >&2; exit 1 ;;
    esac
done <<< "$planning_changes"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "$script_dir/patch_curobo.py" "$planning_root/curobo"
export SETUPTOOLS_SCM_PRETEND_VERSION=0.7.8+elesim.1
# Keep JetPack's existing torch; PyPI's generic torch must not replace it.
"$planning_python" -m pip install 'warp-lang==1.6.2' 'yourdfpy==0.0.58' \
    'numpy==1.26.4' numpy-quaternion 'setuptools==75.8.0' \
    'setuptools_scm==8.2.1' 'wheel==0.45.1' ninja pybind11
"$planning_python" -m pip install --no-deps --no-build-isolation "$planning_root/curobo"
"$planning_python" -c 'import torch, curobo; print(curobo.__version__, torch.cuda.get_device_name(0))'
echo "Install the Pilot release into this venv, then run the documented offline smoke test."
