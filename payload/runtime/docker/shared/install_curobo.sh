#!/bin/sh
# Container-only cuRobo build for Orin; no GPU or device is opened during build.
set -eu
export CUDA_HOME=/usr/local/cuda-12.6
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST=8.7
export MAX_JOBS="${MAX_JOBS:-2}"
export SETUPTOOLS_SCM_PRETEND_VERSION=0.7.8+elesim.1
export SETUPTOOLS_USE_DISTUTILS=stdlib
planning_source="$(mktemp -d /tmp/elesim-curobo.XXXXXX)"
trap 'rm -rf -- "$planning_source"' EXIT HUP INT TERM
git clone --depth 1 --branch v0.7.8 https://github.com/NVlabs/curobo.git "$planning_source/source"
test "$(git -C "$planning_source/source" rev-parse HEAD)" = d64c4b005459db10c5dd867d8b30a87d5bda9bdb
python /tmp/elesim/patch_curobo.py "$planning_source/source"
python -m pip install 'numpy==1.26.4' 'warp-lang==1.6.2' 'yourdfpy==0.0.58' \
    'numpy-quaternion==2024.0.7' 'setuptools==75.8.0' 'setuptools_scm==8.2.1' \
    'wheel==0.45.1' ninja 'pybind11==2.13.6' 'importlib_resources==6.5.2' \
    'scikit-image==0.24.0' 'tqdm==4.67.1'
# Preserve the Jetson Torch stack selected by requirements.jetson.lock.
python -m pip install --no-deps --no-build-isolation "$planning_source/source"
python -c 'import importlib.metadata; assert importlib.metadata.version("nvidia-curobo") == "0.7.8+elesim.1"; import curobo.curobolib.geom_cu'
