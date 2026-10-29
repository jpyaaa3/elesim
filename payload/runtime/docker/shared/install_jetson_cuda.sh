#!/bin/sh
# Container-only user-space dependencies for the pinned Orin CUDA 12.6 wheels.
# Do not install Jetson drivers, kernel packages, or nvidia-l4t-* packages.
set -eu

test "$(dpkg --print-architecture)" = arm64 || {
  echo "Jetson CUDA 12.6 dependencies require arm64" >&2
  exit 2
}

temporary="$(mktemp -d /tmp/elesim-jetson-cuda.XXXXXX)"
trap 'rm -rf -- "$temporary"' EXIT HUP INT TERM
curl -fL --retry 3 https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
  -o "$temporary/jetson.asc"
printf '%s  %s\n' 576f852981855e5c6cfb9b625ffb51b984ca451f1181b2e70435b005034fad55 \
  "$temporary/jetson.asc" | sha256sum -c -
mkdir -p /etc/apt/keyrings
cp "$temporary/jetson.asc" /etc/apt/keyrings/elesim-jetson.asc
printf '%s\n' 'deb [arch=arm64 signed-by=/etc/apt/keyrings/elesim-jetson.asc] https://repo.download.nvidia.com/jetson/common r36.5 main' \
  > /etc/apt/sources.list.d/elesim-jetson-cuda.list
# The Dockerfile supplies a locked external archive cache. Keep successful
# downloads there when a later package or linkage check fails.
rm -f /etc/apt/apt.conf.d/docker-clean
apt-get -o Acquire::Retries=5 update
apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
  cuda-libraries-12-6=12.6.11-1 cuda-cupti-12-6=12.6.68-1 \
  libcudnn9-cuda-12=9.3.0.75-1 libopenblas0-pthread libjpeg-turbo8 libpng16-16
rm -rf /var/lib/apt/lists/*

# Torch's graph fallback needs the CUDA 12.6 Update 2 Jetson cuSolver ABI.
curl -fL --retry 3 \
  https://developer.download.nvidia.com/compute/cuda/redist/libcusolver/linux-aarch64/libcusolver-linux-aarch64-11.7.1.2-archive.tar.xz \
  -o "$temporary/cusolver.tar.xz"
printf '%s  %s\n' d7e578143ebd90582873cb4c84f0f29ae1b3f086104a7660f5a63a28a115b9e6 \
  "$temporary/cusolver.tar.xz" | sha256sum -c -
mkdir -p /opt/elesim/cuda-compat/lib
tar -xJf "$temporary/cusolver.tar.xz" --strip-components=2 \
  -C /opt/elesim/cuda-compat/lib \
  libcusolver-linux-aarch64-11.7.1.2-archive/lib/libcusolver.so.11.7.1.2
ln -sf libcusolver.so.11.7.1.2 /opt/elesim/cuda-compat/lib/libcusolver.so.11
printf '%s\n' /opt/elesim/cuda-compat/lib > /etc/ld.so.conf.d/00-elesim-cusolver.conf

# This Jetson PyTorch build links cuDSS. Use NVIDIA's aarch64/CUDA12 archive,
# not the separately published SBSA or CUDA13 binaries.
curl -fL --retry 3 \
  https://developer.download.nvidia.com/compute/cudss/redist/libcudss/linux-aarch64/libcudss-linux-aarch64-0.8.0.10_cuda12-archive.tar.xz \
  -o "$temporary/cudss.tar.xz"
printf '%s  %s\n' 11cf97c8cecf3a651bab1890894c8b8793666a17ad3002104e573efa564af231 \
  "$temporary/cudss.tar.xz" | sha256sum -c -
mkdir -p /usr/local/cudss
tar -xJf "$temporary/cudss.tar.xz" --strip-components=1 -C /usr/local/cudss
printf '%s\n' /usr/local/cuda-12.6/targets/aarch64-linux/lib /usr/local/cudss/lib \
  > /etc/ld.so.conf.d/elesim-jetson-cuda.conf
ldconfig
