#!/bin/sh
# Build pinned, untrained nominal PyMPC/acados once in the image, never at runtime.
set -eu

constraints=$1
generator=$2
repository=/tmp/elesim-quadruped-pympc
git clone --filter=blob:none https://github.com/iit-DLSLab/Quadruped-PyMPC.git "$repository"
git -C "$repository" checkout 814cd3e8f5733a6652b0df30420b75882a48b0be
test "$(git -C "$repository" rev-parse HEAD)" = 814cd3e8f5733a6652b0df30420b75882a48b0be
git init /opt/acados
git -C /opt/acados remote add origin https://github.com/acados/acados.git
git -C /opt/acados fetch --depth 1 origin 5d358fe80c1037a0feeb8ba1021fcd354f1be8c2
git -C /opt/acados checkout FETCH_HEAD
test "$(git -C /opt/acados rev-parse HEAD)" = 5d358fe80c1037a0feeb8ba1021fcd354f1be8c2
git -C /opt/acados submodule update --init --recursive --depth 1
cmake -S /opt/acados -B /opt/acados/build -DCMAKE_BUILD_TYPE=Release \
  -DACADOS_INSTALL_DIR=/opt/acados \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build /opt/acados/build --parallel 4
cmake --install /opt/acados/build
mkdir -p /opt/acados/bin
curl --fail --location --silent --show-error \
  https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux \
  -o /opt/acados/bin/t_renderer
echo '390063f34a8e13620564b4a136012270168e1421dd7920a747048749e1d99718  /opt/acados/bin/t_renderer' | sha256sum -c -
chmod 0755 /opt/acados/bin/t_renderer
python3 -m pip install -c "$constraints" 'gym-quadruped==1.1.5'
python3 -m pip install -c "$constraints" /opt/acados/interfaces/acados_template "$repository"
python3 "$generator"
python3 -m pip check
rm -rf "$repository" /opt/acados/build
