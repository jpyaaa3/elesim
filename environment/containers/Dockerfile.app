ARG BASE_IMAGE=ros:humble-ros-base-jammy
FROM ${BASE_IMAGE}

ARG ROLE
ARG COMPUTE_MODE=inherit
ARG INSTALL_GO2_MPC=1
ARG CASADI_GIT_REF=3.7.2
ARG CASADI_GIT_COMMIT=f959d3175a444d763e4eda4aece48f4c5f4a6f90
ARG OSQP_GIT_REF=v0.6.3
ARG CASADI_BUILD_JOBS=4
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    HOME=/var/lib/elesim \
    PATH=/opt/openrobots/bin:$PATH \
    PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig \
    LD_LIBRARY_PATH=/opt/openrobots/lib \
    PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages \
    CMAKE_PREFIX_PATH=/opt/openrobots

RUN set -eux; \
    packages="ca-certificates python3-colcon-common-extensions python3-pip python3-venv python-is-python3 ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-default-generators ros-humble-sros2"; \
    case "$ROLE" in \
      pilot) packages="$packages libgl1 libglib2.0-0 libgomp1" ;; \
      ui) packages="$packages libgl1 libgl1-mesa-dri libglx-mesa0 libglu1-mesa libglfw3 libx11-6 libxcursor1 libxi6 libxinerama1 libxrandr2 libxxf86vm1 libfontconfig1" ;; \
      sim) packages="$packages build-essential cmake git python3-dev swig python3-pip python-is-python3 libgl1 libegl1 libglx-mesa0 libglu1-mesa libosmesa6 libglfw3 libglib2.0-0 libx11-6 libxext6 libxrender1" ;; \
      *) echo "unsupported role: $ROLE" >&2; exit 2 ;; \
    esac; \
    apt-get update; \
    apt-get install -y --no-install-recommends $packages; \
    rm -rf /var/lib/apt/lists/*; \
    mkdir -p "$HOME" /opt/elesim

# Pinocchio's pip dependency graph is not compatible with the NumPy 1.x ABI
# used by this project.  Sim images use the same Robotpkg build as the
# validated development image instead.
COPY robotpkg.asc /etc/apt/keyrings/robotpkg.asc
RUN if [ "$ROLE" = sim ]; then \
      test "$(dpkg --print-architecture)" = amd64 || { \
        echo "sim container currently supports amd64 only" >&2; exit 2; \
      }; \
      echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub jammy robotpkg" \
        > /etc/apt/sources.list.d/robotpkg.list; \
      apt-get update; \
      apt-get install -y --no-install-recommends robotpkg-py310-pinocchio; \
      rm -rf /var/lib/apt/lists/*; \
    fi

# Robotpkg supplies the CasADi core used by Pinocchio, and its default build
# does not include CasADi's native OSQP conic interface.  Build the pinned
# CasADi release in that same prefix so the runtime's /opt/openrobots
# PYTHONPATH/LD_LIBRARY_PATH cannot silently select the plugin-less copy.
RUN if [ "$ROLE" = sim ]; then \
      git clone --depth 1 --branch "$CASADI_GIT_REF" \
        https://github.com/casadi/casadi.git /tmp/casadi; \
      test "$(git -C /tmp/casadi rev-parse HEAD)" = "$CASADI_GIT_COMMIT"; \
      cmake -S /tmp/casadi -B /tmp/casadi-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/openrobots \
        -DPYTHON_PREFIX=/opt/openrobots/lib/python3.10/site-packages \
        -DWITH_PYTHON=ON \
        -DWITH_PYTHON3=ON \
        -DWITH_OSQP=ON \
        -DWITH_BUILD_OSQP=ON \
        -DBUILD_OSQP_VERSION="$OSQP_GIT_REF" \
        -DWITH_EXAMPLES=OFF \
        -DWITH_DOC=OFF; \
      cmake --build /tmp/casadi-build --parallel "$CASADI_BUILD_JOBS"; \
      cmake --install /tmp/casadi-build; \
      ldconfig; \
      python -c 'import casadi as ca; assert ca.__version__ == "3.7.2", ca.__version__; assert ca.has_conic("osqp"), ca.CasadiMeta_plugins()'; \
      rm -rf /tmp/casadi /tmp/casadi-build; \
    fi

COPY requirements.lock /opt/elesim/requirements.lock
RUN python -m pip install --no-cache-dir --upgrade "pip<26" "setuptools>=68,<80" wheel && \
    if [ "$ROLE" = pilot ] || [ "$ROLE" = sim ]; then \
      if [ "$COMPUTE_MODE" = cpu ]; then \
        torch_index="https://download.pytorch.org/whl/cpu"; \
        torch_version="2.12.1+cpu"; \
      else \
        torch_index="https://pypi.org/simple"; \
        torch_version="2.12.1"; \
      fi; \
      if [ "$ROLE" = pilot ]; then \
        python -m pip install --no-cache-dir --index-url "$torch_index" \
          "torch==$torch_version" "torchvision==0.27.1"; \
      else \
        python -m pip install --no-cache-dir --index-url "$torch_index" \
          "torch==$torch_version"; \
      fi; \
    fi && \
    python -m pip install --no-cache-dir -r /opt/elesim/requirements.lock

COPY interfaces/elesim_interfaces/ /tmp/elesim/ros_ws/src/elesim_interfaces/
RUN . /opt/ros/humble/setup.sh && \
    colcon --log-base /tmp/elesim/ros_ws/log build \
      --base-paths /tmp/elesim/ros_ws/src/elesim_interfaces \
      --build-base /tmp/elesim/ros_ws/build \
      --install-base /opt/elesim/ros/install && \
    rm -rf /tmp/elesim/ros_ws/build /tmp/elesim/ros_ws/log

COPY protocol/ /tmp/elesim/protocol/
COPY application/ /tmp/elesim/application/
RUN if [ "$ROLE" = sim ] && [ "$INSTALL_GO2_MPC" = 1 ]; then \
      python -m pip install --no-cache-dir \
        "git+https://github.com/elijah-waichong-chan/go2-convex-mpc.git@1c63c6a762779887ab0431fd60db681dede6cb32"; \
    fi && \
    python -m pip install --no-cache-dir --no-deps /tmp/elesim/protocol /tmp/elesim/application && \
    if [ "$ROLE" = sim ]; then \
      python -m pip install --no-cache-dir "setuptools>=68,<80"; \
    fi && \
    python -m pip check && \
    rm -rf /tmp/elesim

COPY entrypoint /usr/local/bin/elesim-entrypoint
RUN chmod 0755 /usr/local/bin/elesim-entrypoint
WORKDIR /opt/elesim
ENTRYPOINT ["/usr/local/bin/elesim-entrypoint"]
