ARG BASE_IMAGE=ros:humble-ros-base-jammy
FROM ${BASE_IMAGE}

ARG ROLE
ARG COMPUTE_MODE=inherit
ARG INSTALL_GO2_MPC=1
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
      sim) packages="$packages git python3-pip python-is-python3 libgl1 libegl1 libglx-mesa0 libglu1-mesa libosmesa6 libglfw3 libglib2.0-0 libx11-6 libxext6 libxrender1" ;; \
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
      python -m pip install --no-cache-dir "git+https://github.com/elijah-waichong-chan/go2-convex-mpc.git"; \
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
