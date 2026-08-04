FROM docker:27.5.1-cli AS docker_cli

FROM ros:humble-ros-base-jammy

COPY --from=docker_cli /usr/local/bin/docker /usr/local/bin/docker
COPY --from=docker_cli /usr/local/libexec/docker/cli-plugins/docker-compose \
    /usr/local/libexec/docker/cli-plugins/docker-compose

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONNOUSERSITE=1 \
    PATH=/opt/openrobots/bin:$PATH \
    PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH \
    LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH \
    PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH \
    CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
ARG COMPUTE_MODE=inherit

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash build-essential cmake curl gdb git iputils-ping lsb-release \
    mesa-utils sudo udev usbutils util-linux vim \
    python3-argcomplete python3-colcon-common-extensions python3-pip \
    python3-rosdep python3-tk python3-vcstool python3-venv \
    ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-default-generators \
    ros-humble-sros2 \
    libdbus-1-3 libegl1 libfontconfig1 libfreetype6 libgl1 libglfw3 \
    libglvnd0 libglu1-mesa libglx-mesa0 libice6 libosmesa6 libsm6 \
    libusb-1.0-0 libx11-6 libx11-xcb1 libxcb-icccm4 libxcb-image0 \
    libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-render0 \
    libxcb-shape0 libxcb-shm0 libxcb-sync1 libxcb-xfixes0 \
    libxcb-xinerama0 libxcb-xinput0 libxcb-xkb1 libxcb1 libxcursor1 \
    libxext6 libxft2 libxi6 libxinerama1 libxkbcommon-x11-0 \
    libxkbcommon0 libxrandr2 libxrender1 \
    && rm -rf /var/lib/apt/lists/*

COPY robotpkg.asc /etc/apt/keyrings/robotpkg.asc
RUN test "$(dpkg --print-architecture)" = amd64 && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub jammy robotpkg" \
      > /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends robotpkg-py310-pinocchio && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.lock /tmp/requirements.lock
RUN python3 -m pip install --no-cache-dir --upgrade "pip<26" "setuptools>=68,<80" wheel && \
    python3 -m pip install --no-cache-dir "numpy==1.26.4" && \
    if [ "$COMPUTE_MODE" = cpu ]; then \
      python3 -m pip install --no-cache-dir \
        --index-url https://download.pytorch.org/whl/cpu \
        --extra-index-url https://pypi.org/simple \
        "torch==2.12.1+cpu" "torchvision==0.27.1+cpu"; \
    fi && \
    python3 -m pip install --no-cache-dir --upgrade-strategy only-if-needed \
      -r /tmp/requirements.lock && \
    python3 -m pip install --no-cache-dir \
      "git+https://github.com/elijah-waichong-chan/go2-convex-mpc.git@1c63c6a762779887ab0431fd60db681dede6cb32" && \
    python3 -m pip check && \
    python3 -m pip cache purge

# aiortc needs a current cryptography release, while Humble's SROS2 CLI uses
# Ubuntu's patched 3.4 ABI.  Keep both: normal applications resolve /usr/local;
# the authority runner prepends /usr/lib/python3/dist-packages for ros2 CLI.
RUN apt-get update && apt-get install -y --reinstall --no-install-recommends \
      python3-cffi python3-cryptography && \
    rm -rf /var/lib/apt/lists/*

RUN if getent group "${GID}" >/dev/null; then \
      group_name="$(getent group "${GID}" | cut -d: -f1)"; \
    else \
      groupadd -g "${GID}" "${USERNAME}"; group_name="${USERNAME}"; \
    fi && \
    if getent passwd "${UID}" >/dev/null; then \
      user_name="$(getent passwd "${UID}" | cut -d: -f1)"; \
    else \
      useradd -m -s /bin/bash -u "${UID}" -g "${group_name}" "${USERNAME}"; \
      user_name="${USERNAME}"; \
    fi && \
    echo "${user_name} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN rosdep init || true

COPY --chmod=0755 dev-env.sh /usr/local/bin/elesim-dev-env
COPY --chmod=0755 entrypoint.sh /usr/local/bin/elesim-dev-entrypoint
USER ${UID}:${GID}
ENTRYPOINT ["/usr/local/bin/elesim-dev-entrypoint"]
CMD ["sleep", "infinity"]
