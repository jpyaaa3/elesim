# syntax=docker/dockerfile:1
FROM docker:27.5.1-cli AS docker_cli

FROM ros:humble-ros-base-jammy

COPY --from=docker_cli /usr/local/bin/docker /usr/local/bin/docker
COPY --from=docker_cli /usr/local/libexec/docker/cli-plugins/docker-compose \
    /usr/local/libexec/docker/cli-plugins/docker-compose

ENV PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-pip python-is-python3 \
      iproute2 \
      ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-default-generators \
      ros-humble-sros2 && \
    rm -rf /var/lib/apt/lists/*
COPY app/requirements.lock /tmp/elesim/
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked python -m pip install --upgrade "pip<26" "setuptools>=68,<80" wheel && \
    python -m pip install -r /tmp/elesim/requirements.lock
RUN apt-get update && apt-get install -y --reinstall --no-install-recommends \
      python3-cffi python3-cryptography && \
    rm -rf /var/lib/apt/lists/*

COPY interfaces/elesim_interfaces/ /tmp/elesim/ros_ws/src/elesim_interfaces/
RUN . /opt/ros/humble/setup.sh && \
    colcon --log-base /tmp/elesim/ros_ws/log build \
      --base-paths /tmp/elesim/ros_ws/src/elesim_interfaces \
      --build-base /tmp/elesim/ros_ws/build \
      --install-base /opt/elesim/ros/install && \
    test -f /opt/elesim/ros/install/local_setup.bash && \
    rm -rf /tmp/elesim/ros_ws/build /tmp/elesim/ros_ws/log

COPY protocol/ /tmp/elesim/protocol/
COPY app/ /tmp/elesim/app/
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked python -m pip install --no-deps /tmp/elesim/protocol /tmp/elesim/app && \
    python -m pip check && \
    rm -rf /tmp/elesim

COPY tools-entrypoint /usr/local/bin/elesim-tools-entrypoint
RUN chmod 0755 /usr/local/bin/elesim-tools-entrypoint
ENTRYPOINT ["/usr/local/bin/elesim-tools-entrypoint"]
