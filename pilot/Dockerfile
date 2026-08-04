FROM ros:humble-ros-base-jammy

ARG PROTOCOL_WHEEL
ARG APP_WHEEL

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    AMENT_PREFIX_PATH=/opt/elesim/ros/install \
    CMAKE_PREFIX_PATH=/opt/elesim/ros/install \
    PYTHONPATH=/opt/elesim/ros/install/local/lib/python3.10/dist-packages \
    LD_LIBRARY_PATH=/opt/elesim/ros/install/lib

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      libgl1 \
      libglib2.0-0 \
      libgomp1 \
      python3-colcon-common-extensions \
      python3-pip \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rosidl-default-generators \
      ros-humble-sros2 && \
    rm -rf /var/lib/apt/lists/*

COPY interfaces/elesim_interfaces/ /tmp/ros_ws/src/elesim_interfaces/
RUN . /opt/ros/humble/setup.sh && \
    colcon --log-base /tmp/ros_ws/log build \
      --base-paths /tmp/ros_ws/src/elesim_interfaces \
      --build-base /tmp/ros_ws/build \
      --install-base /opt/elesim/ros/install && \
    rm -rf /tmp/ros_ws/build /tmp/ros_ws/log

COPY wheels/ /tmp/wheels/
COPY requirements.lock /opt/elesim/requirements.lock
COPY config/ /opt/elesim/config/
RUN python3 -m pip install --no-cache-dir -r /opt/elesim/requirements.lock && \
    python3 -m pip install --no-cache-dir --no-deps \
      "/tmp/wheels/${PROTOCOL_WHEEL}" \
      "/tmp/wheels/${APP_WHEEL}" && \
    python3 -m pip check && \
    rm -rf /tmp/wheels

WORKDIR /opt/elesim
ENTRYPOINT ["/ros_entrypoint.sh", "elesim-pilot", "--config", "/opt/elesim/config/default.yaml", "--runtime-config", "/opt/elesim/config/runtime.yaml"]
