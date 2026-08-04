FROM ros:humble-ros-base-jammy

ARG PROTOCOL_WHEEL
ARG APP_WHEEL
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    AMENT_PREFIX_PATH=/opt/elesim/ros/install:/opt/ros/humble \
    CMAKE_PREFIX_PATH=/opt/elesim/ros/install:/opt/ros/humble \
    LD_LIBRARY_PATH=/opt/elesim/ros/install/lib:/opt/ros/humble/lib \
    PYTHONPATH=/opt/elesim/ros/install/local/lib/python3.10/dist-packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rosidl-default-generators \
      ros-humble-sros2 && \
    rm -rf /var/lib/apt/lists/*
COPY interfaces/elesim_interfaces/ /tmp/elesim/ros_ws/src/elesim_interfaces/
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
      --base-paths /tmp/elesim/ros_ws/src/elesim_interfaces \
      --build-base /tmp/elesim/ros_ws/build \
      --install-base /opt/elesim/ros/install \
      --log-base /tmp/elesim/ros_ws/log && \
    rm -rf /tmp/elesim/ros_ws
COPY wheels/ /tmp/wheels/
COPY requirements.lock /opt/elesim/requirements.lock
RUN python3 -m pip install --no-cache-dir -r /opt/elesim/requirements.lock && \
    python3 -m pip install --no-cache-dir --no-deps \
      "/tmp/wheels/${PROTOCOL_WHEEL}" "/tmp/wheels/${APP_WHEEL}"
COPY model/ /opt/elesim/model/
COPY config/ /opt/elesim/config/
WORKDIR /opt/elesim

CMD ["elesim-sim", "--config", "/opt/elesim/config/default.yaml", "--runtime-config", "/opt/elesim/config/runtime.yaml", "--model-bundle", "/opt/elesim/model/bundles/default"]
