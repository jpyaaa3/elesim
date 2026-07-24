# elesim-protocol

Protocol-v5 DTOs, authority rules, peer discovery helpers, and ROS 2/DDS
transport adapters shared by every Elesim deployment artifact. Endpoint
discovery and RGBD use typed ROS messages. The current control and WebRTC
signaling path uses the bounded `elesim_interfaces/PeerEnvelope` DDS message;
the typed service/action definitions in `packages/elesim_interfaces` are not
yet wired into the runtime.

This package has no ZMQ dependency and contains no robot, simulation, workflow,
or UI implementation. ROS imports remain lazy so release and setup tooling can
inspect the package before a ROS environment is sourced.
