cmake_minimum_required(VERSION 3.8)
project(elesim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EndpointDescriptor.msg"
  "msg/EndpointHeartbeat.msg"
  "msg/EncodedRgbdFrame.msg"
  "msg/LeaseFence.msg"
  "msg/MotionAck.msg"
  "msg/MotionCommand.msg"
  "msg/MotionLease.msg"
  "msg/MotionLeaseEvent.msg"
  "msg/OperatorView.msg"
  "msg/PeerEnvelope.msg"
  "msg/PeerRef.msg"
  "msg/RemoteEstop.msg"
  "msg/RgbdFrame.msg"
  "msg/SimulationResult.msg"
  "msg/SimulationSession.msg"
  "msg/SimulationStatus.msg"
  "msg/StreamDescriptor.msg"
  "msg/Telemetry.msg"
  "msg/TurnCredentials.msg"
  "srv/AcquireMotionLease.srv"
  "srv/CloseSimulationSession.srv"
  "srv/NegotiateWebRtc.srv"
  "srv/OpenSimulationSession.srv"
  "srv/OperatorCommand.srv"
  "srv/ReleaseMotionLease.srv"
  "srv/RenewMotionLease.srv"
  "srv/RenewSimulationSession.srv"
  "srv/SimulationCommand.srv"
  "action/RunOperatorWorkflow.action"
  DEPENDENCIES action_msgs builtin_interfaces sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
