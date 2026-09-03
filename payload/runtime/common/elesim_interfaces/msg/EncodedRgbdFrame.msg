# Bounded RGB-D wire sample.  Pixels are encoded payloads, never raw DDS
# sensor_msgs/Image buffers.  The 4 MiB limit keeps one sample bounded before
# RTPS/SROS2 fragmentation and allows a subscriber to reject oversized data.
elesim_interfaces/PeerRef source
uint64 frame_sequence
builtin_interfaces/Time capture_time
uint32 width
uint32 height
string<=16 color_codec
string<=16 color_encoding
uint8[<=4194304] color_data
bool has_depth
string<=16 depth_codec
string<=16 depth_encoding
uint8[<=4194304] depth_data
float32 depth_scale
float64 fx
float64 fy
float64 cx
float64 cy
string<=128 calibration_id
bool has_arm_q
float64[4] arm_q
bool has_camera_pose
float64[3] camera_world_origin
float64[3] camera_world_look
float64[3] camera_world_right
