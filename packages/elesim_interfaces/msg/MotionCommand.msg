uint8 SETPOINT_Q=1
uint8 GO2_VELOCITY=2
uint8 DISCRETE_OPERATION=3
uint16 MAX_OPERATION_CHARS=128
uint16 MAX_ARGUMENTS_JSON_CHARS=16384

elesim_interfaces/LeaseFence fence
uint8 kind
builtin_interfaces/Time timestamp
float64[4] q
float64 vx
float64 vy
float64 wz
string<=128 operation
string<=16384 arguments_json
