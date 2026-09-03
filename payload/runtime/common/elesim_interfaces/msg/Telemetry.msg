uint32 MAX_PAYLOAD_JSON_CHARS=262144
uint16 MAX_SAFETY_FAULT_CHARS=512

elesim_interfaces/PeerRef source
elesim_interfaces/PeerRef target
uint64 sequence
builtin_interfaces/Time timestamp
uint64 lease_epoch
float64[4] q
bool torque_enabled
string<=512 safety_fault
string<=262144 payload_json
