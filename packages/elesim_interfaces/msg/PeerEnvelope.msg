# Bounded protocol-v5 carrier for direct peer control and signaling.
# JSON payload schemas are versioned and validated by elesim_protocol.
uint16 PROTOCOL_MAJOR=5
uint16 MAX_MESSAGE_ID_CHARS=128
uint16 MAX_TYPE_CHARS=64
uint16 MAX_PEER_ID_CHARS=128
uint16 MAX_BOOT_ID_CHARS=128
uint16 MAX_LEASE_ID_CHARS=128
uint32 MAX_PAYLOAD_JSON_CHARS=1048576
uint32 MAX_TRACE_JSON_CHARS=16384

uint16 version
string<=128 message_id
string<=64 type
string<=128 source_id
string<=128 source_boot_id
string<=128 target_id
string<=128 target_boot_id
uint64 seq
builtin_interfaces/Time timestamp
string<=128 lease_id
string<=1048576 payload_json
string<=16384 trace_json
