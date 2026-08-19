uint16 PROTOCOL_MAJOR=6
uint16 MAX_ROLE_CHARS=32
uint16 MAX_CAPABILITIES=32
uint16 MAX_CAPABILITY_CHARS=128
uint16 MAX_STREAMS=16
uint16 MAX_PREFIX_CHARS=256
uint16 MAX_INTERFACE_HASH_CHARS=128

uint16 protocol_major
elesim_interfaces/PeerRef peer
string<=32 role
string<=128[<=32] capabilities
elesim_interfaces/StreamDescriptor[<=16] streams
uint64 descriptor_revision
string<=256 service_prefix
string<=256 topic_prefix
string<=128 interface_hash
