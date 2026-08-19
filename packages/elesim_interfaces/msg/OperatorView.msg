uint16 SCHEMA_VERSION=1
uint32 MAX_STATE_JSON_CHARS=524288
uint32 MAX_SERVICE_JSON_CHARS=524288

uint16 schema_version
elesim_interfaces/PeerRef pilot
uint64 sequence
builtin_interfaces/Time timestamp
string<=524288 state_json
string<=524288 service_json
