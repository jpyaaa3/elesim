uint16 MAX_REQUEST_ID_CHARS=128
uint16 MAX_REASON_CHARS=512

string<=128 request_id
elesim_interfaces/PeerRef target
elesim_interfaces/PeerRef pilot
uint64 lease_epoch
uint64 command_sequence
bool accepted
string<=512 reason
