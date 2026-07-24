uint16 MAX_REQUEST_ID_CHARS=128
uint16 MAX_COMMAND_CHARS=128
uint16 MAX_REASON_CHARS=512

string<=128 request_id
elesim_interfaces/PeerRef simulator
elesim_interfaces/PeerRef ui
uint64 session_epoch
string<=128 command
bool accepted
string<=512 reason
