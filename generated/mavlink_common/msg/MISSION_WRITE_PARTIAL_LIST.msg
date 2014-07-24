# Automatically Generated in 2014-07-24 21:08:34.250496
# MESSAGE: MISSION_WRITE_PARTIAL_LIST
# Description:This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
uint8 ID = 38
uint8 sysid
uint8 compid
uint8 target_system
uint8 target_component
int16 start_index
int16 end_index
