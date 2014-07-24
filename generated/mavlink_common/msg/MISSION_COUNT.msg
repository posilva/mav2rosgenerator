# Automatically Generated in 2014-07-24 21:08:34.250664
# MESSAGE: MISSION_COUNT
# Description:This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
uint8 ID = 44
uint8 sysid
uint8 compid
uint8 target_system
uint8 target_component
uint16 count
