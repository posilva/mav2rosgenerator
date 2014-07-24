# Automatically Generated in 2014-07-24 21:08:34.250740
# MESSAGE: MISSION_ACK
# Description:Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
uint8 ID = 47
uint8 sysid
uint8 compid
uint8 target_system
uint8 target_component
uint8 type
