# Automatically Generated in 2014-07-24 21:08:34.250523
# MESSAGE: MISSION_ITEM
# Description:Message encoding a mission item. This message is emitted to announce
#                the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
uint8 ID = 39
uint8 sysid
uint8 compid
uint8 target_system
uint8 target_component
uint16 seq
uint8 frame
uint16 command
uint8 current
uint8 autocontinue
float32 param1
float32 param2
float32 param3
float32 param4
float32 x
float32 y
float32 z
