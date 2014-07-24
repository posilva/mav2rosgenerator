# Automatically Generated in 2014-07-24 21:08:34.250928
# MESSAGE: SAFETY_SET_ALLOWED_AREA
# Description:Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/MISSIONs to accept and which to reject. Safety areas are often enforced by national or competition regulations.
uint8 ID = 54
uint8 sysid
uint8 compid
uint8 target_system
uint8 target_component
uint8 frame
float32 p1x
float32 p1y
float32 p1z
float32 p2x
float32 p2y
float32 p2z
