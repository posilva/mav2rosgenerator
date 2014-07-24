# Automatically Generated in 2014-07-24 21:08:34.250819
# MESSAGE: SET_LOCAL_POSITION_SETPOINT
# Description:Set the setpoint for a local position controller. This is the position in local coordinates the MAV should fly to. This message is sent by the path/MISSION planner to the onboard position controller. As some MAVs have a degree of freedom in yaw (e.g. all helicopters/quadrotors), the desired yaw angle is part of the message.
uint8 ID = 50
uint8 sysid
uint8 compid
uint8 target_system
uint8 target_component
uint8 coordinate_frame
float32 x
float32 y
float32 z
float32 yaw
