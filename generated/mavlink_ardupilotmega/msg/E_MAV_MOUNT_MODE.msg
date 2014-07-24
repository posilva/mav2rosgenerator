# Automatically Generated in <function get_timestamp at 0x7ff397db5848>
# ENUM: E_MAV_MOUNT_MODE
# Description:Enumeration of possible mount operation modes
# Load and keep safe position (Roll,Pitch,Yaw) from EEPROM and stop stabilization
uint8 E_MAV_MOUNT_MODE_RETRACT = 0
# Load and keep neutral position (Roll,Pitch,Yaw) from EEPROM.
uint8 E_MAV_MOUNT_MODE_NEUTRAL = 1
# Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
uint8 E_MAV_MOUNT_MODE_MAVLINK_TARGETING = 2
# Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
uint8 E_MAV_MOUNT_MODE_RC_TARGETING = 3
# Load neutral position and start to point to Lat,Lon,Alt
uint8 E_MAV_MOUNT_MODE_GPS_POINT = 4
