# Automatically Generated in <function get_timestamp at 0x7ff397db5848>
# ENUM: E_MAV_DATA_STREAM
# Description:Data stream IDs. A data stream is not a fixed set of messages, but rather a
#     recommendation to the autopilot software. Individual autopilots may or may not obey
#     the recommended messages.
# Enable all data streams
uint8 E_MAV_DATA_STREAM_ALL = 0
# Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
uint8 E_MAV_DATA_STREAM_RAW_SENSORS = 1
# Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
uint8 E_MAV_DATA_STREAM_EXTENDED_STATUS = 2
# Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
uint8 E_MAV_DATA_STREAM_RC_CHANNELS = 3
# Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
uint8 E_MAV_DATA_STREAM_RAW_CONTROLLER = 4
# Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
uint8 E_MAV_DATA_STREAM_POSITION = 6
# Dependent on the autopilot
uint8 E_MAV_DATA_STREAM_EXTRA1 = 10
# Dependent on the autopilot
uint8 E_MAV_DATA_STREAM_EXTRA2 = 11
# Dependent on the autopilot
uint8 E_MAV_DATA_STREAM_EXTRA3 = 12
