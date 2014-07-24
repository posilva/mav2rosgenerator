# Automatically Generated in <function get_timestamp at 0x7ff397db5848>
# ENUM: E_MAV_CMD_ACK
# Description:ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
# Command / mission item is ok.
uint8 E_MAV_CMD_ACK_OK = 1
# Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
uint8 E_MAV_CMD_ACK_ERR_FAIL = 2
# The system is refusing to accept this command from this source / communication partner.
uint8 E_MAV_CMD_ACK_ERR_ACCESS_DENIED = 3
# Command or mission item is not supported, other commands would be accepted.
uint8 E_MAV_CMD_ACK_ERR_NOT_SUPPORTED = 4
# The coordinate frame of this command / mission item is not supported.
uint8 E_MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED = 5
# The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
uint8 E_MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE = 6
# The X or latitude value is out of range.
uint8 E_MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE = 7
# The Y or longitude value is out of range.
uint8 E_MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE = 8
# The Z or altitude value is out of range.
uint8 E_MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE = 9
