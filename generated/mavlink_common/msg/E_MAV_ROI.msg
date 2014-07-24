# Automatically Generated in <function get_timestamp at 0x7ff397db5848>
# ENUM: E_MAV_ROI
# Description: The ROI (region of interest) for the vehicle. This can be
#                be used by the vehicle for camera/vehicle attitude alignment (see
#                MAV_CMD_NAV_ROI).
# No region of interest.
uint8 E_MAV_ROI_NONE = 0
# Point toward next MISSION.
uint8 E_MAV_ROI_WPNEXT = 1
# Point toward given MISSION.
uint8 E_MAV_ROI_WPINDEX = 2
# Point toward fixed location.
uint8 E_MAV_ROI_LOCATION = 3
# Point toward of given id.
uint8 E_MAV_ROI_TARGET = 4
