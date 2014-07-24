# Automatically Generated in 2014-07-24 21:08:34.250046
# MESSAGE: GPS_RAW_INT
# Description:The global position, as returned by the Global Positioning System (GPS). This is
#                NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
uint8 ID = 24
uint8 sysid
uint8 compid
uint64 time_usec
uint8 fix_type
int32 lat
int32 lon
int32 alt
uint16 eph
uint16 epv
uint16 vel
uint16 cog
uint8 satellites_visible
