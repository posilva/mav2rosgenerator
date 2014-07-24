# Automatically Generated in 2014-07-24 21:08:34.250344
# MESSAGE: GLOBAL_POSITION_INT
# Description:The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
#               is designed as scaled integer message since the resolution of float is not sufficient.
uint8 ID = 33
uint8 sysid
uint8 compid
uint32 time_boot_ms
int32 lat
int32 lon
int32 alt
int32 relative_alt
int16 vx
int16 vy
int16 vz
uint16 hdg
