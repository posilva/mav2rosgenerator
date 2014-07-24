# Automatically Generated in 2014-07-24 21:08:34.250084
# MESSAGE: GPS_STATUS
# Description:The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
uint8 ID = 25
uint8 sysid
uint8 compid
uint8 satellites_visible
uint8[20] satellite_prn
uint8[20] satellite_used
uint8[20] satellite_elevation
uint8[20] satellite_azimuth
uint8[20] satellite_snr
