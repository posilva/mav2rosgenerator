# Automatically Generated in 2014-07-24 21:08:34.251156
# MESSAGE: NAV_CONTROLLER_OUTPUT
# Description:Outputs of the APM navigation controller. The primary use of this message is to check the response and signs of the controller before actual flight and to assist with tuning controller parameters.
uint8 ID = 62
uint8 sysid
uint8 compid
float32 nav_roll
float32 nav_pitch
int16 nav_bearing
int16 target_bearing
uint16 wp_dist
float32 alt_error
float32 aspd_error
float32 xtrack_error
