# Automatically Generated in 2014-07-24 21:08:34.249916
# MESSAGE: PARAM_REQUEST_READ
# Description:Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation of QGroundControl and IMU code.
uint8 ID = 20
uint8 sysid
uint8 compid
uint8 target_system
uint8 target_component
char[16] param_id
int16 param_index
