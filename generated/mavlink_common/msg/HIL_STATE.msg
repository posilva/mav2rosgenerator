# Automatically Generated in 2014-07-24 21:08:34.251525
# MESSAGE: HIL_STATE
# Description:Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
uint8 ID = 90
uint8 sysid
uint8 compid
uint64 time_usec
float32 roll
float32 pitch
float32 yaw
float32 rollspeed
float32 pitchspeed
float32 yawspeed
int32 lat
int32 lon
int32 alt
int16 vx
int16 vy
int16 vz
int16 xacc
int16 yacc
int16 zacc
