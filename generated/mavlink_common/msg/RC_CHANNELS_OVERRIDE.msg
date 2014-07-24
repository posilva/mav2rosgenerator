# Automatically Generated in 2014-07-24 21:08:34.251327
# MESSAGE: RC_CHANNELS_OVERRIDE
# Description:The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of -1 means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
uint8 ID = 70
uint8 sysid
uint8 compid
uint8 target_system
uint8 target_component
uint16 chan1_raw
uint16 chan2_raw
uint16 chan3_raw
uint16 chan4_raw
uint16 chan5_raw
uint16 chan6_raw
uint16 chan7_raw
uint16 chan8_raw
