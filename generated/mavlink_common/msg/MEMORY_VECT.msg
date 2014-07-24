# Automatically Generated in 2014-07-24 21:08:34.251960
# MESSAGE: MEMORY_VECT
# Description:Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
uint8 ID = 249
uint8 sysid
uint8 compid
uint16 address
uint8 ver
uint8 type
int8[32] value
