# Automatically Generated in 2014-07-24 21:08:34.249980
# MESSAGE: PARAM_VALUE
# Description:Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
uint8 ID = 22
uint8 sysid
uint8 compid
char[16] param_id
float32 param_value
uint8 param_type
uint16 param_count
uint16 param_index
