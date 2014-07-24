# Automatically Generated in <function get_timestamp at 0x7ff397db5848>
# ENUM: E_MAV_STATE
# Uninitialized system, state is unknown.
uint8 E_MAV_STATE_UNINIT = 0
# System is booting up.
uint8 E_MAV_STATE_BOOT = 1
# System is calibrating and not flight-ready.
uint8 E_MAV_STATE_CALIBRATING = 2
# System is grounded and on standby. It can be launched any time.
uint8 E_MAV_STATE_STANDBY = 3
# System is active and might be already airborne. Motors are engaged.
uint8 E_MAV_STATE_ACTIVE = 4
# System is in a non-normal flight mode. It can however still navigate.
uint8 E_MAV_STATE_CRITICAL = 5
# System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
uint8 E_MAV_STATE_EMERGENCY = 6
# System just initialized its power-down sequence, will shut down now.
uint8 E_MAV_STATE_POWEROFF = 7
