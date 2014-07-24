# Automatically Generated in 2014-07-24 21:08:34.249663
# MESSAGE: SYS_STATUS
# Description:The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows wether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
uint8 ID = 1
uint8 sysid
uint8 compid
uint32 onboard_control_sensors_present
uint32 onboard_control_sensors_enabled
uint32 onboard_control_sensors_health
uint16 load
uint16 voltage_battery
int16 current_battery
int8 battery_remaining
uint16 drop_rate_comm
uint16 errors_comm
uint16 errors_count1
uint16 errors_count2
uint16 errors_count3
uint16 errors_count4
