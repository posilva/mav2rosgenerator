# Automatically Generated in <function get_timestamp at 0x7ff397db5848>
# ENUM: E_MAV_CMD
# Description:Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data.
# Navigate to MISSION.
uint8 E_MAV_CMD_NAV_WAYPOINT = 16
# Loiter around this MISSION an unlimited amount of time
uint8 E_MAV_CMD_NAV_LOITER_UNLIM = 17
# Loiter around this MISSION for X turns
uint8 E_MAV_CMD_NAV_LOITER_TURNS = 18
# Loiter around this MISSION for X seconds
uint8 E_MAV_CMD_NAV_LOITER_TIME = 19
# Return to launch location
uint8 E_MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
# Land at location
uint8 E_MAV_CMD_NAV_LAND = 21
# Takeoff from ground / hand
uint8 E_MAV_CMD_NAV_TAKEOFF = 22
# Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
uint8 E_MAV_CMD_NAV_ROI = 80
# Control autonomous path planning on the MAV.
uint8 E_MAV_CMD_NAV_PATHPLANNING = 81
# NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
uint8 E_MAV_CMD_NAV_LAST = 95
# Delay mission state machine.
uint8 E_MAV_CMD_CONDITION_DELAY = 112
# Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
uint8 E_MAV_CMD_CONDITION_CHANGE_ALT = 113
# Delay mission state machine until within desired distance of next NAV point.
uint8 E_MAV_CMD_CONDITION_DISTANCE = 114
# Reach a certain target angle.
uint8 E_MAV_CMD_CONDITION_YAW = 115
# NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
uint8 E_MAV_CMD_CONDITION_LAST = 159
# Set system mode.
uint8 E_MAV_CMD_DO_SET_MODE = 176
# Jump to the desired command in the mission list.  Repeat this action only the specified number of times
uint8 E_MAV_CMD_DO_JUMP = 177
# Change speed and/or throttle set points.
uint8 E_MAV_CMD_DO_CHANGE_SPEED = 178
# Changes the home location either to the current location or a specified location.
uint8 E_MAV_CMD_DO_SET_HOME = 179
# Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
uint8 E_MAV_CMD_DO_SET_PARAMETER = 180
# Set a relay to a condition.
uint8 E_MAV_CMD_DO_SET_RELAY = 181
# Cycle a relay on and off for a desired number of cyles with a desired period.
uint8 E_MAV_CMD_DO_REPEAT_RELAY = 182
# Set a servo to a desired PWM value.
uint8 E_MAV_CMD_DO_SET_SERVO = 183
# Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
uint8 E_MAV_CMD_DO_REPEAT_SERVO = 184
# Control onboard camera system.
uint8 E_MAV_CMD_DO_CONTROL_VIDEO = 200
# NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
uint8 E_MAV_CMD_DO_LAST = 240
# Trigger calibration. This command will be only accepted if in pre-flight mode.
uint8 E_MAV_CMD_PREFLIGHT_CALIBRATION = 241
# Set sensor offsets. This command will be only accepted if in pre-flight mode.
uint8 E_MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242
# Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
uint8 E_MAV_CMD_PREFLIGHT_STORAGE = 245
# Request the reboot or shutdown of system components.
uint8 E_MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
# Hold / continue the current action
uint8 E_MAV_CMD_OVERRIDE_GOTO = 252
# start running a mission
uint16 E_MAV_CMD_MISSION_START = 300
# Arms / Disarms a component
uint16 E_MAV_CMD_COMPONENT_ARM_DISARM = 400
