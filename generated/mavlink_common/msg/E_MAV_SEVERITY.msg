# Automatically Generated in <function get_timestamp at 0x7ff397db5848>
# ENUM: E_MAV_SEVERITY
# Description:Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
# System is unusable. This is a "panic" condition.
uint8 E_MAV_SEVERITY_EMERGENCY = 0
# Action should be taken immediately. Indicates error in non-critical systems.
uint8 E_MAV_SEVERITY_ALERT = 1
# Action must be taken immediately. Indicates failure in a primary system.
uint8 E_MAV_SEVERITY_CRITICAL = 2
# Indicates an error in secondary/redundant systems.
uint8 E_MAV_SEVERITY_ERROR = 3
# Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
uint8 E_MAV_SEVERITY_WARNING = 4
# An unusual event has occured, though not an error condition. This should be investigated for the root cause.
uint8 E_MAV_SEVERITY_NOTICE = 5
# Normal operational messages. Useful for logging. No action is required for these messages.
uint8 E_MAV_SEVERITY_INFO = 6
# Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
uint8 E_MAV_SEVERITY_DEBUG = 7
