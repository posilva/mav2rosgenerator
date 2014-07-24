
/**
 * 
 **/
#include <ros/ros.h>
#include <mavlink_ardupilotmega/ardupilotmega/mavlink.h>
#include <mavlink_ardupilotmega/mavlink2ros.h>

mavlink_message_t mav_msg;                  //! Global mavlink message 
mavlink_status_t status;                //! Global mavlink status    

ros::Publisher  to_mav_raw_publisher;        //! ROS publisher to write to mavlink interface
ros::Subscriber from_mav_raw_subscriber;     //! ROS subscriber to read from mavlink interface

	ros::Publisher from_mav_heartbeat_pub;
	ros::Publisher from_mav_sys_status_pub;
	ros::Publisher from_mav_system_time_pub;
	ros::Publisher from_mav_ping_pub;
	ros::Publisher from_mav_change_operator_control_pub;
	ros::Publisher from_mav_change_operator_control_ack_pub;
	ros::Publisher from_mav_auth_key_pub;
	ros::Publisher from_mav_set_mode_pub;
	ros::Publisher from_mav_param_request_read_pub;
	ros::Publisher from_mav_param_request_list_pub;
	ros::Publisher from_mav_param_value_pub;
	ros::Publisher from_mav_param_set_pub;
	ros::Publisher from_mav_gps_raw_int_pub;
	ros::Publisher from_mav_gps_status_pub;
	ros::Publisher from_mav_scaled_imu_pub;
	ros::Publisher from_mav_raw_imu_pub;
	ros::Publisher from_mav_raw_pressure_pub;
	ros::Publisher from_mav_scaled_pressure_pub;
	ros::Publisher from_mav_attitude_pub;
	ros::Publisher from_mav_attitude_quaternion_pub;
	ros::Publisher from_mav_local_position_ned_pub;
	ros::Publisher from_mav_global_position_int_pub;
	ros::Publisher from_mav_rc_channels_scaled_pub;
	ros::Publisher from_mav_rc_channels_raw_pub;
	ros::Publisher from_mav_servo_output_raw_pub;
	ros::Publisher from_mav_mission_request_partial_list_pub;
	ros::Publisher from_mav_mission_write_partial_list_pub;
	ros::Publisher from_mav_mission_item_pub;
	ros::Publisher from_mav_mission_request_pub;
	ros::Publisher from_mav_mission_set_current_pub;
	ros::Publisher from_mav_mission_current_pub;
	ros::Publisher from_mav_mission_request_list_pub;
	ros::Publisher from_mav_mission_count_pub;
	ros::Publisher from_mav_mission_clear_all_pub;
	ros::Publisher from_mav_mission_item_reached_pub;
	ros::Publisher from_mav_mission_ack_pub;
	ros::Publisher from_mav_set_gps_global_origin_pub;
	ros::Publisher from_mav_gps_global_origin_pub;
	ros::Publisher from_mav_set_local_position_setpoint_pub;
	ros::Publisher from_mav_local_position_setpoint_pub;
	ros::Publisher from_mav_global_position_setpoint_int_pub;
	ros::Publisher from_mav_set_global_position_setpoint_int_pub;
	ros::Publisher from_mav_safety_set_allowed_area_pub;
	ros::Publisher from_mav_safety_allowed_area_pub;
	ros::Publisher from_mav_set_roll_pitch_yaw_thrust_pub;
	ros::Publisher from_mav_set_roll_pitch_yaw_speed_thrust_pub;
	ros::Publisher from_mav_roll_pitch_yaw_thrust_setpoint_pub;
	ros::Publisher from_mav_roll_pitch_yaw_speed_thrust_setpoint_pub;
	ros::Publisher from_mav_set_quad_motors_setpoint_pub;
	ros::Publisher from_mav_set_quad_swarm_roll_pitch_yaw_thrust_pub;
	ros::Publisher from_mav_nav_controller_output_pub;
	ros::Publisher from_mav_set_quad_swarm_led_roll_pitch_yaw_thrust_pub;
	ros::Publisher from_mav_state_correction_pub;
	ros::Publisher from_mav_request_data_stream_pub;
	ros::Publisher from_mav_data_stream_pub;
	ros::Publisher from_mav_manual_control_pub;
	ros::Publisher from_mav_rc_channels_override_pub;
	ros::Publisher from_mav_vfr_hud_pub;
	ros::Publisher from_mav_command_long_pub;
	ros::Publisher from_mav_command_ack_pub;
	ros::Publisher from_mav_roll_pitch_yaw_rates_thrust_setpoint_pub;
	ros::Publisher from_mav_manual_setpoint_pub;
	ros::Publisher from_mav_local_position_ned_system_global_offset_pub;
	ros::Publisher from_mav_hil_state_pub;
	ros::Publisher from_mav_hil_controls_pub;
	ros::Publisher from_mav_hil_rc_inputs_raw_pub;
	ros::Publisher from_mav_optical_flow_pub;
	ros::Publisher from_mav_global_vision_position_estimate_pub;
	ros::Publisher from_mav_vision_position_estimate_pub;
	ros::Publisher from_mav_vision_speed_estimate_pub;
	ros::Publisher from_mav_vicon_position_estimate_pub;
	ros::Publisher from_mav_highres_imu_pub;
	ros::Publisher from_mav_file_transfer_start_pub;
	ros::Publisher from_mav_file_transfer_dir_list_pub;
	ros::Publisher from_mav_file_transfer_res_pub;
	ros::Publisher from_mav_battery_status_pub;
	ros::Publisher from_mav_setpoint_8dof_pub;
	ros::Publisher from_mav_setpoint_6dof_pub;
	ros::Publisher from_mav_memory_vect_pub;
	ros::Publisher from_mav_debug_vect_pub;
	ros::Publisher from_mav_named_value_float_pub;
	ros::Publisher from_mav_named_value_int_pub;
	ros::Publisher from_mav_statustext_pub;
	ros::Publisher from_mav_debug_pub;
	ros::Publisher from_mav_sensor_offsets_pub;
	ros::Publisher from_mav_set_mag_offsets_pub;
	ros::Publisher from_mav_meminfo_pub;
	ros::Publisher from_mav_ap_adc_pub;
	ros::Publisher from_mav_digicam_configure_pub;
	ros::Publisher from_mav_digicam_control_pub;
	ros::Publisher from_mav_mount_configure_pub;
	ros::Publisher from_mav_mount_control_pub;
	ros::Publisher from_mav_mount_status_pub;
	ros::Publisher from_mav_fence_point_pub;
	ros::Publisher from_mav_fence_fetch_point_pub;
	ros::Publisher from_mav_fence_status_pub;
	ros::Publisher from_mav_ahrs_pub;
	ros::Publisher from_mav_simstate_pub;
	ros::Publisher from_mav_hwstatus_pub;
	ros::Publisher from_mav_radio_pub;
	ros::Publisher from_mav_limits_status_pub;
	ros::Publisher from_mav_wind_pub;
	ros::Publisher from_mav_data16_pub;
	ros::Publisher from_mav_data32_pub;
	ros::Publisher from_mav_data64_pub;
	ros::Publisher from_mav_data96_pub;


/**
 * 
 **/
int write_to_mav (uint8_t * b, int sz){

    mavlink_ardupilotmega::MAV_RAW m;
    
    m.channel=mavlink_ardupilotmega::MAV_RAW::CH_COMM0;
    m.data.assign(b, b+sz);
    int rc = m.data.size();
    to_mav_raw_publisher.publish(m);
    ROS_INFO("Writen to MAV %d bytes",rc);
    return rc;
}        


/**
 *
 */            
void to_mav_heartbeat_callback(const mavlink_common::HEARTBEAT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_HEARTBEAT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_heartbeat_t heartbeat_out;
    
    /** ASSIGN FIELDS **/
    
	heartbeat_out.type = msg->type;
	heartbeat_out.autopilot = msg->autopilot;
	heartbeat_out.base_mode = msg->base_mode;
	heartbeat_out.custom_mode = msg->custom_mode;
	heartbeat_out.system_status = msg->system_status;
	heartbeat_out.mavlink_version = msg->mavlink_version;

    
    mavlink_msg_heartbeat_encode(msg->sysid, msg->compid, &m, &heartbeat_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_sys_status_callback(const mavlink_common::SYS_STATUS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SYS_STATUS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_sys_status_t sys_status_out;
    
    /** ASSIGN FIELDS **/
    
	sys_status_out.onboard_control_sensors_present = msg->onboard_control_sensors_present;
	sys_status_out.onboard_control_sensors_enabled = msg->onboard_control_sensors_enabled;
	sys_status_out.onboard_control_sensors_health = msg->onboard_control_sensors_health;
	sys_status_out.load = msg->load;
	sys_status_out.voltage_battery = msg->voltage_battery;
	sys_status_out.current_battery = msg->current_battery;
	sys_status_out.battery_remaining = msg->battery_remaining;
	sys_status_out.drop_rate_comm = msg->drop_rate_comm;
	sys_status_out.errors_comm = msg->errors_comm;
	sys_status_out.errors_count1 = msg->errors_count1;
	sys_status_out.errors_count2 = msg->errors_count2;
	sys_status_out.errors_count3 = msg->errors_count3;
	sys_status_out.errors_count4 = msg->errors_count4;

    
    mavlink_msg_sys_status_encode(msg->sysid, msg->compid, &m, &sys_status_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_system_time_callback(const mavlink_common::SYSTEM_TIME::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SYSTEM_TIME request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_system_time_t system_time_out;
    
    /** ASSIGN FIELDS **/
    
	system_time_out.time_unix_usec = msg->time_unix_usec;
	system_time_out.time_boot_ms = msg->time_boot_ms;

    
    mavlink_msg_system_time_encode(msg->sysid, msg->compid, &m, &system_time_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_ping_callback(const mavlink_common::PING::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_PING request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_ping_t ping_out;
    
    /** ASSIGN FIELDS **/
    
	ping_out.time_usec = msg->time_usec;
	ping_out.seq = msg->seq;
	ping_out.target_system = msg->target_system;
	ping_out.target_component = msg->target_component;

    
    mavlink_msg_ping_encode(msg->sysid, msg->compid, &m, &ping_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_change_operator_control_callback(const mavlink_common::CHANGE_OPERATOR_CONTROL::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_CHANGE_OPERATOR_CONTROL request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_change_operator_control_t change_operator_control_out;
    
    /** ASSIGN FIELDS **/
    
	change_operator_control_out.target_system = msg->target_system;
	change_operator_control_out.control_request = msg->control_request;
	change_operator_control_out.version = msg->version;
	memcpy(&(change_operator_control_out.passkey), &(msg->passkey[0]), sizeof(char)* (int)(msg->passkey.size()));

    
    mavlink_msg_change_operator_control_encode(msg->sysid, msg->compid, &m, &change_operator_control_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_change_operator_control_ack_callback(const mavlink_common::CHANGE_OPERATOR_CONTROL_ACK::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_CHANGE_OPERATOR_CONTROL_ACK request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_change_operator_control_ack_t change_operator_control_ack_out;
    
    /** ASSIGN FIELDS **/
    
	change_operator_control_ack_out.gcs_system_id = msg->gcs_system_id;
	change_operator_control_ack_out.control_request = msg->control_request;
	change_operator_control_ack_out.ack = msg->ack;

    
    mavlink_msg_change_operator_control_ack_encode(msg->sysid, msg->compid, &m, &change_operator_control_ack_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_auth_key_callback(const mavlink_common::AUTH_KEY::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_AUTH_KEY request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_auth_key_t auth_key_out;
    
    /** ASSIGN FIELDS **/
    
	memcpy(&(auth_key_out.key), &(msg->key[0]), sizeof(char)* (int)(msg->key.size()));

    
    mavlink_msg_auth_key_encode(msg->sysid, msg->compid, &m, &auth_key_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_set_mode_callback(const mavlink_common::SET_MODE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SET_MODE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_set_mode_t set_mode_out;
    
    /** ASSIGN FIELDS **/
    
	set_mode_out.target_system = msg->target_system;
	set_mode_out.base_mode = msg->base_mode;
	set_mode_out.custom_mode = msg->custom_mode;

    
    mavlink_msg_set_mode_encode(msg->sysid, msg->compid, &m, &set_mode_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_param_request_read_callback(const mavlink_common::PARAM_REQUEST_READ::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_PARAM_REQUEST_READ request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_param_request_read_t param_request_read_out;
    
    /** ASSIGN FIELDS **/
    
	param_request_read_out.target_system = msg->target_system;
	param_request_read_out.target_component = msg->target_component;
	memcpy(&(param_request_read_out.param_id), &(msg->param_id[0]), sizeof(char)* (int)(msg->param_id.size()));
	param_request_read_out.param_index = msg->param_index;

    
    mavlink_msg_param_request_read_encode(msg->sysid, msg->compid, &m, &param_request_read_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_param_request_list_callback(const mavlink_common::PARAM_REQUEST_LIST::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_PARAM_REQUEST_LIST request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_param_request_list_t param_request_list_out;
    
    /** ASSIGN FIELDS **/
    
	param_request_list_out.target_system = msg->target_system;
	param_request_list_out.target_component = msg->target_component;

    
    mavlink_msg_param_request_list_encode(msg->sysid, msg->compid, &m, &param_request_list_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_param_value_callback(const mavlink_common::PARAM_VALUE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_PARAM_VALUE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_param_value_t param_value_out;
    
    /** ASSIGN FIELDS **/
    
	memcpy(&(param_value_out.param_id), &(msg->param_id[0]), sizeof(char)* (int)(msg->param_id.size()));
	param_value_out.param_value = msg->param_value;
	param_value_out.param_type = msg->param_type;
	param_value_out.param_count = msg->param_count;
	param_value_out.param_index = msg->param_index;

    
    mavlink_msg_param_value_encode(msg->sysid, msg->compid, &m, &param_value_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_param_set_callback(const mavlink_common::PARAM_SET::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_PARAM_SET request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_param_set_t param_set_out;
    
    /** ASSIGN FIELDS **/
    
	param_set_out.target_system = msg->target_system;
	param_set_out.target_component = msg->target_component;
	memcpy(&(param_set_out.param_id), &(msg->param_id[0]), sizeof(char)* (int)(msg->param_id.size()));
	param_set_out.param_value = msg->param_value;
	param_set_out.param_type = msg->param_type;

    
    mavlink_msg_param_set_encode(msg->sysid, msg->compid, &m, &param_set_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_gps_raw_int_callback(const mavlink_common::GPS_RAW_INT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_GPS_RAW_INT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_gps_raw_int_t gps_raw_int_out;
    
    /** ASSIGN FIELDS **/
    
	gps_raw_int_out.time_usec = msg->time_usec;
	gps_raw_int_out.fix_type = msg->fix_type;
	gps_raw_int_out.lat = msg->lat;
	gps_raw_int_out.lon = msg->lon;
	gps_raw_int_out.alt = msg->alt;
	gps_raw_int_out.eph = msg->eph;
	gps_raw_int_out.epv = msg->epv;
	gps_raw_int_out.vel = msg->vel;
	gps_raw_int_out.cog = msg->cog;
	gps_raw_int_out.satellites_visible = msg->satellites_visible;

    
    mavlink_msg_gps_raw_int_encode(msg->sysid, msg->compid, &m, &gps_raw_int_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_gps_status_callback(const mavlink_common::GPS_STATUS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_GPS_STATUS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_gps_status_t gps_status_out;
    
    /** ASSIGN FIELDS **/
    
	gps_status_out.satellites_visible = msg->satellites_visible;
	memcpy(&(gps_status_out.satellite_prn), &(msg->satellite_prn[0]), sizeof(uint8_t)* (int)(msg->satellite_prn.size()));
	memcpy(&(gps_status_out.satellite_used), &(msg->satellite_used[0]), sizeof(uint8_t)* (int)(msg->satellite_used.size()));
	memcpy(&(gps_status_out.satellite_elevation), &(msg->satellite_elevation[0]), sizeof(uint8_t)* (int)(msg->satellite_elevation.size()));
	memcpy(&(gps_status_out.satellite_azimuth), &(msg->satellite_azimuth[0]), sizeof(uint8_t)* (int)(msg->satellite_azimuth.size()));
	memcpy(&(gps_status_out.satellite_snr), &(msg->satellite_snr[0]), sizeof(uint8_t)* (int)(msg->satellite_snr.size()));

    
    mavlink_msg_gps_status_encode(msg->sysid, msg->compid, &m, &gps_status_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_scaled_imu_callback(const mavlink_common::SCALED_IMU::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SCALED_IMU request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_scaled_imu_t scaled_imu_out;
    
    /** ASSIGN FIELDS **/
    
	scaled_imu_out.time_boot_ms = msg->time_boot_ms;
	scaled_imu_out.xacc = msg->xacc;
	scaled_imu_out.yacc = msg->yacc;
	scaled_imu_out.zacc = msg->zacc;
	scaled_imu_out.xgyro = msg->xgyro;
	scaled_imu_out.ygyro = msg->ygyro;
	scaled_imu_out.zgyro = msg->zgyro;
	scaled_imu_out.xmag = msg->xmag;
	scaled_imu_out.ymag = msg->ymag;
	scaled_imu_out.zmag = msg->zmag;

    
    mavlink_msg_scaled_imu_encode(msg->sysid, msg->compid, &m, &scaled_imu_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_raw_imu_callback(const mavlink_common::RAW_IMU::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_RAW_IMU request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_raw_imu_t raw_imu_out;
    
    /** ASSIGN FIELDS **/
    
	raw_imu_out.time_usec = msg->time_usec;
	raw_imu_out.xacc = msg->xacc;
	raw_imu_out.yacc = msg->yacc;
	raw_imu_out.zacc = msg->zacc;
	raw_imu_out.xgyro = msg->xgyro;
	raw_imu_out.ygyro = msg->ygyro;
	raw_imu_out.zgyro = msg->zgyro;
	raw_imu_out.xmag = msg->xmag;
	raw_imu_out.ymag = msg->ymag;
	raw_imu_out.zmag = msg->zmag;

    
    mavlink_msg_raw_imu_encode(msg->sysid, msg->compid, &m, &raw_imu_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_raw_pressure_callback(const mavlink_common::RAW_PRESSURE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_RAW_PRESSURE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_raw_pressure_t raw_pressure_out;
    
    /** ASSIGN FIELDS **/
    
	raw_pressure_out.time_usec = msg->time_usec;
	raw_pressure_out.press_abs = msg->press_abs;
	raw_pressure_out.press_diff1 = msg->press_diff1;
	raw_pressure_out.press_diff2 = msg->press_diff2;
	raw_pressure_out.temperature = msg->temperature;

    
    mavlink_msg_raw_pressure_encode(msg->sysid, msg->compid, &m, &raw_pressure_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_scaled_pressure_callback(const mavlink_common::SCALED_PRESSURE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SCALED_PRESSURE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_scaled_pressure_t scaled_pressure_out;
    
    /** ASSIGN FIELDS **/
    
	scaled_pressure_out.time_boot_ms = msg->time_boot_ms;
	scaled_pressure_out.press_abs = msg->press_abs;
	scaled_pressure_out.press_diff = msg->press_diff;
	scaled_pressure_out.temperature = msg->temperature;

    
    mavlink_msg_scaled_pressure_encode(msg->sysid, msg->compid, &m, &scaled_pressure_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_attitude_callback(const mavlink_common::ATTITUDE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_ATTITUDE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_attitude_t attitude_out;
    
    /** ASSIGN FIELDS **/
    
	attitude_out.time_boot_ms = msg->time_boot_ms;
	attitude_out.roll = msg->roll;
	attitude_out.pitch = msg->pitch;
	attitude_out.yaw = msg->yaw;
	attitude_out.rollspeed = msg->rollspeed;
	attitude_out.pitchspeed = msg->pitchspeed;
	attitude_out.yawspeed = msg->yawspeed;

    
    mavlink_msg_attitude_encode(msg->sysid, msg->compid, &m, &attitude_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_attitude_quaternion_callback(const mavlink_common::ATTITUDE_QUATERNION::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_ATTITUDE_QUATERNION request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_attitude_quaternion_t attitude_quaternion_out;
    
    /** ASSIGN FIELDS **/
    
	attitude_quaternion_out.time_boot_ms = msg->time_boot_ms;
	attitude_quaternion_out.q1 = msg->q1;
	attitude_quaternion_out.q2 = msg->q2;
	attitude_quaternion_out.q3 = msg->q3;
	attitude_quaternion_out.q4 = msg->q4;
	attitude_quaternion_out.rollspeed = msg->rollspeed;
	attitude_quaternion_out.pitchspeed = msg->pitchspeed;
	attitude_quaternion_out.yawspeed = msg->yawspeed;

    
    mavlink_msg_attitude_quaternion_encode(msg->sysid, msg->compid, &m, &attitude_quaternion_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_local_position_ned_callback(const mavlink_common::LOCAL_POSITION_NED::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_LOCAL_POSITION_NED request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_local_position_ned_t local_position_ned_out;
    
    /** ASSIGN FIELDS **/
    
	local_position_ned_out.time_boot_ms = msg->time_boot_ms;
	local_position_ned_out.x = msg->x;
	local_position_ned_out.y = msg->y;
	local_position_ned_out.z = msg->z;
	local_position_ned_out.vx = msg->vx;
	local_position_ned_out.vy = msg->vy;
	local_position_ned_out.vz = msg->vz;

    
    mavlink_msg_local_position_ned_encode(msg->sysid, msg->compid, &m, &local_position_ned_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_global_position_int_callback(const mavlink_common::GLOBAL_POSITION_INT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_GLOBAL_POSITION_INT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_global_position_int_t global_position_int_out;
    
    /** ASSIGN FIELDS **/
    
	global_position_int_out.time_boot_ms = msg->time_boot_ms;
	global_position_int_out.lat = msg->lat;
	global_position_int_out.lon = msg->lon;
	global_position_int_out.alt = msg->alt;
	global_position_int_out.relative_alt = msg->relative_alt;
	global_position_int_out.vx = msg->vx;
	global_position_int_out.vy = msg->vy;
	global_position_int_out.vz = msg->vz;
	global_position_int_out.hdg = msg->hdg;

    
    mavlink_msg_global_position_int_encode(msg->sysid, msg->compid, &m, &global_position_int_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_rc_channels_scaled_callback(const mavlink_common::RC_CHANNELS_SCALED::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_RC_CHANNELS_SCALED request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_rc_channels_scaled_t rc_channels_scaled_out;
    
    /** ASSIGN FIELDS **/
    
	rc_channels_scaled_out.time_boot_ms = msg->time_boot_ms;
	rc_channels_scaled_out.port = msg->port;
	rc_channels_scaled_out.chan1_scaled = msg->chan1_scaled;
	rc_channels_scaled_out.chan2_scaled = msg->chan2_scaled;
	rc_channels_scaled_out.chan3_scaled = msg->chan3_scaled;
	rc_channels_scaled_out.chan4_scaled = msg->chan4_scaled;
	rc_channels_scaled_out.chan5_scaled = msg->chan5_scaled;
	rc_channels_scaled_out.chan6_scaled = msg->chan6_scaled;
	rc_channels_scaled_out.chan7_scaled = msg->chan7_scaled;
	rc_channels_scaled_out.chan8_scaled = msg->chan8_scaled;
	rc_channels_scaled_out.rssi = msg->rssi;

    
    mavlink_msg_rc_channels_scaled_encode(msg->sysid, msg->compid, &m, &rc_channels_scaled_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_rc_channels_raw_callback(const mavlink_common::RC_CHANNELS_RAW::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_RC_CHANNELS_RAW request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_rc_channels_raw_t rc_channels_raw_out;
    
    /** ASSIGN FIELDS **/
    
	rc_channels_raw_out.time_boot_ms = msg->time_boot_ms;
	rc_channels_raw_out.port = msg->port;
	rc_channels_raw_out.chan1_raw = msg->chan1_raw;
	rc_channels_raw_out.chan2_raw = msg->chan2_raw;
	rc_channels_raw_out.chan3_raw = msg->chan3_raw;
	rc_channels_raw_out.chan4_raw = msg->chan4_raw;
	rc_channels_raw_out.chan5_raw = msg->chan5_raw;
	rc_channels_raw_out.chan6_raw = msg->chan6_raw;
	rc_channels_raw_out.chan7_raw = msg->chan7_raw;
	rc_channels_raw_out.chan8_raw = msg->chan8_raw;
	rc_channels_raw_out.rssi = msg->rssi;

    
    mavlink_msg_rc_channels_raw_encode(msg->sysid, msg->compid, &m, &rc_channels_raw_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_servo_output_raw_callback(const mavlink_common::SERVO_OUTPUT_RAW::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SERVO_OUTPUT_RAW request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_servo_output_raw_t servo_output_raw_out;
    
    /** ASSIGN FIELDS **/
    
	servo_output_raw_out.time_usec = msg->time_usec;
	servo_output_raw_out.port = msg->port;
	servo_output_raw_out.servo1_raw = msg->servo1_raw;
	servo_output_raw_out.servo2_raw = msg->servo2_raw;
	servo_output_raw_out.servo3_raw = msg->servo3_raw;
	servo_output_raw_out.servo4_raw = msg->servo4_raw;
	servo_output_raw_out.servo5_raw = msg->servo5_raw;
	servo_output_raw_out.servo6_raw = msg->servo6_raw;
	servo_output_raw_out.servo7_raw = msg->servo7_raw;
	servo_output_raw_out.servo8_raw = msg->servo8_raw;

    
    mavlink_msg_servo_output_raw_encode(msg->sysid, msg->compid, &m, &servo_output_raw_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_request_partial_list_callback(const mavlink_common::MISSION_REQUEST_PARTIAL_LIST::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_REQUEST_PARTIAL_LIST request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_request_partial_list_t mission_request_partial_list_out;
    
    /** ASSIGN FIELDS **/
    
	mission_request_partial_list_out.target_system = msg->target_system;
	mission_request_partial_list_out.target_component = msg->target_component;
	mission_request_partial_list_out.start_index = msg->start_index;
	mission_request_partial_list_out.end_index = msg->end_index;

    
    mavlink_msg_mission_request_partial_list_encode(msg->sysid, msg->compid, &m, &mission_request_partial_list_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_write_partial_list_callback(const mavlink_common::MISSION_WRITE_PARTIAL_LIST::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_WRITE_PARTIAL_LIST request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_write_partial_list_t mission_write_partial_list_out;
    
    /** ASSIGN FIELDS **/
    
	mission_write_partial_list_out.target_system = msg->target_system;
	mission_write_partial_list_out.target_component = msg->target_component;
	mission_write_partial_list_out.start_index = msg->start_index;
	mission_write_partial_list_out.end_index = msg->end_index;

    
    mavlink_msg_mission_write_partial_list_encode(msg->sysid, msg->compid, &m, &mission_write_partial_list_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_item_callback(const mavlink_common::MISSION_ITEM::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_ITEM request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_item_t mission_item_out;
    
    /** ASSIGN FIELDS **/
    
	mission_item_out.target_system = msg->target_system;
	mission_item_out.target_component = msg->target_component;
	mission_item_out.seq = msg->seq;
	mission_item_out.frame = msg->frame;
	mission_item_out.command = msg->command;
	mission_item_out.current = msg->current;
	mission_item_out.autocontinue = msg->autocontinue;
	mission_item_out.param1 = msg->param1;
	mission_item_out.param2 = msg->param2;
	mission_item_out.param3 = msg->param3;
	mission_item_out.param4 = msg->param4;
	mission_item_out.x = msg->x;
	mission_item_out.y = msg->y;
	mission_item_out.z = msg->z;

    
    mavlink_msg_mission_item_encode(msg->sysid, msg->compid, &m, &mission_item_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_request_callback(const mavlink_common::MISSION_REQUEST::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_REQUEST request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_request_t mission_request_out;
    
    /** ASSIGN FIELDS **/
    
	mission_request_out.target_system = msg->target_system;
	mission_request_out.target_component = msg->target_component;
	mission_request_out.seq = msg->seq;

    
    mavlink_msg_mission_request_encode(msg->sysid, msg->compid, &m, &mission_request_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_set_current_callback(const mavlink_common::MISSION_SET_CURRENT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_SET_CURRENT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_set_current_t mission_set_current_out;
    
    /** ASSIGN FIELDS **/
    
	mission_set_current_out.target_system = msg->target_system;
	mission_set_current_out.target_component = msg->target_component;
	mission_set_current_out.seq = msg->seq;

    
    mavlink_msg_mission_set_current_encode(msg->sysid, msg->compid, &m, &mission_set_current_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_current_callback(const mavlink_common::MISSION_CURRENT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_CURRENT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_current_t mission_current_out;
    
    /** ASSIGN FIELDS **/
    
	mission_current_out.seq = msg->seq;

    
    mavlink_msg_mission_current_encode(msg->sysid, msg->compid, &m, &mission_current_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_request_list_callback(const mavlink_common::MISSION_REQUEST_LIST::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_REQUEST_LIST request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_request_list_t mission_request_list_out;
    
    /** ASSIGN FIELDS **/
    
	mission_request_list_out.target_system = msg->target_system;
	mission_request_list_out.target_component = msg->target_component;

    
    mavlink_msg_mission_request_list_encode(msg->sysid, msg->compid, &m, &mission_request_list_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_count_callback(const mavlink_common::MISSION_COUNT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_COUNT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_count_t mission_count_out;
    
    /** ASSIGN FIELDS **/
    
	mission_count_out.target_system = msg->target_system;
	mission_count_out.target_component = msg->target_component;
	mission_count_out.count = msg->count;

    
    mavlink_msg_mission_count_encode(msg->sysid, msg->compid, &m, &mission_count_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_clear_all_callback(const mavlink_common::MISSION_CLEAR_ALL::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_CLEAR_ALL request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_clear_all_t mission_clear_all_out;
    
    /** ASSIGN FIELDS **/
    
	mission_clear_all_out.target_system = msg->target_system;
	mission_clear_all_out.target_component = msg->target_component;

    
    mavlink_msg_mission_clear_all_encode(msg->sysid, msg->compid, &m, &mission_clear_all_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_item_reached_callback(const mavlink_common::MISSION_ITEM_REACHED::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_ITEM_REACHED request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_item_reached_t mission_item_reached_out;
    
    /** ASSIGN FIELDS **/
    
	mission_item_reached_out.seq = msg->seq;

    
    mavlink_msg_mission_item_reached_encode(msg->sysid, msg->compid, &m, &mission_item_reached_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mission_ack_callback(const mavlink_common::MISSION_ACK::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MISSION_ACK request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mission_ack_t mission_ack_out;
    
    /** ASSIGN FIELDS **/
    
	mission_ack_out.target_system = msg->target_system;
	mission_ack_out.target_component = msg->target_component;
	mission_ack_out.type = msg->type;

    
    mavlink_msg_mission_ack_encode(msg->sysid, msg->compid, &m, &mission_ack_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_set_gps_global_origin_callback(const mavlink_common::SET_GPS_GLOBAL_ORIGIN::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SET_GPS_GLOBAL_ORIGIN request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_set_gps_global_origin_t set_gps_global_origin_out;
    
    /** ASSIGN FIELDS **/
    
	set_gps_global_origin_out.target_system = msg->target_system;
	set_gps_global_origin_out.latitude = msg->latitude;
	set_gps_global_origin_out.longitude = msg->longitude;
	set_gps_global_origin_out.altitude = msg->altitude;

    
    mavlink_msg_set_gps_global_origin_encode(msg->sysid, msg->compid, &m, &set_gps_global_origin_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_gps_global_origin_callback(const mavlink_common::GPS_GLOBAL_ORIGIN::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_GPS_GLOBAL_ORIGIN request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_gps_global_origin_t gps_global_origin_out;
    
    /** ASSIGN FIELDS **/
    
	gps_global_origin_out.latitude = msg->latitude;
	gps_global_origin_out.longitude = msg->longitude;
	gps_global_origin_out.altitude = msg->altitude;

    
    mavlink_msg_gps_global_origin_encode(msg->sysid, msg->compid, &m, &gps_global_origin_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_set_local_position_setpoint_callback(const mavlink_common::SET_LOCAL_POSITION_SETPOINT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SET_LOCAL_POSITION_SETPOINT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_set_local_position_setpoint_t set_local_position_setpoint_out;
    
    /** ASSIGN FIELDS **/
    
	set_local_position_setpoint_out.target_system = msg->target_system;
	set_local_position_setpoint_out.target_component = msg->target_component;
	set_local_position_setpoint_out.coordinate_frame = msg->coordinate_frame;
	set_local_position_setpoint_out.x = msg->x;
	set_local_position_setpoint_out.y = msg->y;
	set_local_position_setpoint_out.z = msg->z;
	set_local_position_setpoint_out.yaw = msg->yaw;

    
    mavlink_msg_set_local_position_setpoint_encode(msg->sysid, msg->compid, &m, &set_local_position_setpoint_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_local_position_setpoint_callback(const mavlink_common::LOCAL_POSITION_SETPOINT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_LOCAL_POSITION_SETPOINT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_local_position_setpoint_t local_position_setpoint_out;
    
    /** ASSIGN FIELDS **/
    
	local_position_setpoint_out.coordinate_frame = msg->coordinate_frame;
	local_position_setpoint_out.x = msg->x;
	local_position_setpoint_out.y = msg->y;
	local_position_setpoint_out.z = msg->z;
	local_position_setpoint_out.yaw = msg->yaw;

    
    mavlink_msg_local_position_setpoint_encode(msg->sysid, msg->compid, &m, &local_position_setpoint_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_global_position_setpoint_int_callback(const mavlink_common::GLOBAL_POSITION_SETPOINT_INT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_GLOBAL_POSITION_SETPOINT_INT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_global_position_setpoint_int_t global_position_setpoint_int_out;
    
    /** ASSIGN FIELDS **/
    
	global_position_setpoint_int_out.coordinate_frame = msg->coordinate_frame;
	global_position_setpoint_int_out.latitude = msg->latitude;
	global_position_setpoint_int_out.longitude = msg->longitude;
	global_position_setpoint_int_out.altitude = msg->altitude;
	global_position_setpoint_int_out.yaw = msg->yaw;

    
    mavlink_msg_global_position_setpoint_int_encode(msg->sysid, msg->compid, &m, &global_position_setpoint_int_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_set_global_position_setpoint_int_callback(const mavlink_common::SET_GLOBAL_POSITION_SETPOINT_INT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SET_GLOBAL_POSITION_SETPOINT_INT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_set_global_position_setpoint_int_t set_global_position_setpoint_int_out;
    
    /** ASSIGN FIELDS **/
    
	set_global_position_setpoint_int_out.coordinate_frame = msg->coordinate_frame;
	set_global_position_setpoint_int_out.latitude = msg->latitude;
	set_global_position_setpoint_int_out.longitude = msg->longitude;
	set_global_position_setpoint_int_out.altitude = msg->altitude;
	set_global_position_setpoint_int_out.yaw = msg->yaw;

    
    mavlink_msg_set_global_position_setpoint_int_encode(msg->sysid, msg->compid, &m, &set_global_position_setpoint_int_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_safety_set_allowed_area_callback(const mavlink_common::SAFETY_SET_ALLOWED_AREA::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SAFETY_SET_ALLOWED_AREA request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_safety_set_allowed_area_t safety_set_allowed_area_out;
    
    /** ASSIGN FIELDS **/
    
	safety_set_allowed_area_out.target_system = msg->target_system;
	safety_set_allowed_area_out.target_component = msg->target_component;
	safety_set_allowed_area_out.frame = msg->frame;
	safety_set_allowed_area_out.p1x = msg->p1x;
	safety_set_allowed_area_out.p1y = msg->p1y;
	safety_set_allowed_area_out.p1z = msg->p1z;
	safety_set_allowed_area_out.p2x = msg->p2x;
	safety_set_allowed_area_out.p2y = msg->p2y;
	safety_set_allowed_area_out.p2z = msg->p2z;

    
    mavlink_msg_safety_set_allowed_area_encode(msg->sysid, msg->compid, &m, &safety_set_allowed_area_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_safety_allowed_area_callback(const mavlink_common::SAFETY_ALLOWED_AREA::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SAFETY_ALLOWED_AREA request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_safety_allowed_area_t safety_allowed_area_out;
    
    /** ASSIGN FIELDS **/
    
	safety_allowed_area_out.frame = msg->frame;
	safety_allowed_area_out.p1x = msg->p1x;
	safety_allowed_area_out.p1y = msg->p1y;
	safety_allowed_area_out.p1z = msg->p1z;
	safety_allowed_area_out.p2x = msg->p2x;
	safety_allowed_area_out.p2y = msg->p2y;
	safety_allowed_area_out.p2z = msg->p2z;

    
    mavlink_msg_safety_allowed_area_encode(msg->sysid, msg->compid, &m, &safety_allowed_area_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_set_roll_pitch_yaw_thrust_callback(const mavlink_common::SET_ROLL_PITCH_YAW_THRUST::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SET_ROLL_PITCH_YAW_THRUST request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_set_roll_pitch_yaw_thrust_t set_roll_pitch_yaw_thrust_out;
    
    /** ASSIGN FIELDS **/
    
	set_roll_pitch_yaw_thrust_out.target_system = msg->target_system;
	set_roll_pitch_yaw_thrust_out.target_component = msg->target_component;
	set_roll_pitch_yaw_thrust_out.roll = msg->roll;
	set_roll_pitch_yaw_thrust_out.pitch = msg->pitch;
	set_roll_pitch_yaw_thrust_out.yaw = msg->yaw;
	set_roll_pitch_yaw_thrust_out.thrust = msg->thrust;

    
    mavlink_msg_set_roll_pitch_yaw_thrust_encode(msg->sysid, msg->compid, &m, &set_roll_pitch_yaw_thrust_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_set_roll_pitch_yaw_speed_thrust_callback(const mavlink_common::SET_ROLL_PITCH_YAW_SPEED_THRUST::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SET_ROLL_PITCH_YAW_SPEED_THRUST request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_set_roll_pitch_yaw_speed_thrust_t set_roll_pitch_yaw_speed_thrust_out;
    
    /** ASSIGN FIELDS **/
    
	set_roll_pitch_yaw_speed_thrust_out.target_system = msg->target_system;
	set_roll_pitch_yaw_speed_thrust_out.target_component = msg->target_component;
	set_roll_pitch_yaw_speed_thrust_out.roll_speed = msg->roll_speed;
	set_roll_pitch_yaw_speed_thrust_out.pitch_speed = msg->pitch_speed;
	set_roll_pitch_yaw_speed_thrust_out.yaw_speed = msg->yaw_speed;
	set_roll_pitch_yaw_speed_thrust_out.thrust = msg->thrust;

    
    mavlink_msg_set_roll_pitch_yaw_speed_thrust_encode(msg->sysid, msg->compid, &m, &set_roll_pitch_yaw_speed_thrust_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_roll_pitch_yaw_thrust_setpoint_callback(const mavlink_common::ROLL_PITCH_YAW_THRUST_SETPOINT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_ROLL_PITCH_YAW_THRUST_SETPOINT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_roll_pitch_yaw_thrust_setpoint_t roll_pitch_yaw_thrust_setpoint_out;
    
    /** ASSIGN FIELDS **/
    
	roll_pitch_yaw_thrust_setpoint_out.time_boot_ms = msg->time_boot_ms;
	roll_pitch_yaw_thrust_setpoint_out.roll = msg->roll;
	roll_pitch_yaw_thrust_setpoint_out.pitch = msg->pitch;
	roll_pitch_yaw_thrust_setpoint_out.yaw = msg->yaw;
	roll_pitch_yaw_thrust_setpoint_out.thrust = msg->thrust;

    
    mavlink_msg_roll_pitch_yaw_thrust_setpoint_encode(msg->sysid, msg->compid, &m, &roll_pitch_yaw_thrust_setpoint_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_roll_pitch_yaw_speed_thrust_setpoint_callback(const mavlink_common::ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_roll_pitch_yaw_speed_thrust_setpoint_t roll_pitch_yaw_speed_thrust_setpoint_out;
    
    /** ASSIGN FIELDS **/
    
	roll_pitch_yaw_speed_thrust_setpoint_out.time_boot_ms = msg->time_boot_ms;
	roll_pitch_yaw_speed_thrust_setpoint_out.roll_speed = msg->roll_speed;
	roll_pitch_yaw_speed_thrust_setpoint_out.pitch_speed = msg->pitch_speed;
	roll_pitch_yaw_speed_thrust_setpoint_out.yaw_speed = msg->yaw_speed;
	roll_pitch_yaw_speed_thrust_setpoint_out.thrust = msg->thrust;

    
    mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_encode(msg->sysid, msg->compid, &m, &roll_pitch_yaw_speed_thrust_setpoint_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_set_quad_motors_setpoint_callback(const mavlink_common::SET_QUAD_MOTORS_SETPOINT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SET_QUAD_MOTORS_SETPOINT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_set_quad_motors_setpoint_t set_quad_motors_setpoint_out;
    
    /** ASSIGN FIELDS **/
    
	set_quad_motors_setpoint_out.target_system = msg->target_system;
	set_quad_motors_setpoint_out.motor_front_nw = msg->motor_front_nw;
	set_quad_motors_setpoint_out.motor_right_ne = msg->motor_right_ne;
	set_quad_motors_setpoint_out.motor_back_se = msg->motor_back_se;
	set_quad_motors_setpoint_out.motor_left_sw = msg->motor_left_sw;

    
    mavlink_msg_set_quad_motors_setpoint_encode(msg->sysid, msg->compid, &m, &set_quad_motors_setpoint_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_set_quad_swarm_roll_pitch_yaw_thrust_callback(const mavlink_common::SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t set_quad_swarm_roll_pitch_yaw_thrust_out;
    
    /** ASSIGN FIELDS **/
    
	set_quad_swarm_roll_pitch_yaw_thrust_out.group = msg->group;
	set_quad_swarm_roll_pitch_yaw_thrust_out.mode = msg->mode;
	memcpy(&(set_quad_swarm_roll_pitch_yaw_thrust_out.roll), &(msg->roll[0]), sizeof(int16_t)* (int)(msg->roll.size()));
	memcpy(&(set_quad_swarm_roll_pitch_yaw_thrust_out.pitch), &(msg->pitch[0]), sizeof(int16_t)* (int)(msg->pitch.size()));
	memcpy(&(set_quad_swarm_roll_pitch_yaw_thrust_out.yaw), &(msg->yaw[0]), sizeof(int16_t)* (int)(msg->yaw.size()));
	memcpy(&(set_quad_swarm_roll_pitch_yaw_thrust_out.thrust), &(msg->thrust[0]), sizeof(uint16_t)* (int)(msg->thrust.size()));

    
    mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_encode(msg->sysid, msg->compid, &m, &set_quad_swarm_roll_pitch_yaw_thrust_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_nav_controller_output_callback(const mavlink_common::NAV_CONTROLLER_OUTPUT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_NAV_CONTROLLER_OUTPUT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_nav_controller_output_t nav_controller_output_out;
    
    /** ASSIGN FIELDS **/
    
	nav_controller_output_out.nav_roll = msg->nav_roll;
	nav_controller_output_out.nav_pitch = msg->nav_pitch;
	nav_controller_output_out.nav_bearing = msg->nav_bearing;
	nav_controller_output_out.target_bearing = msg->target_bearing;
	nav_controller_output_out.wp_dist = msg->wp_dist;
	nav_controller_output_out.alt_error = msg->alt_error;
	nav_controller_output_out.aspd_error = msg->aspd_error;
	nav_controller_output_out.xtrack_error = msg->xtrack_error;

    
    mavlink_msg_nav_controller_output_encode(msg->sysid, msg->compid, &m, &nav_controller_output_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_set_quad_swarm_led_roll_pitch_yaw_thrust_callback(const mavlink_common::SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_set_quad_swarm_led_roll_pitch_yaw_thrust_t set_quad_swarm_led_roll_pitch_yaw_thrust_out;
    
    /** ASSIGN FIELDS **/
    
	set_quad_swarm_led_roll_pitch_yaw_thrust_out.group = msg->group;
	set_quad_swarm_led_roll_pitch_yaw_thrust_out.mode = msg->mode;
	memcpy(&(set_quad_swarm_led_roll_pitch_yaw_thrust_out.led_red), &(msg->led_red[0]), sizeof(uint8_t)* (int)(msg->led_red.size()));
	memcpy(&(set_quad_swarm_led_roll_pitch_yaw_thrust_out.led_blue), &(msg->led_blue[0]), sizeof(uint8_t)* (int)(msg->led_blue.size()));
	memcpy(&(set_quad_swarm_led_roll_pitch_yaw_thrust_out.led_green), &(msg->led_green[0]), sizeof(uint8_t)* (int)(msg->led_green.size()));
	memcpy(&(set_quad_swarm_led_roll_pitch_yaw_thrust_out.roll), &(msg->roll[0]), sizeof(int16_t)* (int)(msg->roll.size()));
	memcpy(&(set_quad_swarm_led_roll_pitch_yaw_thrust_out.pitch), &(msg->pitch[0]), sizeof(int16_t)* (int)(msg->pitch.size()));
	memcpy(&(set_quad_swarm_led_roll_pitch_yaw_thrust_out.yaw), &(msg->yaw[0]), sizeof(int16_t)* (int)(msg->yaw.size()));
	memcpy(&(set_quad_swarm_led_roll_pitch_yaw_thrust_out.thrust), &(msg->thrust[0]), sizeof(uint16_t)* (int)(msg->thrust.size()));

    
    mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_encode(msg->sysid, msg->compid, &m, &set_quad_swarm_led_roll_pitch_yaw_thrust_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_state_correction_callback(const mavlink_common::STATE_CORRECTION::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_STATE_CORRECTION request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_state_correction_t state_correction_out;
    
    /** ASSIGN FIELDS **/
    
	state_correction_out.xErr = msg->xErr;
	state_correction_out.yErr = msg->yErr;
	state_correction_out.zErr = msg->zErr;
	state_correction_out.rollErr = msg->rollErr;
	state_correction_out.pitchErr = msg->pitchErr;
	state_correction_out.yawErr = msg->yawErr;
	state_correction_out.vxErr = msg->vxErr;
	state_correction_out.vyErr = msg->vyErr;
	state_correction_out.vzErr = msg->vzErr;

    
    mavlink_msg_state_correction_encode(msg->sysid, msg->compid, &m, &state_correction_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_request_data_stream_callback(const mavlink_common::REQUEST_DATA_STREAM::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_REQUEST_DATA_STREAM request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_request_data_stream_t request_data_stream_out;
    
    /** ASSIGN FIELDS **/
    
	request_data_stream_out.target_system = msg->target_system;
	request_data_stream_out.target_component = msg->target_component;
	request_data_stream_out.req_stream_id = msg->req_stream_id;
	request_data_stream_out.req_message_rate = msg->req_message_rate;
	request_data_stream_out.start_stop = msg->start_stop;

    
    mavlink_msg_request_data_stream_encode(msg->sysid, msg->compid, &m, &request_data_stream_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_data_stream_callback(const mavlink_common::DATA_STREAM::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_DATA_STREAM request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_data_stream_t data_stream_out;
    
    /** ASSIGN FIELDS **/
    
	data_stream_out.stream_id = msg->stream_id;
	data_stream_out.message_rate = msg->message_rate;
	data_stream_out.on_off = msg->on_off;

    
    mavlink_msg_data_stream_encode(msg->sysid, msg->compid, &m, &data_stream_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_manual_control_callback(const mavlink_common::MANUAL_CONTROL::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MANUAL_CONTROL request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_manual_control_t manual_control_out;
    
    /** ASSIGN FIELDS **/
    
	manual_control_out.target = msg->target;
	manual_control_out.x = msg->x;
	manual_control_out.y = msg->y;
	manual_control_out.z = msg->z;
	manual_control_out.r = msg->r;
	manual_control_out.buttons = msg->buttons;

    
    mavlink_msg_manual_control_encode(msg->sysid, msg->compid, &m, &manual_control_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_rc_channels_override_callback(const mavlink_common::RC_CHANNELS_OVERRIDE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_RC_CHANNELS_OVERRIDE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_rc_channels_override_t rc_channels_override_out;
    
    /** ASSIGN FIELDS **/
    
	rc_channels_override_out.target_system = msg->target_system;
	rc_channels_override_out.target_component = msg->target_component;
	rc_channels_override_out.chan1_raw = msg->chan1_raw;
	rc_channels_override_out.chan2_raw = msg->chan2_raw;
	rc_channels_override_out.chan3_raw = msg->chan3_raw;
	rc_channels_override_out.chan4_raw = msg->chan4_raw;
	rc_channels_override_out.chan5_raw = msg->chan5_raw;
	rc_channels_override_out.chan6_raw = msg->chan6_raw;
	rc_channels_override_out.chan7_raw = msg->chan7_raw;
	rc_channels_override_out.chan8_raw = msg->chan8_raw;

    
    mavlink_msg_rc_channels_override_encode(msg->sysid, msg->compid, &m, &rc_channels_override_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_vfr_hud_callback(const mavlink_common::VFR_HUD::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_VFR_HUD request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_vfr_hud_t vfr_hud_out;
    
    /** ASSIGN FIELDS **/
    
	vfr_hud_out.airspeed = msg->airspeed;
	vfr_hud_out.groundspeed = msg->groundspeed;
	vfr_hud_out.heading = msg->heading;
	vfr_hud_out.throttle = msg->throttle;
	vfr_hud_out.alt = msg->alt;
	vfr_hud_out.climb = msg->climb;

    
    mavlink_msg_vfr_hud_encode(msg->sysid, msg->compid, &m, &vfr_hud_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_command_long_callback(const mavlink_common::COMMAND_LONG::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_COMMAND_LONG request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_command_long_t command_long_out;
    
    /** ASSIGN FIELDS **/
    
	command_long_out.target_system = msg->target_system;
	command_long_out.target_component = msg->target_component;
	command_long_out.command = msg->command;
	command_long_out.confirmation = msg->confirmation;
	command_long_out.param1 = msg->param1;
	command_long_out.param2 = msg->param2;
	command_long_out.param3 = msg->param3;
	command_long_out.param4 = msg->param4;
	command_long_out.param5 = msg->param5;
	command_long_out.param6 = msg->param6;
	command_long_out.param7 = msg->param7;

    
    mavlink_msg_command_long_encode(msg->sysid, msg->compid, &m, &command_long_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_command_ack_callback(const mavlink_common::COMMAND_ACK::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_COMMAND_ACK request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_command_ack_t command_ack_out;
    
    /** ASSIGN FIELDS **/
    
	command_ack_out.command = msg->command;
	command_ack_out.result = msg->result;

    
    mavlink_msg_command_ack_encode(msg->sysid, msg->compid, &m, &command_ack_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_roll_pitch_yaw_rates_thrust_setpoint_callback(const mavlink_common::ROLL_PITCH_YAW_RATES_THRUST_SETPOINT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_ROLL_PITCH_YAW_RATES_THRUST_SETPOINT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_roll_pitch_yaw_rates_thrust_setpoint_t roll_pitch_yaw_rates_thrust_setpoint_out;
    
    /** ASSIGN FIELDS **/
    
	roll_pitch_yaw_rates_thrust_setpoint_out.time_boot_ms = msg->time_boot_ms;
	roll_pitch_yaw_rates_thrust_setpoint_out.roll_rate = msg->roll_rate;
	roll_pitch_yaw_rates_thrust_setpoint_out.pitch_rate = msg->pitch_rate;
	roll_pitch_yaw_rates_thrust_setpoint_out.yaw_rate = msg->yaw_rate;
	roll_pitch_yaw_rates_thrust_setpoint_out.thrust = msg->thrust;

    
    mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_encode(msg->sysid, msg->compid, &m, &roll_pitch_yaw_rates_thrust_setpoint_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_manual_setpoint_callback(const mavlink_common::MANUAL_SETPOINT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MANUAL_SETPOINT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_manual_setpoint_t manual_setpoint_out;
    
    /** ASSIGN FIELDS **/
    
	manual_setpoint_out.time_boot_ms = msg->time_boot_ms;
	manual_setpoint_out.roll = msg->roll;
	manual_setpoint_out.pitch = msg->pitch;
	manual_setpoint_out.yaw = msg->yaw;
	manual_setpoint_out.thrust = msg->thrust;
	manual_setpoint_out.mode_switch = msg->mode_switch;
	manual_setpoint_out.manual_override_switch = msg->manual_override_switch;

    
    mavlink_msg_manual_setpoint_encode(msg->sysid, msg->compid, &m, &manual_setpoint_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_local_position_ned_system_global_offset_callback(const mavlink_common::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_local_position_ned_system_global_offset_t local_position_ned_system_global_offset_out;
    
    /** ASSIGN FIELDS **/
    
	local_position_ned_system_global_offset_out.time_boot_ms = msg->time_boot_ms;
	local_position_ned_system_global_offset_out.x = msg->x;
	local_position_ned_system_global_offset_out.y = msg->y;
	local_position_ned_system_global_offset_out.z = msg->z;
	local_position_ned_system_global_offset_out.roll = msg->roll;
	local_position_ned_system_global_offset_out.pitch = msg->pitch;
	local_position_ned_system_global_offset_out.yaw = msg->yaw;

    
    mavlink_msg_local_position_ned_system_global_offset_encode(msg->sysid, msg->compid, &m, &local_position_ned_system_global_offset_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_hil_state_callback(const mavlink_common::HIL_STATE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_HIL_STATE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_hil_state_t hil_state_out;
    
    /** ASSIGN FIELDS **/
    
	hil_state_out.time_usec = msg->time_usec;
	hil_state_out.roll = msg->roll;
	hil_state_out.pitch = msg->pitch;
	hil_state_out.yaw = msg->yaw;
	hil_state_out.rollspeed = msg->rollspeed;
	hil_state_out.pitchspeed = msg->pitchspeed;
	hil_state_out.yawspeed = msg->yawspeed;
	hil_state_out.lat = msg->lat;
	hil_state_out.lon = msg->lon;
	hil_state_out.alt = msg->alt;
	hil_state_out.vx = msg->vx;
	hil_state_out.vy = msg->vy;
	hil_state_out.vz = msg->vz;
	hil_state_out.xacc = msg->xacc;
	hil_state_out.yacc = msg->yacc;
	hil_state_out.zacc = msg->zacc;

    
    mavlink_msg_hil_state_encode(msg->sysid, msg->compid, &m, &hil_state_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_hil_controls_callback(const mavlink_common::HIL_CONTROLS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_HIL_CONTROLS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_hil_controls_t hil_controls_out;
    
    /** ASSIGN FIELDS **/
    
	hil_controls_out.time_usec = msg->time_usec;
	hil_controls_out.roll_ailerons = msg->roll_ailerons;
	hil_controls_out.pitch_elevator = msg->pitch_elevator;
	hil_controls_out.yaw_rudder = msg->yaw_rudder;
	hil_controls_out.throttle = msg->throttle;
	hil_controls_out.aux1 = msg->aux1;
	hil_controls_out.aux2 = msg->aux2;
	hil_controls_out.aux3 = msg->aux3;
	hil_controls_out.aux4 = msg->aux4;
	hil_controls_out.mode = msg->mode;
	hil_controls_out.nav_mode = msg->nav_mode;

    
    mavlink_msg_hil_controls_encode(msg->sysid, msg->compid, &m, &hil_controls_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_hil_rc_inputs_raw_callback(const mavlink_common::HIL_RC_INPUTS_RAW::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_HIL_RC_INPUTS_RAW request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_hil_rc_inputs_raw_t hil_rc_inputs_raw_out;
    
    /** ASSIGN FIELDS **/
    
	hil_rc_inputs_raw_out.time_usec = msg->time_usec;
	hil_rc_inputs_raw_out.chan1_raw = msg->chan1_raw;
	hil_rc_inputs_raw_out.chan2_raw = msg->chan2_raw;
	hil_rc_inputs_raw_out.chan3_raw = msg->chan3_raw;
	hil_rc_inputs_raw_out.chan4_raw = msg->chan4_raw;
	hil_rc_inputs_raw_out.chan5_raw = msg->chan5_raw;
	hil_rc_inputs_raw_out.chan6_raw = msg->chan6_raw;
	hil_rc_inputs_raw_out.chan7_raw = msg->chan7_raw;
	hil_rc_inputs_raw_out.chan8_raw = msg->chan8_raw;
	hil_rc_inputs_raw_out.chan9_raw = msg->chan9_raw;
	hil_rc_inputs_raw_out.chan10_raw = msg->chan10_raw;
	hil_rc_inputs_raw_out.chan11_raw = msg->chan11_raw;
	hil_rc_inputs_raw_out.chan12_raw = msg->chan12_raw;
	hil_rc_inputs_raw_out.rssi = msg->rssi;

    
    mavlink_msg_hil_rc_inputs_raw_encode(msg->sysid, msg->compid, &m, &hil_rc_inputs_raw_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_optical_flow_callback(const mavlink_common::OPTICAL_FLOW::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_OPTICAL_FLOW request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_optical_flow_t optical_flow_out;
    
    /** ASSIGN FIELDS **/
    
	optical_flow_out.time_usec = msg->time_usec;
	optical_flow_out.sensor_id = msg->sensor_id;
	optical_flow_out.flow_x = msg->flow_x;
	optical_flow_out.flow_y = msg->flow_y;
	optical_flow_out.flow_comp_m_x = msg->flow_comp_m_x;
	optical_flow_out.flow_comp_m_y = msg->flow_comp_m_y;
	optical_flow_out.quality = msg->quality;
	optical_flow_out.ground_distance = msg->ground_distance;

    
    mavlink_msg_optical_flow_encode(msg->sysid, msg->compid, &m, &optical_flow_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_global_vision_position_estimate_callback(const mavlink_common::GLOBAL_VISION_POSITION_ESTIMATE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_GLOBAL_VISION_POSITION_ESTIMATE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_global_vision_position_estimate_t global_vision_position_estimate_out;
    
    /** ASSIGN FIELDS **/
    
	global_vision_position_estimate_out.usec = msg->usec;
	global_vision_position_estimate_out.x = msg->x;
	global_vision_position_estimate_out.y = msg->y;
	global_vision_position_estimate_out.z = msg->z;
	global_vision_position_estimate_out.roll = msg->roll;
	global_vision_position_estimate_out.pitch = msg->pitch;
	global_vision_position_estimate_out.yaw = msg->yaw;

    
    mavlink_msg_global_vision_position_estimate_encode(msg->sysid, msg->compid, &m, &global_vision_position_estimate_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_vision_position_estimate_callback(const mavlink_common::VISION_POSITION_ESTIMATE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_VISION_POSITION_ESTIMATE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_vision_position_estimate_t vision_position_estimate_out;
    
    /** ASSIGN FIELDS **/
    
	vision_position_estimate_out.usec = msg->usec;
	vision_position_estimate_out.x = msg->x;
	vision_position_estimate_out.y = msg->y;
	vision_position_estimate_out.z = msg->z;
	vision_position_estimate_out.roll = msg->roll;
	vision_position_estimate_out.pitch = msg->pitch;
	vision_position_estimate_out.yaw = msg->yaw;

    
    mavlink_msg_vision_position_estimate_encode(msg->sysid, msg->compid, &m, &vision_position_estimate_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_vision_speed_estimate_callback(const mavlink_common::VISION_SPEED_ESTIMATE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_VISION_SPEED_ESTIMATE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_vision_speed_estimate_t vision_speed_estimate_out;
    
    /** ASSIGN FIELDS **/
    
	vision_speed_estimate_out.usec = msg->usec;
	vision_speed_estimate_out.x = msg->x;
	vision_speed_estimate_out.y = msg->y;
	vision_speed_estimate_out.z = msg->z;

    
    mavlink_msg_vision_speed_estimate_encode(msg->sysid, msg->compid, &m, &vision_speed_estimate_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_vicon_position_estimate_callback(const mavlink_common::VICON_POSITION_ESTIMATE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_VICON_POSITION_ESTIMATE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_vicon_position_estimate_t vicon_position_estimate_out;
    
    /** ASSIGN FIELDS **/
    
	vicon_position_estimate_out.usec = msg->usec;
	vicon_position_estimate_out.x = msg->x;
	vicon_position_estimate_out.y = msg->y;
	vicon_position_estimate_out.z = msg->z;
	vicon_position_estimate_out.roll = msg->roll;
	vicon_position_estimate_out.pitch = msg->pitch;
	vicon_position_estimate_out.yaw = msg->yaw;

    
    mavlink_msg_vicon_position_estimate_encode(msg->sysid, msg->compid, &m, &vicon_position_estimate_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_highres_imu_callback(const mavlink_common::HIGHRES_IMU::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_HIGHRES_IMU request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_highres_imu_t highres_imu_out;
    
    /** ASSIGN FIELDS **/
    
	highres_imu_out.time_usec = msg->time_usec;
	highres_imu_out.xacc = msg->xacc;
	highres_imu_out.yacc = msg->yacc;
	highres_imu_out.zacc = msg->zacc;
	highres_imu_out.xgyro = msg->xgyro;
	highres_imu_out.ygyro = msg->ygyro;
	highres_imu_out.zgyro = msg->zgyro;
	highres_imu_out.xmag = msg->xmag;
	highres_imu_out.ymag = msg->ymag;
	highres_imu_out.zmag = msg->zmag;
	highres_imu_out.abs_pressure = msg->abs_pressure;
	highres_imu_out.diff_pressure = msg->diff_pressure;
	highres_imu_out.pressure_alt = msg->pressure_alt;
	highres_imu_out.temperature = msg->temperature;
	highres_imu_out.fields_updated = msg->fields_updated;

    
    mavlink_msg_highres_imu_encode(msg->sysid, msg->compid, &m, &highres_imu_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_file_transfer_start_callback(const mavlink_common::FILE_TRANSFER_START::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_FILE_TRANSFER_START request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_file_transfer_start_t file_transfer_start_out;
    
    /** ASSIGN FIELDS **/
    
	file_transfer_start_out.transfer_uid = msg->transfer_uid;
	memcpy(&(file_transfer_start_out.dest_path), &(msg->dest_path[0]), sizeof(char)* (int)(msg->dest_path.size()));
	file_transfer_start_out.direction = msg->direction;
	file_transfer_start_out.file_size = msg->file_size;
	file_transfer_start_out.flags = msg->flags;

    
    mavlink_msg_file_transfer_start_encode(msg->sysid, msg->compid, &m, &file_transfer_start_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_file_transfer_dir_list_callback(const mavlink_common::FILE_TRANSFER_DIR_LIST::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_FILE_TRANSFER_DIR_LIST request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_file_transfer_dir_list_t file_transfer_dir_list_out;
    
    /** ASSIGN FIELDS **/
    
	file_transfer_dir_list_out.transfer_uid = msg->transfer_uid;
	memcpy(&(file_transfer_dir_list_out.dir_path), &(msg->dir_path[0]), sizeof(char)* (int)(msg->dir_path.size()));
	file_transfer_dir_list_out.flags = msg->flags;

    
    mavlink_msg_file_transfer_dir_list_encode(msg->sysid, msg->compid, &m, &file_transfer_dir_list_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_file_transfer_res_callback(const mavlink_common::FILE_TRANSFER_RES::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_FILE_TRANSFER_RES request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_file_transfer_res_t file_transfer_res_out;
    
    /** ASSIGN FIELDS **/
    
	file_transfer_res_out.transfer_uid = msg->transfer_uid;
	file_transfer_res_out.result = msg->result;

    
    mavlink_msg_file_transfer_res_encode(msg->sysid, msg->compid, &m, &file_transfer_res_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_battery_status_callback(const mavlink_common::BATTERY_STATUS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_BATTERY_STATUS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_battery_status_t battery_status_out;
    
    /** ASSIGN FIELDS **/
    
	battery_status_out.accu_id = msg->accu_id;
	battery_status_out.voltage_cell_1 = msg->voltage_cell_1;
	battery_status_out.voltage_cell_2 = msg->voltage_cell_2;
	battery_status_out.voltage_cell_3 = msg->voltage_cell_3;
	battery_status_out.voltage_cell_4 = msg->voltage_cell_4;
	battery_status_out.voltage_cell_5 = msg->voltage_cell_5;
	battery_status_out.voltage_cell_6 = msg->voltage_cell_6;
	battery_status_out.current_battery = msg->current_battery;
	battery_status_out.battery_remaining = msg->battery_remaining;

    
    mavlink_msg_battery_status_encode(msg->sysid, msg->compid, &m, &battery_status_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_setpoint_8dof_callback(const mavlink_common::SETPOINT_8DOF::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SETPOINT_8DOF request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_setpoint_8dof_t setpoint_8dof_out;
    
    /** ASSIGN FIELDS **/
    
	setpoint_8dof_out.target_system = msg->target_system;
	setpoint_8dof_out.val1 = msg->val1;
	setpoint_8dof_out.val2 = msg->val2;
	setpoint_8dof_out.val3 = msg->val3;
	setpoint_8dof_out.val4 = msg->val4;
	setpoint_8dof_out.val5 = msg->val5;
	setpoint_8dof_out.val6 = msg->val6;
	setpoint_8dof_out.val7 = msg->val7;
	setpoint_8dof_out.val8 = msg->val8;

    
    mavlink_msg_setpoint_8dof_encode(msg->sysid, msg->compid, &m, &setpoint_8dof_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_setpoint_6dof_callback(const mavlink_common::SETPOINT_6DOF::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_SETPOINT_6DOF request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_setpoint_6dof_t setpoint_6dof_out;
    
    /** ASSIGN FIELDS **/
    
	setpoint_6dof_out.target_system = msg->target_system;
	setpoint_6dof_out.trans_x = msg->trans_x;
	setpoint_6dof_out.trans_y = msg->trans_y;
	setpoint_6dof_out.trans_z = msg->trans_z;
	setpoint_6dof_out.rot_x = msg->rot_x;
	setpoint_6dof_out.rot_y = msg->rot_y;
	setpoint_6dof_out.rot_z = msg->rot_z;

    
    mavlink_msg_setpoint_6dof_encode(msg->sysid, msg->compid, &m, &setpoint_6dof_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_memory_vect_callback(const mavlink_common::MEMORY_VECT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_MEMORY_VECT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_memory_vect_t memory_vect_out;
    
    /** ASSIGN FIELDS **/
    
	memory_vect_out.address = msg->address;
	memory_vect_out.ver = msg->ver;
	memory_vect_out.type = msg->type;
	memcpy(&(memory_vect_out.value), &(msg->value[0]), sizeof(int8_t)* (int)(msg->value.size()));

    
    mavlink_msg_memory_vect_encode(msg->sysid, msg->compid, &m, &memory_vect_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_debug_vect_callback(const mavlink_common::DEBUG_VECT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_DEBUG_VECT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_debug_vect_t debug_vect_out;
    
    /** ASSIGN FIELDS **/
    
	memcpy(&(debug_vect_out.name), &(msg->name[0]), sizeof(char)* (int)(msg->name.size()));
	debug_vect_out.time_usec = msg->time_usec;
	debug_vect_out.x = msg->x;
	debug_vect_out.y = msg->y;
	debug_vect_out.z = msg->z;

    
    mavlink_msg_debug_vect_encode(msg->sysid, msg->compid, &m, &debug_vect_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_named_value_float_callback(const mavlink_common::NAMED_VALUE_FLOAT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_NAMED_VALUE_FLOAT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_named_value_float_t named_value_float_out;
    
    /** ASSIGN FIELDS **/
    
	named_value_float_out.time_boot_ms = msg->time_boot_ms;
	memcpy(&(named_value_float_out.name), &(msg->name[0]), sizeof(char)* (int)(msg->name.size()));
	named_value_float_out.value = msg->value;

    
    mavlink_msg_named_value_float_encode(msg->sysid, msg->compid, &m, &named_value_float_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_named_value_int_callback(const mavlink_common::NAMED_VALUE_INT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_NAMED_VALUE_INT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_named_value_int_t named_value_int_out;
    
    /** ASSIGN FIELDS **/
    
	named_value_int_out.time_boot_ms = msg->time_boot_ms;
	memcpy(&(named_value_int_out.name), &(msg->name[0]), sizeof(char)* (int)(msg->name.size()));
	named_value_int_out.value = msg->value;

    
    mavlink_msg_named_value_int_encode(msg->sysid, msg->compid, &m, &named_value_int_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_statustext_callback(const mavlink_common::STATUSTEXT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_STATUSTEXT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_statustext_t statustext_out;
    
    /** ASSIGN FIELDS **/
    
	statustext_out.severity = msg->severity;
	memcpy(&(statustext_out.text), &(msg->text[0]), sizeof(char)* (int)(msg->text.size()));

    
    mavlink_msg_statustext_encode(msg->sysid, msg->compid, &m, &statustext_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_debug_callback(const mavlink_common::DEBUG::ConstPtr& msg)
{
    ROS_INFO("[mavlink_common] Received a  'to_mav_DEBUG request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_debug_t debug_out;
    
    /** ASSIGN FIELDS **/
    
	debug_out.time_boot_ms = msg->time_boot_ms;
	debug_out.ind = msg->ind;
	debug_out.value = msg->value;

    
    mavlink_msg_debug_encode(msg->sysid, msg->compid, &m, &debug_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_sensor_offsets_callback(const mavlink_ardupilotmega::SENSOR_OFFSETS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_SENSOR_OFFSETS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_sensor_offsets_t sensor_offsets_out;
    
    /** ASSIGN FIELDS **/
    
	sensor_offsets_out.mag_ofs_x = msg->mag_ofs_x;
	sensor_offsets_out.mag_ofs_y = msg->mag_ofs_y;
	sensor_offsets_out.mag_ofs_z = msg->mag_ofs_z;
	sensor_offsets_out.mag_declination = msg->mag_declination;
	sensor_offsets_out.raw_press = msg->raw_press;
	sensor_offsets_out.raw_temp = msg->raw_temp;
	sensor_offsets_out.gyro_cal_x = msg->gyro_cal_x;
	sensor_offsets_out.gyro_cal_y = msg->gyro_cal_y;
	sensor_offsets_out.gyro_cal_z = msg->gyro_cal_z;
	sensor_offsets_out.accel_cal_x = msg->accel_cal_x;
	sensor_offsets_out.accel_cal_y = msg->accel_cal_y;
	sensor_offsets_out.accel_cal_z = msg->accel_cal_z;

    
    mavlink_msg_sensor_offsets_encode(msg->sysid, msg->compid, &m, &sensor_offsets_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_set_mag_offsets_callback(const mavlink_ardupilotmega::SET_MAG_OFFSETS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_SET_MAG_OFFSETS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_set_mag_offsets_t set_mag_offsets_out;
    
    /** ASSIGN FIELDS **/
    
	set_mag_offsets_out.target_system = msg->target_system;
	set_mag_offsets_out.target_component = msg->target_component;
	set_mag_offsets_out.mag_ofs_x = msg->mag_ofs_x;
	set_mag_offsets_out.mag_ofs_y = msg->mag_ofs_y;
	set_mag_offsets_out.mag_ofs_z = msg->mag_ofs_z;

    
    mavlink_msg_set_mag_offsets_encode(msg->sysid, msg->compid, &m, &set_mag_offsets_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_meminfo_callback(const mavlink_ardupilotmega::MEMINFO::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_MEMINFO request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_meminfo_t meminfo_out;
    
    /** ASSIGN FIELDS **/
    
	meminfo_out.brkval = msg->brkval;
	meminfo_out.freemem = msg->freemem;

    
    mavlink_msg_meminfo_encode(msg->sysid, msg->compid, &m, &meminfo_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_ap_adc_callback(const mavlink_ardupilotmega::AP_ADC::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_AP_ADC request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_ap_adc_t ap_adc_out;
    
    /** ASSIGN FIELDS **/
    
	ap_adc_out.adc1 = msg->adc1;
	ap_adc_out.adc2 = msg->adc2;
	ap_adc_out.adc3 = msg->adc3;
	ap_adc_out.adc4 = msg->adc4;
	ap_adc_out.adc5 = msg->adc5;
	ap_adc_out.adc6 = msg->adc6;

    
    mavlink_msg_ap_adc_encode(msg->sysid, msg->compid, &m, &ap_adc_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_digicam_configure_callback(const mavlink_ardupilotmega::DIGICAM_CONFIGURE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_DIGICAM_CONFIGURE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_digicam_configure_t digicam_configure_out;
    
    /** ASSIGN FIELDS **/
    
	digicam_configure_out.target_system = msg->target_system;
	digicam_configure_out.target_component = msg->target_component;
	digicam_configure_out.mode = msg->mode;
	digicam_configure_out.shutter_speed = msg->shutter_speed;
	digicam_configure_out.aperture = msg->aperture;
	digicam_configure_out.iso = msg->iso;
	digicam_configure_out.exposure_type = msg->exposure_type;
	digicam_configure_out.command_id = msg->command_id;
	digicam_configure_out.engine_cut_off = msg->engine_cut_off;
	digicam_configure_out.extra_param = msg->extra_param;
	digicam_configure_out.extra_value = msg->extra_value;

    
    mavlink_msg_digicam_configure_encode(msg->sysid, msg->compid, &m, &digicam_configure_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_digicam_control_callback(const mavlink_ardupilotmega::DIGICAM_CONTROL::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_DIGICAM_CONTROL request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_digicam_control_t digicam_control_out;
    
    /** ASSIGN FIELDS **/
    
	digicam_control_out.target_system = msg->target_system;
	digicam_control_out.target_component = msg->target_component;
	digicam_control_out.session = msg->session;
	digicam_control_out.zoom_pos = msg->zoom_pos;
	digicam_control_out.zoom_step = msg->zoom_step;
	digicam_control_out.focus_lock = msg->focus_lock;
	digicam_control_out.shot = msg->shot;
	digicam_control_out.command_id = msg->command_id;
	digicam_control_out.extra_param = msg->extra_param;
	digicam_control_out.extra_value = msg->extra_value;

    
    mavlink_msg_digicam_control_encode(msg->sysid, msg->compid, &m, &digicam_control_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mount_configure_callback(const mavlink_ardupilotmega::MOUNT_CONFIGURE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_MOUNT_CONFIGURE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mount_configure_t mount_configure_out;
    
    /** ASSIGN FIELDS **/
    
	mount_configure_out.target_system = msg->target_system;
	mount_configure_out.target_component = msg->target_component;
	mount_configure_out.mount_mode = msg->mount_mode;
	mount_configure_out.stab_roll = msg->stab_roll;
	mount_configure_out.stab_pitch = msg->stab_pitch;
	mount_configure_out.stab_yaw = msg->stab_yaw;

    
    mavlink_msg_mount_configure_encode(msg->sysid, msg->compid, &m, &mount_configure_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mount_control_callback(const mavlink_ardupilotmega::MOUNT_CONTROL::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_MOUNT_CONTROL request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mount_control_t mount_control_out;
    
    /** ASSIGN FIELDS **/
    
	mount_control_out.target_system = msg->target_system;
	mount_control_out.target_component = msg->target_component;
	mount_control_out.input_a = msg->input_a;
	mount_control_out.input_b = msg->input_b;
	mount_control_out.input_c = msg->input_c;
	mount_control_out.save_position = msg->save_position;

    
    mavlink_msg_mount_control_encode(msg->sysid, msg->compid, &m, &mount_control_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_mount_status_callback(const mavlink_ardupilotmega::MOUNT_STATUS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_MOUNT_STATUS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_mount_status_t mount_status_out;
    
    /** ASSIGN FIELDS **/
    
	mount_status_out.target_system = msg->target_system;
	mount_status_out.target_component = msg->target_component;
	mount_status_out.pointing_a = msg->pointing_a;
	mount_status_out.pointing_b = msg->pointing_b;
	mount_status_out.pointing_c = msg->pointing_c;

    
    mavlink_msg_mount_status_encode(msg->sysid, msg->compid, &m, &mount_status_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_fence_point_callback(const mavlink_ardupilotmega::FENCE_POINT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_FENCE_POINT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_fence_point_t fence_point_out;
    
    /** ASSIGN FIELDS **/
    
	fence_point_out.target_system = msg->target_system;
	fence_point_out.target_component = msg->target_component;
	fence_point_out.idx = msg->idx;
	fence_point_out.count = msg->count;
	fence_point_out.lat = msg->lat;
	fence_point_out.lng = msg->lng;

    
    mavlink_msg_fence_point_encode(msg->sysid, msg->compid, &m, &fence_point_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_fence_fetch_point_callback(const mavlink_ardupilotmega::FENCE_FETCH_POINT::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_FENCE_FETCH_POINT request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_fence_fetch_point_t fence_fetch_point_out;
    
    /** ASSIGN FIELDS **/
    
	fence_fetch_point_out.target_system = msg->target_system;
	fence_fetch_point_out.target_component = msg->target_component;
	fence_fetch_point_out.idx = msg->idx;

    
    mavlink_msg_fence_fetch_point_encode(msg->sysid, msg->compid, &m, &fence_fetch_point_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_fence_status_callback(const mavlink_ardupilotmega::FENCE_STATUS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_FENCE_STATUS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_fence_status_t fence_status_out;
    
    /** ASSIGN FIELDS **/
    
	fence_status_out.breach_status = msg->breach_status;
	fence_status_out.breach_count = msg->breach_count;
	fence_status_out.breach_type = msg->breach_type;
	fence_status_out.breach_time = msg->breach_time;

    
    mavlink_msg_fence_status_encode(msg->sysid, msg->compid, &m, &fence_status_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_ahrs_callback(const mavlink_ardupilotmega::AHRS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_AHRS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_ahrs_t ahrs_out;
    
    /** ASSIGN FIELDS **/
    
	ahrs_out.omegaIx = msg->omegaIx;
	ahrs_out.omegaIy = msg->omegaIy;
	ahrs_out.omegaIz = msg->omegaIz;
	ahrs_out.accel_weight = msg->accel_weight;
	ahrs_out.renorm_val = msg->renorm_val;
	ahrs_out.error_rp = msg->error_rp;
	ahrs_out.error_yaw = msg->error_yaw;

    
    mavlink_msg_ahrs_encode(msg->sysid, msg->compid, &m, &ahrs_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_simstate_callback(const mavlink_ardupilotmega::SIMSTATE::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_SIMSTATE request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_simstate_t simstate_out;
    
    /** ASSIGN FIELDS **/
    
	simstate_out.roll = msg->roll;
	simstate_out.pitch = msg->pitch;
	simstate_out.yaw = msg->yaw;
	simstate_out.xacc = msg->xacc;
	simstate_out.yacc = msg->yacc;
	simstate_out.zacc = msg->zacc;
	simstate_out.xgyro = msg->xgyro;
	simstate_out.ygyro = msg->ygyro;
	simstate_out.zgyro = msg->zgyro;
	simstate_out.lat = msg->lat;
	simstate_out.lng = msg->lng;

    
    mavlink_msg_simstate_encode(msg->sysid, msg->compid, &m, &simstate_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_hwstatus_callback(const mavlink_ardupilotmega::HWSTATUS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_HWSTATUS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_hwstatus_t hwstatus_out;
    
    /** ASSIGN FIELDS **/
    
	hwstatus_out.Vcc = msg->Vcc;
	hwstatus_out.I2Cerr = msg->I2Cerr;

    
    mavlink_msg_hwstatus_encode(msg->sysid, msg->compid, &m, &hwstatus_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_radio_callback(const mavlink_ardupilotmega::RADIO::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_RADIO request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_radio_t radio_out;
    
    /** ASSIGN FIELDS **/
    
	radio_out.rssi = msg->rssi;
	radio_out.remrssi = msg->remrssi;
	radio_out.txbuf = msg->txbuf;
	radio_out.noise = msg->noise;
	radio_out.remnoise = msg->remnoise;
	radio_out.rxerrors = msg->rxerrors;
	radio_out.fixed = msg->fixed;

    
    mavlink_msg_radio_encode(msg->sysid, msg->compid, &m, &radio_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_limits_status_callback(const mavlink_ardupilotmega::LIMITS_STATUS::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_LIMITS_STATUS request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_limits_status_t limits_status_out;
    
    /** ASSIGN FIELDS **/
    
	limits_status_out.limits_state = msg->limits_state;
	limits_status_out.last_trigger = msg->last_trigger;
	limits_status_out.last_action = msg->last_action;
	limits_status_out.last_recovery = msg->last_recovery;
	limits_status_out.last_clear = msg->last_clear;
	limits_status_out.breach_count = msg->breach_count;
	limits_status_out.mods_enabled = msg->mods_enabled;
	limits_status_out.mods_required = msg->mods_required;
	limits_status_out.mods_triggered = msg->mods_triggered;

    
    mavlink_msg_limits_status_encode(msg->sysid, msg->compid, &m, &limits_status_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_wind_callback(const mavlink_ardupilotmega::WIND::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_WIND request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_wind_t wind_out;
    
    /** ASSIGN FIELDS **/
    
	wind_out.direction = msg->direction;
	wind_out.speed = msg->speed;
	wind_out.speed_z = msg->speed_z;

    
    mavlink_msg_wind_encode(msg->sysid, msg->compid, &m, &wind_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_data16_callback(const mavlink_ardupilotmega::DATA16::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_DATA16 request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_data16_t data16_out;
    
    /** ASSIGN FIELDS **/
    
	data16_out.type = msg->type;
	data16_out.len = msg->len;
	memcpy(&(data16_out.data), &(msg->data[0]), sizeof(uint8_t)* (int)(msg->data.size()));

    
    mavlink_msg_data16_encode(msg->sysid, msg->compid, &m, &data16_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_data32_callback(const mavlink_ardupilotmega::DATA32::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_DATA32 request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_data32_t data32_out;
    
    /** ASSIGN FIELDS **/
    
	data32_out.type = msg->type;
	data32_out.len = msg->len;
	memcpy(&(data32_out.data), &(msg->data[0]), sizeof(uint8_t)* (int)(msg->data.size()));

    
    mavlink_msg_data32_encode(msg->sysid, msg->compid, &m, &data32_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_data64_callback(const mavlink_ardupilotmega::DATA64::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_DATA64 request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_data64_t data64_out;
    
    /** ASSIGN FIELDS **/
    
	data64_out.type = msg->type;
	data64_out.len = msg->len;
	memcpy(&(data64_out.data), &(msg->data[0]), sizeof(uint8_t)* (int)(msg->data.size()));

    
    mavlink_msg_data64_encode(msg->sysid, msg->compid, &m, &data64_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}
/**
 *
 */            
void to_mav_data96_callback(const mavlink_ardupilotmega::DATA96::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'to_mav_DATA96 request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_data96_t data96_out;
    
    /** ASSIGN FIELDS **/
    
	data96_out.type = msg->type;
	data96_out.len = msg->len;
	memcpy(&(data96_out.data), &(msg->data[0]), sizeof(uint8_t)* (int)(msg->data.size()));

    
    mavlink_msg_data96_encode(msg->sysid, msg->compid, &m, &data96_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}

/**
 *
 */            
void from_mav_raw_callback(const mavlink_ardupilotmega::MAV_RAW::ConstPtr& msg)
{
    ROS_INFO("[mavlink_ardupilotmega] Received a  'from_mav_raw_callback request");
    
    for (int i = 0;i<msg->data.size();++i){
        // Try to get a new message
        if(mavlink_parse_char(msg->channel, msg->data[i], &mav_msg, &status)) {
            // Handle message
            switch(mav_msg.msgid)
            {
                
                    case MAVLINK_MSG_ID_HEARTBEAT:
                    {
                        mavlink_common::HEARTBEAT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_heartbeat_t heartbeat_in;
                        memset(&heartbeat_in, 0, sizeof(heartbeat_in));
                        mavlink_msg_heartbeat_decode(&mav_msg, &heartbeat_in);
                        
                        	m.type = heartbeat_in.type;
	m.autopilot = heartbeat_in.autopilot;
	m.base_mode = heartbeat_in.base_mode;
	m.custom_mode = heartbeat_in.custom_mode;
	m.system_status = heartbeat_in.system_status;
	m.mavlink_version = heartbeat_in.mavlink_version;

                        
                        from_mav_heartbeat_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SYS_STATUS:
                    {
                        mavlink_common::SYS_STATUS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_sys_status_t sys_status_in;
                        memset(&sys_status_in, 0, sizeof(sys_status_in));
                        mavlink_msg_sys_status_decode(&mav_msg, &sys_status_in);
                        
                        	m.onboard_control_sensors_present = sys_status_in.onboard_control_sensors_present;
	m.onboard_control_sensors_enabled = sys_status_in.onboard_control_sensors_enabled;
	m.onboard_control_sensors_health = sys_status_in.onboard_control_sensors_health;
	m.load = sys_status_in.load;
	m.voltage_battery = sys_status_in.voltage_battery;
	m.current_battery = sys_status_in.current_battery;
	m.battery_remaining = sys_status_in.battery_remaining;
	m.drop_rate_comm = sys_status_in.drop_rate_comm;
	m.errors_comm = sys_status_in.errors_comm;
	m.errors_count1 = sys_status_in.errors_count1;
	m.errors_count2 = sys_status_in.errors_count2;
	m.errors_count3 = sys_status_in.errors_count3;
	m.errors_count4 = sys_status_in.errors_count4;

                        
                        from_mav_sys_status_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SYSTEM_TIME:
                    {
                        mavlink_common::SYSTEM_TIME m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_system_time_t system_time_in;
                        memset(&system_time_in, 0, sizeof(system_time_in));
                        mavlink_msg_system_time_decode(&mav_msg, &system_time_in);
                        
                        	m.time_unix_usec = system_time_in.time_unix_usec;
	m.time_boot_ms = system_time_in.time_boot_ms;

                        
                        from_mav_system_time_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_PING:
                    {
                        mavlink_common::PING m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_ping_t ping_in;
                        memset(&ping_in, 0, sizeof(ping_in));
                        mavlink_msg_ping_decode(&mav_msg, &ping_in);
                        
                        	m.time_usec = ping_in.time_usec;
	m.seq = ping_in.seq;
	m.target_system = ping_in.target_system;
	m.target_component = ping_in.target_component;

                        
                        from_mav_ping_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
                    {
                        mavlink_common::CHANGE_OPERATOR_CONTROL m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_change_operator_control_t change_operator_control_in;
                        memset(&change_operator_control_in, 0, sizeof(change_operator_control_in));
                        mavlink_msg_change_operator_control_decode(&mav_msg, &change_operator_control_in);
                        
                        	m.target_system = change_operator_control_in.target_system;
	m.control_request = change_operator_control_in.control_request;
	m.version = change_operator_control_in.version;
	memcpy(&(m.passkey), &(change_operator_control_in.passkey), sizeof(char)*25);

                        
                        from_mav_change_operator_control_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:
                    {
                        mavlink_common::CHANGE_OPERATOR_CONTROL_ACK m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_change_operator_control_ack_t change_operator_control_ack_in;
                        memset(&change_operator_control_ack_in, 0, sizeof(change_operator_control_ack_in));
                        mavlink_msg_change_operator_control_ack_decode(&mav_msg, &change_operator_control_ack_in);
                        
                        	m.gcs_system_id = change_operator_control_ack_in.gcs_system_id;
	m.control_request = change_operator_control_ack_in.control_request;
	m.ack = change_operator_control_ack_in.ack;

                        
                        from_mav_change_operator_control_ack_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_AUTH_KEY:
                    {
                        mavlink_common::AUTH_KEY m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_auth_key_t auth_key_in;
                        memset(&auth_key_in, 0, sizeof(auth_key_in));
                        mavlink_msg_auth_key_decode(&mav_msg, &auth_key_in);
                        
                        	memcpy(&(m.key), &(auth_key_in.key), sizeof(char)*32);

                        
                        from_mav_auth_key_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SET_MODE:
                    {
                        mavlink_common::SET_MODE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_set_mode_t set_mode_in;
                        memset(&set_mode_in, 0, sizeof(set_mode_in));
                        mavlink_msg_set_mode_decode(&mav_msg, &set_mode_in);
                        
                        	m.target_system = set_mode_in.target_system;
	m.base_mode = set_mode_in.base_mode;
	m.custom_mode = set_mode_in.custom_mode;

                        
                        from_mav_set_mode_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                    {
                        mavlink_common::PARAM_REQUEST_READ m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_param_request_read_t param_request_read_in;
                        memset(&param_request_read_in, 0, sizeof(param_request_read_in));
                        mavlink_msg_param_request_read_decode(&mav_msg, &param_request_read_in);
                        
                        	m.target_system = param_request_read_in.target_system;
	m.target_component = param_request_read_in.target_component;
	memcpy(&(m.param_id), &(param_request_read_in.param_id), sizeof(char)*16);
	m.param_index = param_request_read_in.param_index;

                        
                        from_mav_param_request_read_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                    {
                        mavlink_common::PARAM_REQUEST_LIST m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_param_request_list_t param_request_list_in;
                        memset(&param_request_list_in, 0, sizeof(param_request_list_in));
                        mavlink_msg_param_request_list_decode(&mav_msg, &param_request_list_in);
                        
                        	m.target_system = param_request_list_in.target_system;
	m.target_component = param_request_list_in.target_component;

                        
                        from_mav_param_request_list_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_PARAM_VALUE:
                    {
                        mavlink_common::PARAM_VALUE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_param_value_t param_value_in;
                        memset(&param_value_in, 0, sizeof(param_value_in));
                        mavlink_msg_param_value_decode(&mav_msg, &param_value_in);
                        
                        	memcpy(&(m.param_id), &(param_value_in.param_id), sizeof(char)*16);
	m.param_value = param_value_in.param_value;
	m.param_type = param_value_in.param_type;
	m.param_count = param_value_in.param_count;
	m.param_index = param_value_in.param_index;

                        
                        from_mav_param_value_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_PARAM_SET:
                    {
                        mavlink_common::PARAM_SET m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_param_set_t param_set_in;
                        memset(&param_set_in, 0, sizeof(param_set_in));
                        mavlink_msg_param_set_decode(&mav_msg, &param_set_in);
                        
                        	m.target_system = param_set_in.target_system;
	m.target_component = param_set_in.target_component;
	memcpy(&(m.param_id), &(param_set_in.param_id), sizeof(char)*16);
	m.param_value = param_set_in.param_value;
	m.param_type = param_set_in.param_type;

                        
                        from_mav_param_set_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_GPS_RAW_INT:
                    {
                        mavlink_common::GPS_RAW_INT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_gps_raw_int_t gps_raw_int_in;
                        memset(&gps_raw_int_in, 0, sizeof(gps_raw_int_in));
                        mavlink_msg_gps_raw_int_decode(&mav_msg, &gps_raw_int_in);
                        
                        	m.time_usec = gps_raw_int_in.time_usec;
	m.fix_type = gps_raw_int_in.fix_type;
	m.lat = gps_raw_int_in.lat;
	m.lon = gps_raw_int_in.lon;
	m.alt = gps_raw_int_in.alt;
	m.eph = gps_raw_int_in.eph;
	m.epv = gps_raw_int_in.epv;
	m.vel = gps_raw_int_in.vel;
	m.cog = gps_raw_int_in.cog;
	m.satellites_visible = gps_raw_int_in.satellites_visible;

                        
                        from_mav_gps_raw_int_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_GPS_STATUS:
                    {
                        mavlink_common::GPS_STATUS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_gps_status_t gps_status_in;
                        memset(&gps_status_in, 0, sizeof(gps_status_in));
                        mavlink_msg_gps_status_decode(&mav_msg, &gps_status_in);
                        
                        	m.satellites_visible = gps_status_in.satellites_visible;
	memcpy(&(m.satellite_prn), &(gps_status_in.satellite_prn), sizeof(uint8_t)*20);
	memcpy(&(m.satellite_used), &(gps_status_in.satellite_used), sizeof(uint8_t)*20);
	memcpy(&(m.satellite_elevation), &(gps_status_in.satellite_elevation), sizeof(uint8_t)*20);
	memcpy(&(m.satellite_azimuth), &(gps_status_in.satellite_azimuth), sizeof(uint8_t)*20);
	memcpy(&(m.satellite_snr), &(gps_status_in.satellite_snr), sizeof(uint8_t)*20);

                        
                        from_mav_gps_status_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SCALED_IMU:
                    {
                        mavlink_common::SCALED_IMU m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_scaled_imu_t scaled_imu_in;
                        memset(&scaled_imu_in, 0, sizeof(scaled_imu_in));
                        mavlink_msg_scaled_imu_decode(&mav_msg, &scaled_imu_in);
                        
                        	m.time_boot_ms = scaled_imu_in.time_boot_ms;
	m.xacc = scaled_imu_in.xacc;
	m.yacc = scaled_imu_in.yacc;
	m.zacc = scaled_imu_in.zacc;
	m.xgyro = scaled_imu_in.xgyro;
	m.ygyro = scaled_imu_in.ygyro;
	m.zgyro = scaled_imu_in.zgyro;
	m.xmag = scaled_imu_in.xmag;
	m.ymag = scaled_imu_in.ymag;
	m.zmag = scaled_imu_in.zmag;

                        
                        from_mav_scaled_imu_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_RAW_IMU:
                    {
                        mavlink_common::RAW_IMU m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_raw_imu_t raw_imu_in;
                        memset(&raw_imu_in, 0, sizeof(raw_imu_in));
                        mavlink_msg_raw_imu_decode(&mav_msg, &raw_imu_in);
                        
                        	m.time_usec = raw_imu_in.time_usec;
	m.xacc = raw_imu_in.xacc;
	m.yacc = raw_imu_in.yacc;
	m.zacc = raw_imu_in.zacc;
	m.xgyro = raw_imu_in.xgyro;
	m.ygyro = raw_imu_in.ygyro;
	m.zgyro = raw_imu_in.zgyro;
	m.xmag = raw_imu_in.xmag;
	m.ymag = raw_imu_in.ymag;
	m.zmag = raw_imu_in.zmag;

                        
                        from_mav_raw_imu_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_RAW_PRESSURE:
                    {
                        mavlink_common::RAW_PRESSURE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_raw_pressure_t raw_pressure_in;
                        memset(&raw_pressure_in, 0, sizeof(raw_pressure_in));
                        mavlink_msg_raw_pressure_decode(&mav_msg, &raw_pressure_in);
                        
                        	m.time_usec = raw_pressure_in.time_usec;
	m.press_abs = raw_pressure_in.press_abs;
	m.press_diff1 = raw_pressure_in.press_diff1;
	m.press_diff2 = raw_pressure_in.press_diff2;
	m.temperature = raw_pressure_in.temperature;

                        
                        from_mav_raw_pressure_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SCALED_PRESSURE:
                    {
                        mavlink_common::SCALED_PRESSURE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_scaled_pressure_t scaled_pressure_in;
                        memset(&scaled_pressure_in, 0, sizeof(scaled_pressure_in));
                        mavlink_msg_scaled_pressure_decode(&mav_msg, &scaled_pressure_in);
                        
                        	m.time_boot_ms = scaled_pressure_in.time_boot_ms;
	m.press_abs = scaled_pressure_in.press_abs;
	m.press_diff = scaled_pressure_in.press_diff;
	m.temperature = scaled_pressure_in.temperature;

                        
                        from_mav_scaled_pressure_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_ATTITUDE:
                    {
                        mavlink_common::ATTITUDE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_attitude_t attitude_in;
                        memset(&attitude_in, 0, sizeof(attitude_in));
                        mavlink_msg_attitude_decode(&mav_msg, &attitude_in);
                        
                        	m.time_boot_ms = attitude_in.time_boot_ms;
	m.roll = attitude_in.roll;
	m.pitch = attitude_in.pitch;
	m.yaw = attitude_in.yaw;
	m.rollspeed = attitude_in.rollspeed;
	m.pitchspeed = attitude_in.pitchspeed;
	m.yawspeed = attitude_in.yawspeed;

                        
                        from_mav_attitude_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                    {
                        mavlink_common::ATTITUDE_QUATERNION m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_attitude_quaternion_t attitude_quaternion_in;
                        memset(&attitude_quaternion_in, 0, sizeof(attitude_quaternion_in));
                        mavlink_msg_attitude_quaternion_decode(&mav_msg, &attitude_quaternion_in);
                        
                        	m.time_boot_ms = attitude_quaternion_in.time_boot_ms;
	m.q1 = attitude_quaternion_in.q1;
	m.q2 = attitude_quaternion_in.q2;
	m.q3 = attitude_quaternion_in.q3;
	m.q4 = attitude_quaternion_in.q4;
	m.rollspeed = attitude_quaternion_in.rollspeed;
	m.pitchspeed = attitude_quaternion_in.pitchspeed;
	m.yawspeed = attitude_quaternion_in.yawspeed;

                        
                        from_mav_attitude_quaternion_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    {
                        mavlink_common::LOCAL_POSITION_NED m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_local_position_ned_t local_position_ned_in;
                        memset(&local_position_ned_in, 0, sizeof(local_position_ned_in));
                        mavlink_msg_local_position_ned_decode(&mav_msg, &local_position_ned_in);
                        
                        	m.time_boot_ms = local_position_ned_in.time_boot_ms;
	m.x = local_position_ned_in.x;
	m.y = local_position_ned_in.y;
	m.z = local_position_ned_in.z;
	m.vx = local_position_ned_in.vx;
	m.vy = local_position_ned_in.vy;
	m.vz = local_position_ned_in.vz;

                        
                        from_mav_local_position_ned_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    {
                        mavlink_common::GLOBAL_POSITION_INT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_global_position_int_t global_position_int_in;
                        memset(&global_position_int_in, 0, sizeof(global_position_int_in));
                        mavlink_msg_global_position_int_decode(&mav_msg, &global_position_int_in);
                        
                        	m.time_boot_ms = global_position_int_in.time_boot_ms;
	m.lat = global_position_int_in.lat;
	m.lon = global_position_int_in.lon;
	m.alt = global_position_int_in.alt;
	m.relative_alt = global_position_int_in.relative_alt;
	m.vx = global_position_int_in.vx;
	m.vy = global_position_int_in.vy;
	m.vz = global_position_int_in.vz;
	m.hdg = global_position_int_in.hdg;

                        
                        from_mav_global_position_int_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
                    {
                        mavlink_common::RC_CHANNELS_SCALED m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_rc_channels_scaled_t rc_channels_scaled_in;
                        memset(&rc_channels_scaled_in, 0, sizeof(rc_channels_scaled_in));
                        mavlink_msg_rc_channels_scaled_decode(&mav_msg, &rc_channels_scaled_in);
                        
                        	m.time_boot_ms = rc_channels_scaled_in.time_boot_ms;
	m.port = rc_channels_scaled_in.port;
	m.chan1_scaled = rc_channels_scaled_in.chan1_scaled;
	m.chan2_scaled = rc_channels_scaled_in.chan2_scaled;
	m.chan3_scaled = rc_channels_scaled_in.chan3_scaled;
	m.chan4_scaled = rc_channels_scaled_in.chan4_scaled;
	m.chan5_scaled = rc_channels_scaled_in.chan5_scaled;
	m.chan6_scaled = rc_channels_scaled_in.chan6_scaled;
	m.chan7_scaled = rc_channels_scaled_in.chan7_scaled;
	m.chan8_scaled = rc_channels_scaled_in.chan8_scaled;
	m.rssi = rc_channels_scaled_in.rssi;

                        
                        from_mav_rc_channels_scaled_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                    {
                        mavlink_common::RC_CHANNELS_RAW m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_rc_channels_raw_t rc_channels_raw_in;
                        memset(&rc_channels_raw_in, 0, sizeof(rc_channels_raw_in));
                        mavlink_msg_rc_channels_raw_decode(&mav_msg, &rc_channels_raw_in);
                        
                        	m.time_boot_ms = rc_channels_raw_in.time_boot_ms;
	m.port = rc_channels_raw_in.port;
	m.chan1_raw = rc_channels_raw_in.chan1_raw;
	m.chan2_raw = rc_channels_raw_in.chan2_raw;
	m.chan3_raw = rc_channels_raw_in.chan3_raw;
	m.chan4_raw = rc_channels_raw_in.chan4_raw;
	m.chan5_raw = rc_channels_raw_in.chan5_raw;
	m.chan6_raw = rc_channels_raw_in.chan6_raw;
	m.chan7_raw = rc_channels_raw_in.chan7_raw;
	m.chan8_raw = rc_channels_raw_in.chan8_raw;
	m.rssi = rc_channels_raw_in.rssi;

                        
                        from_mav_rc_channels_raw_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                    {
                        mavlink_common::SERVO_OUTPUT_RAW m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_servo_output_raw_t servo_output_raw_in;
                        memset(&servo_output_raw_in, 0, sizeof(servo_output_raw_in));
                        mavlink_msg_servo_output_raw_decode(&mav_msg, &servo_output_raw_in);
                        
                        	m.time_usec = servo_output_raw_in.time_usec;
	m.port = servo_output_raw_in.port;
	m.servo1_raw = servo_output_raw_in.servo1_raw;
	m.servo2_raw = servo_output_raw_in.servo2_raw;
	m.servo3_raw = servo_output_raw_in.servo3_raw;
	m.servo4_raw = servo_output_raw_in.servo4_raw;
	m.servo5_raw = servo_output_raw_in.servo5_raw;
	m.servo6_raw = servo_output_raw_in.servo6_raw;
	m.servo7_raw = servo_output_raw_in.servo7_raw;
	m.servo8_raw = servo_output_raw_in.servo8_raw;

                        
                        from_mav_servo_output_raw_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
                    {
                        mavlink_common::MISSION_REQUEST_PARTIAL_LIST m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_request_partial_list_t mission_request_partial_list_in;
                        memset(&mission_request_partial_list_in, 0, sizeof(mission_request_partial_list_in));
                        mavlink_msg_mission_request_partial_list_decode(&mav_msg, &mission_request_partial_list_in);
                        
                        	m.target_system = mission_request_partial_list_in.target_system;
	m.target_component = mission_request_partial_list_in.target_component;
	m.start_index = mission_request_partial_list_in.start_index;
	m.end_index = mission_request_partial_list_in.end_index;

                        
                        from_mav_mission_request_partial_list_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
                    {
                        mavlink_common::MISSION_WRITE_PARTIAL_LIST m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_write_partial_list_t mission_write_partial_list_in;
                        memset(&mission_write_partial_list_in, 0, sizeof(mission_write_partial_list_in));
                        mavlink_msg_mission_write_partial_list_decode(&mav_msg, &mission_write_partial_list_in);
                        
                        	m.target_system = mission_write_partial_list_in.target_system;
	m.target_component = mission_write_partial_list_in.target_component;
	m.start_index = mission_write_partial_list_in.start_index;
	m.end_index = mission_write_partial_list_in.end_index;

                        
                        from_mav_mission_write_partial_list_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_ITEM:
                    {
                        mavlink_common::MISSION_ITEM m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_item_t mission_item_in;
                        memset(&mission_item_in, 0, sizeof(mission_item_in));
                        mavlink_msg_mission_item_decode(&mav_msg, &mission_item_in);
                        
                        	m.target_system = mission_item_in.target_system;
	m.target_component = mission_item_in.target_component;
	m.seq = mission_item_in.seq;
	m.frame = mission_item_in.frame;
	m.command = mission_item_in.command;
	m.current = mission_item_in.current;
	m.autocontinue = mission_item_in.autocontinue;
	m.param1 = mission_item_in.param1;
	m.param2 = mission_item_in.param2;
	m.param3 = mission_item_in.param3;
	m.param4 = mission_item_in.param4;
	m.x = mission_item_in.x;
	m.y = mission_item_in.y;
	m.z = mission_item_in.z;

                        
                        from_mav_mission_item_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_REQUEST:
                    {
                        mavlink_common::MISSION_REQUEST m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_request_t mission_request_in;
                        memset(&mission_request_in, 0, sizeof(mission_request_in));
                        mavlink_msg_mission_request_decode(&mav_msg, &mission_request_in);
                        
                        	m.target_system = mission_request_in.target_system;
	m.target_component = mission_request_in.target_component;
	m.seq = mission_request_in.seq;

                        
                        from_mav_mission_request_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
                    {
                        mavlink_common::MISSION_SET_CURRENT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_set_current_t mission_set_current_in;
                        memset(&mission_set_current_in, 0, sizeof(mission_set_current_in));
                        mavlink_msg_mission_set_current_decode(&mav_msg, &mission_set_current_in);
                        
                        	m.target_system = mission_set_current_in.target_system;
	m.target_component = mission_set_current_in.target_component;
	m.seq = mission_set_current_in.seq;

                        
                        from_mav_mission_set_current_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_CURRENT:
                    {
                        mavlink_common::MISSION_CURRENT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_current_t mission_current_in;
                        memset(&mission_current_in, 0, sizeof(mission_current_in));
                        mavlink_msg_mission_current_decode(&mav_msg, &mission_current_in);
                        
                        	m.seq = mission_current_in.seq;

                        
                        from_mav_mission_current_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                    {
                        mavlink_common::MISSION_REQUEST_LIST m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_request_list_t mission_request_list_in;
                        memset(&mission_request_list_in, 0, sizeof(mission_request_list_in));
                        mavlink_msg_mission_request_list_decode(&mav_msg, &mission_request_list_in);
                        
                        	m.target_system = mission_request_list_in.target_system;
	m.target_component = mission_request_list_in.target_component;

                        
                        from_mav_mission_request_list_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_COUNT:
                    {
                        mavlink_common::MISSION_COUNT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_count_t mission_count_in;
                        memset(&mission_count_in, 0, sizeof(mission_count_in));
                        mavlink_msg_mission_count_decode(&mav_msg, &mission_count_in);
                        
                        	m.target_system = mission_count_in.target_system;
	m.target_component = mission_count_in.target_component;
	m.count = mission_count_in.count;

                        
                        from_mav_mission_count_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                    {
                        mavlink_common::MISSION_CLEAR_ALL m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_clear_all_t mission_clear_all_in;
                        memset(&mission_clear_all_in, 0, sizeof(mission_clear_all_in));
                        mavlink_msg_mission_clear_all_decode(&mav_msg, &mission_clear_all_in);
                        
                        	m.target_system = mission_clear_all_in.target_system;
	m.target_component = mission_clear_all_in.target_component;

                        
                        from_mav_mission_clear_all_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
                    {
                        mavlink_common::MISSION_ITEM_REACHED m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_item_reached_t mission_item_reached_in;
                        memset(&mission_item_reached_in, 0, sizeof(mission_item_reached_in));
                        mavlink_msg_mission_item_reached_decode(&mav_msg, &mission_item_reached_in);
                        
                        	m.seq = mission_item_reached_in.seq;

                        
                        from_mav_mission_item_reached_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MISSION_ACK:
                    {
                        mavlink_common::MISSION_ACK m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mission_ack_t mission_ack_in;
                        memset(&mission_ack_in, 0, sizeof(mission_ack_in));
                        mavlink_msg_mission_ack_decode(&mav_msg, &mission_ack_in);
                        
                        	m.target_system = mission_ack_in.target_system;
	m.target_component = mission_ack_in.target_component;
	m.type = mission_ack_in.type;

                        
                        from_mav_mission_ack_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
                    {
                        mavlink_common::SET_GPS_GLOBAL_ORIGIN m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_set_gps_global_origin_t set_gps_global_origin_in;
                        memset(&set_gps_global_origin_in, 0, sizeof(set_gps_global_origin_in));
                        mavlink_msg_set_gps_global_origin_decode(&mav_msg, &set_gps_global_origin_in);
                        
                        	m.target_system = set_gps_global_origin_in.target_system;
	m.latitude = set_gps_global_origin_in.latitude;
	m.longitude = set_gps_global_origin_in.longitude;
	m.altitude = set_gps_global_origin_in.altitude;

                        
                        from_mav_set_gps_global_origin_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
                    {
                        mavlink_common::GPS_GLOBAL_ORIGIN m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_gps_global_origin_t gps_global_origin_in;
                        memset(&gps_global_origin_in, 0, sizeof(gps_global_origin_in));
                        mavlink_msg_gps_global_origin_decode(&mav_msg, &gps_global_origin_in);
                        
                        	m.latitude = gps_global_origin_in.latitude;
	m.longitude = gps_global_origin_in.longitude;
	m.altitude = gps_global_origin_in.altitude;

                        
                        from_mav_gps_global_origin_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT:
                    {
                        mavlink_common::SET_LOCAL_POSITION_SETPOINT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_set_local_position_setpoint_t set_local_position_setpoint_in;
                        memset(&set_local_position_setpoint_in, 0, sizeof(set_local_position_setpoint_in));
                        mavlink_msg_set_local_position_setpoint_decode(&mav_msg, &set_local_position_setpoint_in);
                        
                        	m.target_system = set_local_position_setpoint_in.target_system;
	m.target_component = set_local_position_setpoint_in.target_component;
	m.coordinate_frame = set_local_position_setpoint_in.coordinate_frame;
	m.x = set_local_position_setpoint_in.x;
	m.y = set_local_position_setpoint_in.y;
	m.z = set_local_position_setpoint_in.z;
	m.yaw = set_local_position_setpoint_in.yaw;

                        
                        from_mav_set_local_position_setpoint_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT:
                    {
                        mavlink_common::LOCAL_POSITION_SETPOINT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_local_position_setpoint_t local_position_setpoint_in;
                        memset(&local_position_setpoint_in, 0, sizeof(local_position_setpoint_in));
                        mavlink_msg_local_position_setpoint_decode(&mav_msg, &local_position_setpoint_in);
                        
                        	m.coordinate_frame = local_position_setpoint_in.coordinate_frame;
	m.x = local_position_setpoint_in.x;
	m.y = local_position_setpoint_in.y;
	m.z = local_position_setpoint_in.z;
	m.yaw = local_position_setpoint_in.yaw;

                        
                        from_mav_local_position_setpoint_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT:
                    {
                        mavlink_common::GLOBAL_POSITION_SETPOINT_INT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_global_position_setpoint_int_t global_position_setpoint_int_in;
                        memset(&global_position_setpoint_int_in, 0, sizeof(global_position_setpoint_int_in));
                        mavlink_msg_global_position_setpoint_int_decode(&mav_msg, &global_position_setpoint_int_in);
                        
                        	m.coordinate_frame = global_position_setpoint_int_in.coordinate_frame;
	m.latitude = global_position_setpoint_int_in.latitude;
	m.longitude = global_position_setpoint_int_in.longitude;
	m.altitude = global_position_setpoint_int_in.altitude;
	m.yaw = global_position_setpoint_int_in.yaw;

                        
                        from_mav_global_position_setpoint_int_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SET_GLOBAL_POSITION_SETPOINT_INT:
                    {
                        mavlink_common::SET_GLOBAL_POSITION_SETPOINT_INT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_set_global_position_setpoint_int_t set_global_position_setpoint_int_in;
                        memset(&set_global_position_setpoint_int_in, 0, sizeof(set_global_position_setpoint_int_in));
                        mavlink_msg_set_global_position_setpoint_int_decode(&mav_msg, &set_global_position_setpoint_int_in);
                        
                        	m.coordinate_frame = set_global_position_setpoint_int_in.coordinate_frame;
	m.latitude = set_global_position_setpoint_int_in.latitude;
	m.longitude = set_global_position_setpoint_int_in.longitude;
	m.altitude = set_global_position_setpoint_int_in.altitude;
	m.yaw = set_global_position_setpoint_int_in.yaw;

                        
                        from_mav_set_global_position_setpoint_int_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA:
                    {
                        mavlink_common::SAFETY_SET_ALLOWED_AREA m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_safety_set_allowed_area_t safety_set_allowed_area_in;
                        memset(&safety_set_allowed_area_in, 0, sizeof(safety_set_allowed_area_in));
                        mavlink_msg_safety_set_allowed_area_decode(&mav_msg, &safety_set_allowed_area_in);
                        
                        	m.target_system = safety_set_allowed_area_in.target_system;
	m.target_component = safety_set_allowed_area_in.target_component;
	m.frame = safety_set_allowed_area_in.frame;
	m.p1x = safety_set_allowed_area_in.p1x;
	m.p1y = safety_set_allowed_area_in.p1y;
	m.p1z = safety_set_allowed_area_in.p1z;
	m.p2x = safety_set_allowed_area_in.p2x;
	m.p2y = safety_set_allowed_area_in.p2y;
	m.p2z = safety_set_allowed_area_in.p2z;

                        
                        from_mav_safety_set_allowed_area_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA:
                    {
                        mavlink_common::SAFETY_ALLOWED_AREA m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_safety_allowed_area_t safety_allowed_area_in;
                        memset(&safety_allowed_area_in, 0, sizeof(safety_allowed_area_in));
                        mavlink_msg_safety_allowed_area_decode(&mav_msg, &safety_allowed_area_in);
                        
                        	m.frame = safety_allowed_area_in.frame;
	m.p1x = safety_allowed_area_in.p1x;
	m.p1y = safety_allowed_area_in.p1y;
	m.p1z = safety_allowed_area_in.p1z;
	m.p2x = safety_allowed_area_in.p2x;
	m.p2y = safety_allowed_area_in.p2y;
	m.p2z = safety_allowed_area_in.p2z;

                        
                        from_mav_safety_allowed_area_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST:
                    {
                        mavlink_common::SET_ROLL_PITCH_YAW_THRUST m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_set_roll_pitch_yaw_thrust_t set_roll_pitch_yaw_thrust_in;
                        memset(&set_roll_pitch_yaw_thrust_in, 0, sizeof(set_roll_pitch_yaw_thrust_in));
                        mavlink_msg_set_roll_pitch_yaw_thrust_decode(&mav_msg, &set_roll_pitch_yaw_thrust_in);
                        
                        	m.target_system = set_roll_pitch_yaw_thrust_in.target_system;
	m.target_component = set_roll_pitch_yaw_thrust_in.target_component;
	m.roll = set_roll_pitch_yaw_thrust_in.roll;
	m.pitch = set_roll_pitch_yaw_thrust_in.pitch;
	m.yaw = set_roll_pitch_yaw_thrust_in.yaw;
	m.thrust = set_roll_pitch_yaw_thrust_in.thrust;

                        
                        from_mav_set_roll_pitch_yaw_thrust_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST:
                    {
                        mavlink_common::SET_ROLL_PITCH_YAW_SPEED_THRUST m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_set_roll_pitch_yaw_speed_thrust_t set_roll_pitch_yaw_speed_thrust_in;
                        memset(&set_roll_pitch_yaw_speed_thrust_in, 0, sizeof(set_roll_pitch_yaw_speed_thrust_in));
                        mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(&mav_msg, &set_roll_pitch_yaw_speed_thrust_in);
                        
                        	m.target_system = set_roll_pitch_yaw_speed_thrust_in.target_system;
	m.target_component = set_roll_pitch_yaw_speed_thrust_in.target_component;
	m.roll_speed = set_roll_pitch_yaw_speed_thrust_in.roll_speed;
	m.pitch_speed = set_roll_pitch_yaw_speed_thrust_in.pitch_speed;
	m.yaw_speed = set_roll_pitch_yaw_speed_thrust_in.yaw_speed;
	m.thrust = set_roll_pitch_yaw_speed_thrust_in.thrust;

                        
                        from_mav_set_roll_pitch_yaw_speed_thrust_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT:
                    {
                        mavlink_common::ROLL_PITCH_YAW_THRUST_SETPOINT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_roll_pitch_yaw_thrust_setpoint_t roll_pitch_yaw_thrust_setpoint_in;
                        memset(&roll_pitch_yaw_thrust_setpoint_in, 0, sizeof(roll_pitch_yaw_thrust_setpoint_in));
                        mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(&mav_msg, &roll_pitch_yaw_thrust_setpoint_in);
                        
                        	m.time_boot_ms = roll_pitch_yaw_thrust_setpoint_in.time_boot_ms;
	m.roll = roll_pitch_yaw_thrust_setpoint_in.roll;
	m.pitch = roll_pitch_yaw_thrust_setpoint_in.pitch;
	m.yaw = roll_pitch_yaw_thrust_setpoint_in.yaw;
	m.thrust = roll_pitch_yaw_thrust_setpoint_in.thrust;

                        
                        from_mav_roll_pitch_yaw_thrust_setpoint_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT:
                    {
                        mavlink_common::ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_roll_pitch_yaw_speed_thrust_setpoint_t roll_pitch_yaw_speed_thrust_setpoint_in;
                        memset(&roll_pitch_yaw_speed_thrust_setpoint_in, 0, sizeof(roll_pitch_yaw_speed_thrust_setpoint_in));
                        mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(&mav_msg, &roll_pitch_yaw_speed_thrust_setpoint_in);
                        
                        	m.time_boot_ms = roll_pitch_yaw_speed_thrust_setpoint_in.time_boot_ms;
	m.roll_speed = roll_pitch_yaw_speed_thrust_setpoint_in.roll_speed;
	m.pitch_speed = roll_pitch_yaw_speed_thrust_setpoint_in.pitch_speed;
	m.yaw_speed = roll_pitch_yaw_speed_thrust_setpoint_in.yaw_speed;
	m.thrust = roll_pitch_yaw_speed_thrust_setpoint_in.thrust;

                        
                        from_mav_roll_pitch_yaw_speed_thrust_setpoint_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT:
                    {
                        mavlink_common::SET_QUAD_MOTORS_SETPOINT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_set_quad_motors_setpoint_t set_quad_motors_setpoint_in;
                        memset(&set_quad_motors_setpoint_in, 0, sizeof(set_quad_motors_setpoint_in));
                        mavlink_msg_set_quad_motors_setpoint_decode(&mav_msg, &set_quad_motors_setpoint_in);
                        
                        	m.target_system = set_quad_motors_setpoint_in.target_system;
	m.motor_front_nw = set_quad_motors_setpoint_in.motor_front_nw;
	m.motor_right_ne = set_quad_motors_setpoint_in.motor_right_ne;
	m.motor_back_se = set_quad_motors_setpoint_in.motor_back_se;
	m.motor_left_sw = set_quad_motors_setpoint_in.motor_left_sw;

                        
                        from_mav_set_quad_motors_setpoint_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST:
                    {
                        mavlink_common::SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t set_quad_swarm_roll_pitch_yaw_thrust_in;
                        memset(&set_quad_swarm_roll_pitch_yaw_thrust_in, 0, sizeof(set_quad_swarm_roll_pitch_yaw_thrust_in));
                        mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(&mav_msg, &set_quad_swarm_roll_pitch_yaw_thrust_in);
                        
                        	m.group = set_quad_swarm_roll_pitch_yaw_thrust_in.group;
	m.mode = set_quad_swarm_roll_pitch_yaw_thrust_in.mode;
	memcpy(&(m.roll), &(set_quad_swarm_roll_pitch_yaw_thrust_in.roll), sizeof(int16_t)*4);
	memcpy(&(m.pitch), &(set_quad_swarm_roll_pitch_yaw_thrust_in.pitch), sizeof(int16_t)*4);
	memcpy(&(m.yaw), &(set_quad_swarm_roll_pitch_yaw_thrust_in.yaw), sizeof(int16_t)*4);
	memcpy(&(m.thrust), &(set_quad_swarm_roll_pitch_yaw_thrust_in.thrust), sizeof(uint16_t)*4);

                        
                        from_mav_set_quad_swarm_roll_pitch_yaw_thrust_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
                    {
                        mavlink_common::NAV_CONTROLLER_OUTPUT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_nav_controller_output_t nav_controller_output_in;
                        memset(&nav_controller_output_in, 0, sizeof(nav_controller_output_in));
                        mavlink_msg_nav_controller_output_decode(&mav_msg, &nav_controller_output_in);
                        
                        	m.nav_roll = nav_controller_output_in.nav_roll;
	m.nav_pitch = nav_controller_output_in.nav_pitch;
	m.nav_bearing = nav_controller_output_in.nav_bearing;
	m.target_bearing = nav_controller_output_in.target_bearing;
	m.wp_dist = nav_controller_output_in.wp_dist;
	m.alt_error = nav_controller_output_in.alt_error;
	m.aspd_error = nav_controller_output_in.aspd_error;
	m.xtrack_error = nav_controller_output_in.xtrack_error;

                        
                        from_mav_nav_controller_output_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST:
                    {
                        mavlink_common::SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_set_quad_swarm_led_roll_pitch_yaw_thrust_t set_quad_swarm_led_roll_pitch_yaw_thrust_in;
                        memset(&set_quad_swarm_led_roll_pitch_yaw_thrust_in, 0, sizeof(set_quad_swarm_led_roll_pitch_yaw_thrust_in));
                        mavlink_msg_set_quad_swarm_led_roll_pitch_yaw_thrust_decode(&mav_msg, &set_quad_swarm_led_roll_pitch_yaw_thrust_in);
                        
                        	m.group = set_quad_swarm_led_roll_pitch_yaw_thrust_in.group;
	m.mode = set_quad_swarm_led_roll_pitch_yaw_thrust_in.mode;
	memcpy(&(m.led_red), &(set_quad_swarm_led_roll_pitch_yaw_thrust_in.led_red), sizeof(uint8_t)*4);
	memcpy(&(m.led_blue), &(set_quad_swarm_led_roll_pitch_yaw_thrust_in.led_blue), sizeof(uint8_t)*4);
	memcpy(&(m.led_green), &(set_quad_swarm_led_roll_pitch_yaw_thrust_in.led_green), sizeof(uint8_t)*4);
	memcpy(&(m.roll), &(set_quad_swarm_led_roll_pitch_yaw_thrust_in.roll), sizeof(int16_t)*4);
	memcpy(&(m.pitch), &(set_quad_swarm_led_roll_pitch_yaw_thrust_in.pitch), sizeof(int16_t)*4);
	memcpy(&(m.yaw), &(set_quad_swarm_led_roll_pitch_yaw_thrust_in.yaw), sizeof(int16_t)*4);
	memcpy(&(m.thrust), &(set_quad_swarm_led_roll_pitch_yaw_thrust_in.thrust), sizeof(uint16_t)*4);

                        
                        from_mav_set_quad_swarm_led_roll_pitch_yaw_thrust_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_STATE_CORRECTION:
                    {
                        mavlink_common::STATE_CORRECTION m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_state_correction_t state_correction_in;
                        memset(&state_correction_in, 0, sizeof(state_correction_in));
                        mavlink_msg_state_correction_decode(&mav_msg, &state_correction_in);
                        
                        	m.xErr = state_correction_in.xErr;
	m.yErr = state_correction_in.yErr;
	m.zErr = state_correction_in.zErr;
	m.rollErr = state_correction_in.rollErr;
	m.pitchErr = state_correction_in.pitchErr;
	m.yawErr = state_correction_in.yawErr;
	m.vxErr = state_correction_in.vxErr;
	m.vyErr = state_correction_in.vyErr;
	m.vzErr = state_correction_in.vzErr;

                        
                        from_mav_state_correction_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
                    {
                        mavlink_common::REQUEST_DATA_STREAM m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_request_data_stream_t request_data_stream_in;
                        memset(&request_data_stream_in, 0, sizeof(request_data_stream_in));
                        mavlink_msg_request_data_stream_decode(&mav_msg, &request_data_stream_in);
                        
                        	m.target_system = request_data_stream_in.target_system;
	m.target_component = request_data_stream_in.target_component;
	m.req_stream_id = request_data_stream_in.req_stream_id;
	m.req_message_rate = request_data_stream_in.req_message_rate;
	m.start_stop = request_data_stream_in.start_stop;

                        
                        from_mav_request_data_stream_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_DATA_STREAM:
                    {
                        mavlink_common::DATA_STREAM m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_data_stream_t data_stream_in;
                        memset(&data_stream_in, 0, sizeof(data_stream_in));
                        mavlink_msg_data_stream_decode(&mav_msg, &data_stream_in);
                        
                        	m.stream_id = data_stream_in.stream_id;
	m.message_rate = data_stream_in.message_rate;
	m.on_off = data_stream_in.on_off;

                        
                        from_mav_data_stream_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MANUAL_CONTROL:
                    {
                        mavlink_common::MANUAL_CONTROL m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_manual_control_t manual_control_in;
                        memset(&manual_control_in, 0, sizeof(manual_control_in));
                        mavlink_msg_manual_control_decode(&mav_msg, &manual_control_in);
                        
                        	m.target = manual_control_in.target;
	m.x = manual_control_in.x;
	m.y = manual_control_in.y;
	m.z = manual_control_in.z;
	m.r = manual_control_in.r;
	m.buttons = manual_control_in.buttons;

                        
                        from_mav_manual_control_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                    {
                        mavlink_common::RC_CHANNELS_OVERRIDE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_rc_channels_override_t rc_channels_override_in;
                        memset(&rc_channels_override_in, 0, sizeof(rc_channels_override_in));
                        mavlink_msg_rc_channels_override_decode(&mav_msg, &rc_channels_override_in);
                        
                        	m.target_system = rc_channels_override_in.target_system;
	m.target_component = rc_channels_override_in.target_component;
	m.chan1_raw = rc_channels_override_in.chan1_raw;
	m.chan2_raw = rc_channels_override_in.chan2_raw;
	m.chan3_raw = rc_channels_override_in.chan3_raw;
	m.chan4_raw = rc_channels_override_in.chan4_raw;
	m.chan5_raw = rc_channels_override_in.chan5_raw;
	m.chan6_raw = rc_channels_override_in.chan6_raw;
	m.chan7_raw = rc_channels_override_in.chan7_raw;
	m.chan8_raw = rc_channels_override_in.chan8_raw;

                        
                        from_mav_rc_channels_override_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_VFR_HUD:
                    {
                        mavlink_common::VFR_HUD m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_vfr_hud_t vfr_hud_in;
                        memset(&vfr_hud_in, 0, sizeof(vfr_hud_in));
                        mavlink_msg_vfr_hud_decode(&mav_msg, &vfr_hud_in);
                        
                        	m.airspeed = vfr_hud_in.airspeed;
	m.groundspeed = vfr_hud_in.groundspeed;
	m.heading = vfr_hud_in.heading;
	m.throttle = vfr_hud_in.throttle;
	m.alt = vfr_hud_in.alt;
	m.climb = vfr_hud_in.climb;

                        
                        from_mav_vfr_hud_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_COMMAND_LONG:
                    {
                        mavlink_common::COMMAND_LONG m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_command_long_t command_long_in;
                        memset(&command_long_in, 0, sizeof(command_long_in));
                        mavlink_msg_command_long_decode(&mav_msg, &command_long_in);
                        
                        	m.target_system = command_long_in.target_system;
	m.target_component = command_long_in.target_component;
	m.command = command_long_in.command;
	m.confirmation = command_long_in.confirmation;
	m.param1 = command_long_in.param1;
	m.param2 = command_long_in.param2;
	m.param3 = command_long_in.param3;
	m.param4 = command_long_in.param4;
	m.param5 = command_long_in.param5;
	m.param6 = command_long_in.param6;
	m.param7 = command_long_in.param7;

                        
                        from_mav_command_long_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_COMMAND_ACK:
                    {
                        mavlink_common::COMMAND_ACK m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_command_ack_t command_ack_in;
                        memset(&command_ack_in, 0, sizeof(command_ack_in));
                        mavlink_msg_command_ack_decode(&mav_msg, &command_ack_in);
                        
                        	m.command = command_ack_in.command;
	m.result = command_ack_in.result;

                        
                        from_mav_command_ack_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_ROLL_PITCH_YAW_RATES_THRUST_SETPOINT:
                    {
                        mavlink_common::ROLL_PITCH_YAW_RATES_THRUST_SETPOINT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_roll_pitch_yaw_rates_thrust_setpoint_t roll_pitch_yaw_rates_thrust_setpoint_in;
                        memset(&roll_pitch_yaw_rates_thrust_setpoint_in, 0, sizeof(roll_pitch_yaw_rates_thrust_setpoint_in));
                        mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_decode(&mav_msg, &roll_pitch_yaw_rates_thrust_setpoint_in);
                        
                        	m.time_boot_ms = roll_pitch_yaw_rates_thrust_setpoint_in.time_boot_ms;
	m.roll_rate = roll_pitch_yaw_rates_thrust_setpoint_in.roll_rate;
	m.pitch_rate = roll_pitch_yaw_rates_thrust_setpoint_in.pitch_rate;
	m.yaw_rate = roll_pitch_yaw_rates_thrust_setpoint_in.yaw_rate;
	m.thrust = roll_pitch_yaw_rates_thrust_setpoint_in.thrust;

                        
                        from_mav_roll_pitch_yaw_rates_thrust_setpoint_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MANUAL_SETPOINT:
                    {
                        mavlink_common::MANUAL_SETPOINT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_manual_setpoint_t manual_setpoint_in;
                        memset(&manual_setpoint_in, 0, sizeof(manual_setpoint_in));
                        mavlink_msg_manual_setpoint_decode(&mav_msg, &manual_setpoint_in);
                        
                        	m.time_boot_ms = manual_setpoint_in.time_boot_ms;
	m.roll = manual_setpoint_in.roll;
	m.pitch = manual_setpoint_in.pitch;
	m.yaw = manual_setpoint_in.yaw;
	m.thrust = manual_setpoint_in.thrust;
	m.mode_switch = manual_setpoint_in.mode_switch;
	m.manual_override_switch = manual_setpoint_in.manual_override_switch;

                        
                        from_mav_manual_setpoint_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET:
                    {
                        mavlink_common::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_local_position_ned_system_global_offset_t local_position_ned_system_global_offset_in;
                        memset(&local_position_ned_system_global_offset_in, 0, sizeof(local_position_ned_system_global_offset_in));
                        mavlink_msg_local_position_ned_system_global_offset_decode(&mav_msg, &local_position_ned_system_global_offset_in);
                        
                        	m.time_boot_ms = local_position_ned_system_global_offset_in.time_boot_ms;
	m.x = local_position_ned_system_global_offset_in.x;
	m.y = local_position_ned_system_global_offset_in.y;
	m.z = local_position_ned_system_global_offset_in.z;
	m.roll = local_position_ned_system_global_offset_in.roll;
	m.pitch = local_position_ned_system_global_offset_in.pitch;
	m.yaw = local_position_ned_system_global_offset_in.yaw;

                        
                        from_mav_local_position_ned_system_global_offset_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_HIL_STATE:
                    {
                        mavlink_common::HIL_STATE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_hil_state_t hil_state_in;
                        memset(&hil_state_in, 0, sizeof(hil_state_in));
                        mavlink_msg_hil_state_decode(&mav_msg, &hil_state_in);
                        
                        	m.time_usec = hil_state_in.time_usec;
	m.roll = hil_state_in.roll;
	m.pitch = hil_state_in.pitch;
	m.yaw = hil_state_in.yaw;
	m.rollspeed = hil_state_in.rollspeed;
	m.pitchspeed = hil_state_in.pitchspeed;
	m.yawspeed = hil_state_in.yawspeed;
	m.lat = hil_state_in.lat;
	m.lon = hil_state_in.lon;
	m.alt = hil_state_in.alt;
	m.vx = hil_state_in.vx;
	m.vy = hil_state_in.vy;
	m.vz = hil_state_in.vz;
	m.xacc = hil_state_in.xacc;
	m.yacc = hil_state_in.yacc;
	m.zacc = hil_state_in.zacc;

                        
                        from_mav_hil_state_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_HIL_CONTROLS:
                    {
                        mavlink_common::HIL_CONTROLS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_hil_controls_t hil_controls_in;
                        memset(&hil_controls_in, 0, sizeof(hil_controls_in));
                        mavlink_msg_hil_controls_decode(&mav_msg, &hil_controls_in);
                        
                        	m.time_usec = hil_controls_in.time_usec;
	m.roll_ailerons = hil_controls_in.roll_ailerons;
	m.pitch_elevator = hil_controls_in.pitch_elevator;
	m.yaw_rudder = hil_controls_in.yaw_rudder;
	m.throttle = hil_controls_in.throttle;
	m.aux1 = hil_controls_in.aux1;
	m.aux2 = hil_controls_in.aux2;
	m.aux3 = hil_controls_in.aux3;
	m.aux4 = hil_controls_in.aux4;
	m.mode = hil_controls_in.mode;
	m.nav_mode = hil_controls_in.nav_mode;

                        
                        from_mav_hil_controls_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW:
                    {
                        mavlink_common::HIL_RC_INPUTS_RAW m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_hil_rc_inputs_raw_t hil_rc_inputs_raw_in;
                        memset(&hil_rc_inputs_raw_in, 0, sizeof(hil_rc_inputs_raw_in));
                        mavlink_msg_hil_rc_inputs_raw_decode(&mav_msg, &hil_rc_inputs_raw_in);
                        
                        	m.time_usec = hil_rc_inputs_raw_in.time_usec;
	m.chan1_raw = hil_rc_inputs_raw_in.chan1_raw;
	m.chan2_raw = hil_rc_inputs_raw_in.chan2_raw;
	m.chan3_raw = hil_rc_inputs_raw_in.chan3_raw;
	m.chan4_raw = hil_rc_inputs_raw_in.chan4_raw;
	m.chan5_raw = hil_rc_inputs_raw_in.chan5_raw;
	m.chan6_raw = hil_rc_inputs_raw_in.chan6_raw;
	m.chan7_raw = hil_rc_inputs_raw_in.chan7_raw;
	m.chan8_raw = hil_rc_inputs_raw_in.chan8_raw;
	m.chan9_raw = hil_rc_inputs_raw_in.chan9_raw;
	m.chan10_raw = hil_rc_inputs_raw_in.chan10_raw;
	m.chan11_raw = hil_rc_inputs_raw_in.chan11_raw;
	m.chan12_raw = hil_rc_inputs_raw_in.chan12_raw;
	m.rssi = hil_rc_inputs_raw_in.rssi;

                        
                        from_mav_hil_rc_inputs_raw_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_OPTICAL_FLOW:
                    {
                        mavlink_common::OPTICAL_FLOW m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_optical_flow_t optical_flow_in;
                        memset(&optical_flow_in, 0, sizeof(optical_flow_in));
                        mavlink_msg_optical_flow_decode(&mav_msg, &optical_flow_in);
                        
                        	m.time_usec = optical_flow_in.time_usec;
	m.sensor_id = optical_flow_in.sensor_id;
	m.flow_x = optical_flow_in.flow_x;
	m.flow_y = optical_flow_in.flow_y;
	m.flow_comp_m_x = optical_flow_in.flow_comp_m_x;
	m.flow_comp_m_y = optical_flow_in.flow_comp_m_y;
	m.quality = optical_flow_in.quality;
	m.ground_distance = optical_flow_in.ground_distance;

                        
                        from_mav_optical_flow_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
                    {
                        mavlink_common::GLOBAL_VISION_POSITION_ESTIMATE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_global_vision_position_estimate_t global_vision_position_estimate_in;
                        memset(&global_vision_position_estimate_in, 0, sizeof(global_vision_position_estimate_in));
                        mavlink_msg_global_vision_position_estimate_decode(&mav_msg, &global_vision_position_estimate_in);
                        
                        	m.usec = global_vision_position_estimate_in.usec;
	m.x = global_vision_position_estimate_in.x;
	m.y = global_vision_position_estimate_in.y;
	m.z = global_vision_position_estimate_in.z;
	m.roll = global_vision_position_estimate_in.roll;
	m.pitch = global_vision_position_estimate_in.pitch;
	m.yaw = global_vision_position_estimate_in.yaw;

                        
                        from_mav_global_vision_position_estimate_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
                    {
                        mavlink_common::VISION_POSITION_ESTIMATE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_vision_position_estimate_t vision_position_estimate_in;
                        memset(&vision_position_estimate_in, 0, sizeof(vision_position_estimate_in));
                        mavlink_msg_vision_position_estimate_decode(&mav_msg, &vision_position_estimate_in);
                        
                        	m.usec = vision_position_estimate_in.usec;
	m.x = vision_position_estimate_in.x;
	m.y = vision_position_estimate_in.y;
	m.z = vision_position_estimate_in.z;
	m.roll = vision_position_estimate_in.roll;
	m.pitch = vision_position_estimate_in.pitch;
	m.yaw = vision_position_estimate_in.yaw;

                        
                        from_mav_vision_position_estimate_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
                    {
                        mavlink_common::VISION_SPEED_ESTIMATE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_vision_speed_estimate_t vision_speed_estimate_in;
                        memset(&vision_speed_estimate_in, 0, sizeof(vision_speed_estimate_in));
                        mavlink_msg_vision_speed_estimate_decode(&mav_msg, &vision_speed_estimate_in);
                        
                        	m.usec = vision_speed_estimate_in.usec;
	m.x = vision_speed_estimate_in.x;
	m.y = vision_speed_estimate_in.y;
	m.z = vision_speed_estimate_in.z;

                        
                        from_mav_vision_speed_estimate_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
                    {
                        mavlink_common::VICON_POSITION_ESTIMATE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_vicon_position_estimate_t vicon_position_estimate_in;
                        memset(&vicon_position_estimate_in, 0, sizeof(vicon_position_estimate_in));
                        mavlink_msg_vicon_position_estimate_decode(&mav_msg, &vicon_position_estimate_in);
                        
                        	m.usec = vicon_position_estimate_in.usec;
	m.x = vicon_position_estimate_in.x;
	m.y = vicon_position_estimate_in.y;
	m.z = vicon_position_estimate_in.z;
	m.roll = vicon_position_estimate_in.roll;
	m.pitch = vicon_position_estimate_in.pitch;
	m.yaw = vicon_position_estimate_in.yaw;

                        
                        from_mav_vicon_position_estimate_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_HIGHRES_IMU:
                    {
                        mavlink_common::HIGHRES_IMU m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_highres_imu_t highres_imu_in;
                        memset(&highres_imu_in, 0, sizeof(highres_imu_in));
                        mavlink_msg_highres_imu_decode(&mav_msg, &highres_imu_in);
                        
                        	m.time_usec = highres_imu_in.time_usec;
	m.xacc = highres_imu_in.xacc;
	m.yacc = highres_imu_in.yacc;
	m.zacc = highres_imu_in.zacc;
	m.xgyro = highres_imu_in.xgyro;
	m.ygyro = highres_imu_in.ygyro;
	m.zgyro = highres_imu_in.zgyro;
	m.xmag = highres_imu_in.xmag;
	m.ymag = highres_imu_in.ymag;
	m.zmag = highres_imu_in.zmag;
	m.abs_pressure = highres_imu_in.abs_pressure;
	m.diff_pressure = highres_imu_in.diff_pressure;
	m.pressure_alt = highres_imu_in.pressure_alt;
	m.temperature = highres_imu_in.temperature;
	m.fields_updated = highres_imu_in.fields_updated;

                        
                        from_mav_highres_imu_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_FILE_TRANSFER_START:
                    {
                        mavlink_common::FILE_TRANSFER_START m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_file_transfer_start_t file_transfer_start_in;
                        memset(&file_transfer_start_in, 0, sizeof(file_transfer_start_in));
                        mavlink_msg_file_transfer_start_decode(&mav_msg, &file_transfer_start_in);
                        
                        	m.transfer_uid = file_transfer_start_in.transfer_uid;
	memcpy(&(m.dest_path), &(file_transfer_start_in.dest_path), sizeof(char)*240);
	m.direction = file_transfer_start_in.direction;
	m.file_size = file_transfer_start_in.file_size;
	m.flags = file_transfer_start_in.flags;

                        
                        from_mav_file_transfer_start_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_FILE_TRANSFER_DIR_LIST:
                    {
                        mavlink_common::FILE_TRANSFER_DIR_LIST m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_file_transfer_dir_list_t file_transfer_dir_list_in;
                        memset(&file_transfer_dir_list_in, 0, sizeof(file_transfer_dir_list_in));
                        mavlink_msg_file_transfer_dir_list_decode(&mav_msg, &file_transfer_dir_list_in);
                        
                        	m.transfer_uid = file_transfer_dir_list_in.transfer_uid;
	memcpy(&(m.dir_path), &(file_transfer_dir_list_in.dir_path), sizeof(char)*240);
	m.flags = file_transfer_dir_list_in.flags;

                        
                        from_mav_file_transfer_dir_list_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_FILE_TRANSFER_RES:
                    {
                        mavlink_common::FILE_TRANSFER_RES m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_file_transfer_res_t file_transfer_res_in;
                        memset(&file_transfer_res_in, 0, sizeof(file_transfer_res_in));
                        mavlink_msg_file_transfer_res_decode(&mav_msg, &file_transfer_res_in);
                        
                        	m.transfer_uid = file_transfer_res_in.transfer_uid;
	m.result = file_transfer_res_in.result;

                        
                        from_mav_file_transfer_res_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_BATTERY_STATUS:
                    {
                        mavlink_common::BATTERY_STATUS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_battery_status_t battery_status_in;
                        memset(&battery_status_in, 0, sizeof(battery_status_in));
                        mavlink_msg_battery_status_decode(&mav_msg, &battery_status_in);
                        
                        	m.accu_id = battery_status_in.accu_id;
	m.voltage_cell_1 = battery_status_in.voltage_cell_1;
	m.voltage_cell_2 = battery_status_in.voltage_cell_2;
	m.voltage_cell_3 = battery_status_in.voltage_cell_3;
	m.voltage_cell_4 = battery_status_in.voltage_cell_4;
	m.voltage_cell_5 = battery_status_in.voltage_cell_5;
	m.voltage_cell_6 = battery_status_in.voltage_cell_6;
	m.current_battery = battery_status_in.current_battery;
	m.battery_remaining = battery_status_in.battery_remaining;

                        
                        from_mav_battery_status_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SETPOINT_8DOF:
                    {
                        mavlink_common::SETPOINT_8DOF m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_setpoint_8dof_t setpoint_8dof_in;
                        memset(&setpoint_8dof_in, 0, sizeof(setpoint_8dof_in));
                        mavlink_msg_setpoint_8dof_decode(&mav_msg, &setpoint_8dof_in);
                        
                        	m.target_system = setpoint_8dof_in.target_system;
	m.val1 = setpoint_8dof_in.val1;
	m.val2 = setpoint_8dof_in.val2;
	m.val3 = setpoint_8dof_in.val3;
	m.val4 = setpoint_8dof_in.val4;
	m.val5 = setpoint_8dof_in.val5;
	m.val6 = setpoint_8dof_in.val6;
	m.val7 = setpoint_8dof_in.val7;
	m.val8 = setpoint_8dof_in.val8;

                        
                        from_mav_setpoint_8dof_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SETPOINT_6DOF:
                    {
                        mavlink_common::SETPOINT_6DOF m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_setpoint_6dof_t setpoint_6dof_in;
                        memset(&setpoint_6dof_in, 0, sizeof(setpoint_6dof_in));
                        mavlink_msg_setpoint_6dof_decode(&mav_msg, &setpoint_6dof_in);
                        
                        	m.target_system = setpoint_6dof_in.target_system;
	m.trans_x = setpoint_6dof_in.trans_x;
	m.trans_y = setpoint_6dof_in.trans_y;
	m.trans_z = setpoint_6dof_in.trans_z;
	m.rot_x = setpoint_6dof_in.rot_x;
	m.rot_y = setpoint_6dof_in.rot_y;
	m.rot_z = setpoint_6dof_in.rot_z;

                        
                        from_mav_setpoint_6dof_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MEMORY_VECT:
                    {
                        mavlink_common::MEMORY_VECT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_memory_vect_t memory_vect_in;
                        memset(&memory_vect_in, 0, sizeof(memory_vect_in));
                        mavlink_msg_memory_vect_decode(&mav_msg, &memory_vect_in);
                        
                        	m.address = memory_vect_in.address;
	m.ver = memory_vect_in.ver;
	m.type = memory_vect_in.type;
	memcpy(&(m.value), &(memory_vect_in.value), sizeof(int8_t)*32);

                        
                        from_mav_memory_vect_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_DEBUG_VECT:
                    {
                        mavlink_common::DEBUG_VECT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_debug_vect_t debug_vect_in;
                        memset(&debug_vect_in, 0, sizeof(debug_vect_in));
                        mavlink_msg_debug_vect_decode(&mav_msg, &debug_vect_in);
                        
                        	memcpy(&(m.name), &(debug_vect_in.name), sizeof(char)*10);
	m.time_usec = debug_vect_in.time_usec;
	m.x = debug_vect_in.x;
	m.y = debug_vect_in.y;
	m.z = debug_vect_in.z;

                        
                        from_mav_debug_vect_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
                    {
                        mavlink_common::NAMED_VALUE_FLOAT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_named_value_float_t named_value_float_in;
                        memset(&named_value_float_in, 0, sizeof(named_value_float_in));
                        mavlink_msg_named_value_float_decode(&mav_msg, &named_value_float_in);
                        
                        	m.time_boot_ms = named_value_float_in.time_boot_ms;
	memcpy(&(m.name), &(named_value_float_in.name), sizeof(char)*10);
	m.value = named_value_float_in.value;

                        
                        from_mav_named_value_float_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
                    {
                        mavlink_common::NAMED_VALUE_INT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_named_value_int_t named_value_int_in;
                        memset(&named_value_int_in, 0, sizeof(named_value_int_in));
                        mavlink_msg_named_value_int_decode(&mav_msg, &named_value_int_in);
                        
                        	m.time_boot_ms = named_value_int_in.time_boot_ms;
	memcpy(&(m.name), &(named_value_int_in.name), sizeof(char)*10);
	m.value = named_value_int_in.value;

                        
                        from_mav_named_value_int_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_STATUSTEXT:
                    {
                        mavlink_common::STATUSTEXT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_statustext_t statustext_in;
                        memset(&statustext_in, 0, sizeof(statustext_in));
                        mavlink_msg_statustext_decode(&mav_msg, &statustext_in);
                        
                        	m.severity = statustext_in.severity;
	memcpy(&(m.text), &(statustext_in.text), sizeof(char)*50);

                        
                        from_mav_statustext_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_DEBUG:
                    {
                        mavlink_common::DEBUG m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_debug_t debug_in;
                        memset(&debug_in, 0, sizeof(debug_in));
                        mavlink_msg_debug_decode(&mav_msg, &debug_in);
                        
                        	m.time_boot_ms = debug_in.time_boot_ms;
	m.ind = debug_in.ind;
	m.value = debug_in.value;

                        
                        from_mav_debug_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SENSOR_OFFSETS:
                    {
                        mavlink_ardupilotmega::SENSOR_OFFSETS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_sensor_offsets_t sensor_offsets_in;
                        memset(&sensor_offsets_in, 0, sizeof(sensor_offsets_in));
                        mavlink_msg_sensor_offsets_decode(&mav_msg, &sensor_offsets_in);
                        
                        	m.mag_ofs_x = sensor_offsets_in.mag_ofs_x;
	m.mag_ofs_y = sensor_offsets_in.mag_ofs_y;
	m.mag_ofs_z = sensor_offsets_in.mag_ofs_z;
	m.mag_declination = sensor_offsets_in.mag_declination;
	m.raw_press = sensor_offsets_in.raw_press;
	m.raw_temp = sensor_offsets_in.raw_temp;
	m.gyro_cal_x = sensor_offsets_in.gyro_cal_x;
	m.gyro_cal_y = sensor_offsets_in.gyro_cal_y;
	m.gyro_cal_z = sensor_offsets_in.gyro_cal_z;
	m.accel_cal_x = sensor_offsets_in.accel_cal_x;
	m.accel_cal_y = sensor_offsets_in.accel_cal_y;
	m.accel_cal_z = sensor_offsets_in.accel_cal_z;

                        
                        from_mav_sensor_offsets_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
                    {
                        mavlink_ardupilotmega::SET_MAG_OFFSETS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_set_mag_offsets_t set_mag_offsets_in;
                        memset(&set_mag_offsets_in, 0, sizeof(set_mag_offsets_in));
                        mavlink_msg_set_mag_offsets_decode(&mav_msg, &set_mag_offsets_in);
                        
                        	m.target_system = set_mag_offsets_in.target_system;
	m.target_component = set_mag_offsets_in.target_component;
	m.mag_ofs_x = set_mag_offsets_in.mag_ofs_x;
	m.mag_ofs_y = set_mag_offsets_in.mag_ofs_y;
	m.mag_ofs_z = set_mag_offsets_in.mag_ofs_z;

                        
                        from_mav_set_mag_offsets_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MEMINFO:
                    {
                        mavlink_ardupilotmega::MEMINFO m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_meminfo_t meminfo_in;
                        memset(&meminfo_in, 0, sizeof(meminfo_in));
                        mavlink_msg_meminfo_decode(&mav_msg, &meminfo_in);
                        
                        	m.brkval = meminfo_in.brkval;
	m.freemem = meminfo_in.freemem;

                        
                        from_mav_meminfo_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_AP_ADC:
                    {
                        mavlink_ardupilotmega::AP_ADC m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_ap_adc_t ap_adc_in;
                        memset(&ap_adc_in, 0, sizeof(ap_adc_in));
                        mavlink_msg_ap_adc_decode(&mav_msg, &ap_adc_in);
                        
                        	m.adc1 = ap_adc_in.adc1;
	m.adc2 = ap_adc_in.adc2;
	m.adc3 = ap_adc_in.adc3;
	m.adc4 = ap_adc_in.adc4;
	m.adc5 = ap_adc_in.adc5;
	m.adc6 = ap_adc_in.adc6;

                        
                        from_mav_ap_adc_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
                    {
                        mavlink_ardupilotmega::DIGICAM_CONFIGURE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_digicam_configure_t digicam_configure_in;
                        memset(&digicam_configure_in, 0, sizeof(digicam_configure_in));
                        mavlink_msg_digicam_configure_decode(&mav_msg, &digicam_configure_in);
                        
                        	m.target_system = digicam_configure_in.target_system;
	m.target_component = digicam_configure_in.target_component;
	m.mode = digicam_configure_in.mode;
	m.shutter_speed = digicam_configure_in.shutter_speed;
	m.aperture = digicam_configure_in.aperture;
	m.iso = digicam_configure_in.iso;
	m.exposure_type = digicam_configure_in.exposure_type;
	m.command_id = digicam_configure_in.command_id;
	m.engine_cut_off = digicam_configure_in.engine_cut_off;
	m.extra_param = digicam_configure_in.extra_param;
	m.extra_value = digicam_configure_in.extra_value;

                        
                        from_mav_digicam_configure_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
                    {
                        mavlink_ardupilotmega::DIGICAM_CONTROL m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_digicam_control_t digicam_control_in;
                        memset(&digicam_control_in, 0, sizeof(digicam_control_in));
                        mavlink_msg_digicam_control_decode(&mav_msg, &digicam_control_in);
                        
                        	m.target_system = digicam_control_in.target_system;
	m.target_component = digicam_control_in.target_component;
	m.session = digicam_control_in.session;
	m.zoom_pos = digicam_control_in.zoom_pos;
	m.zoom_step = digicam_control_in.zoom_step;
	m.focus_lock = digicam_control_in.focus_lock;
	m.shot = digicam_control_in.shot;
	m.command_id = digicam_control_in.command_id;
	m.extra_param = digicam_control_in.extra_param;
	m.extra_value = digicam_control_in.extra_value;

                        
                        from_mav_digicam_control_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
                    {
                        mavlink_ardupilotmega::MOUNT_CONFIGURE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mount_configure_t mount_configure_in;
                        memset(&mount_configure_in, 0, sizeof(mount_configure_in));
                        mavlink_msg_mount_configure_decode(&mav_msg, &mount_configure_in);
                        
                        	m.target_system = mount_configure_in.target_system;
	m.target_component = mount_configure_in.target_component;
	m.mount_mode = mount_configure_in.mount_mode;
	m.stab_roll = mount_configure_in.stab_roll;
	m.stab_pitch = mount_configure_in.stab_pitch;
	m.stab_yaw = mount_configure_in.stab_yaw;

                        
                        from_mav_mount_configure_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MOUNT_CONTROL:
                    {
                        mavlink_ardupilotmega::MOUNT_CONTROL m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mount_control_t mount_control_in;
                        memset(&mount_control_in, 0, sizeof(mount_control_in));
                        mavlink_msg_mount_control_decode(&mav_msg, &mount_control_in);
                        
                        	m.target_system = mount_control_in.target_system;
	m.target_component = mount_control_in.target_component;
	m.input_a = mount_control_in.input_a;
	m.input_b = mount_control_in.input_b;
	m.input_c = mount_control_in.input_c;
	m.save_position = mount_control_in.save_position;

                        
                        from_mav_mount_control_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_MOUNT_STATUS:
                    {
                        mavlink_ardupilotmega::MOUNT_STATUS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_mount_status_t mount_status_in;
                        memset(&mount_status_in, 0, sizeof(mount_status_in));
                        mavlink_msg_mount_status_decode(&mav_msg, &mount_status_in);
                        
                        	m.target_system = mount_status_in.target_system;
	m.target_component = mount_status_in.target_component;
	m.pointing_a = mount_status_in.pointing_a;
	m.pointing_b = mount_status_in.pointing_b;
	m.pointing_c = mount_status_in.pointing_c;

                        
                        from_mav_mount_status_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_FENCE_POINT:
                    {
                        mavlink_ardupilotmega::FENCE_POINT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_fence_point_t fence_point_in;
                        memset(&fence_point_in, 0, sizeof(fence_point_in));
                        mavlink_msg_fence_point_decode(&mav_msg, &fence_point_in);
                        
                        	m.target_system = fence_point_in.target_system;
	m.target_component = fence_point_in.target_component;
	m.idx = fence_point_in.idx;
	m.count = fence_point_in.count;
	m.lat = fence_point_in.lat;
	m.lng = fence_point_in.lng;

                        
                        from_mav_fence_point_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
                    {
                        mavlink_ardupilotmega::FENCE_FETCH_POINT m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_fence_fetch_point_t fence_fetch_point_in;
                        memset(&fence_fetch_point_in, 0, sizeof(fence_fetch_point_in));
                        mavlink_msg_fence_fetch_point_decode(&mav_msg, &fence_fetch_point_in);
                        
                        	m.target_system = fence_fetch_point_in.target_system;
	m.target_component = fence_fetch_point_in.target_component;
	m.idx = fence_fetch_point_in.idx;

                        
                        from_mav_fence_fetch_point_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_FENCE_STATUS:
                    {
                        mavlink_ardupilotmega::FENCE_STATUS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_fence_status_t fence_status_in;
                        memset(&fence_status_in, 0, sizeof(fence_status_in));
                        mavlink_msg_fence_status_decode(&mav_msg, &fence_status_in);
                        
                        	m.breach_status = fence_status_in.breach_status;
	m.breach_count = fence_status_in.breach_count;
	m.breach_type = fence_status_in.breach_type;
	m.breach_time = fence_status_in.breach_time;

                        
                        from_mav_fence_status_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_AHRS:
                    {
                        mavlink_ardupilotmega::AHRS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_ahrs_t ahrs_in;
                        memset(&ahrs_in, 0, sizeof(ahrs_in));
                        mavlink_msg_ahrs_decode(&mav_msg, &ahrs_in);
                        
                        	m.omegaIx = ahrs_in.omegaIx;
	m.omegaIy = ahrs_in.omegaIy;
	m.omegaIz = ahrs_in.omegaIz;
	m.accel_weight = ahrs_in.accel_weight;
	m.renorm_val = ahrs_in.renorm_val;
	m.error_rp = ahrs_in.error_rp;
	m.error_yaw = ahrs_in.error_yaw;

                        
                        from_mav_ahrs_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_SIMSTATE:
                    {
                        mavlink_ardupilotmega::SIMSTATE m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_simstate_t simstate_in;
                        memset(&simstate_in, 0, sizeof(simstate_in));
                        mavlink_msg_simstate_decode(&mav_msg, &simstate_in);
                        
                        	m.roll = simstate_in.roll;
	m.pitch = simstate_in.pitch;
	m.yaw = simstate_in.yaw;
	m.xacc = simstate_in.xacc;
	m.yacc = simstate_in.yacc;
	m.zacc = simstate_in.zacc;
	m.xgyro = simstate_in.xgyro;
	m.ygyro = simstate_in.ygyro;
	m.zgyro = simstate_in.zgyro;
	m.lat = simstate_in.lat;
	m.lng = simstate_in.lng;

                        
                        from_mav_simstate_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_HWSTATUS:
                    {
                        mavlink_ardupilotmega::HWSTATUS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_hwstatus_t hwstatus_in;
                        memset(&hwstatus_in, 0, sizeof(hwstatus_in));
                        mavlink_msg_hwstatus_decode(&mav_msg, &hwstatus_in);
                        
                        	m.Vcc = hwstatus_in.Vcc;
	m.I2Cerr = hwstatus_in.I2Cerr;

                        
                        from_mav_hwstatus_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_RADIO:
                    {
                        mavlink_ardupilotmega::RADIO m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_radio_t radio_in;
                        memset(&radio_in, 0, sizeof(radio_in));
                        mavlink_msg_radio_decode(&mav_msg, &radio_in);
                        
                        	m.rssi = radio_in.rssi;
	m.remrssi = radio_in.remrssi;
	m.txbuf = radio_in.txbuf;
	m.noise = radio_in.noise;
	m.remnoise = radio_in.remnoise;
	m.rxerrors = radio_in.rxerrors;
	m.fixed = radio_in.fixed;

                        
                        from_mav_radio_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_LIMITS_STATUS:
                    {
                        mavlink_ardupilotmega::LIMITS_STATUS m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_limits_status_t limits_status_in;
                        memset(&limits_status_in, 0, sizeof(limits_status_in));
                        mavlink_msg_limits_status_decode(&mav_msg, &limits_status_in);
                        
                        	m.limits_state = limits_status_in.limits_state;
	m.last_trigger = limits_status_in.last_trigger;
	m.last_action = limits_status_in.last_action;
	m.last_recovery = limits_status_in.last_recovery;
	m.last_clear = limits_status_in.last_clear;
	m.breach_count = limits_status_in.breach_count;
	m.mods_enabled = limits_status_in.mods_enabled;
	m.mods_required = limits_status_in.mods_required;
	m.mods_triggered = limits_status_in.mods_triggered;

                        
                        from_mav_limits_status_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_WIND:
                    {
                        mavlink_ardupilotmega::WIND m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_wind_t wind_in;
                        memset(&wind_in, 0, sizeof(wind_in));
                        mavlink_msg_wind_decode(&mav_msg, &wind_in);
                        
                        	m.direction = wind_in.direction;
	m.speed = wind_in.speed;
	m.speed_z = wind_in.speed_z;

                        
                        from_mav_wind_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_DATA16:
                    {
                        mavlink_ardupilotmega::DATA16 m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_data16_t data16_in;
                        memset(&data16_in, 0, sizeof(data16_in));
                        mavlink_msg_data16_decode(&mav_msg, &data16_in);
                        
                        	m.type = data16_in.type;
	m.len = data16_in.len;
	memcpy(&(m.data), &(data16_in.data), sizeof(uint8_t)*16);

                        
                        from_mav_data16_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_DATA32:
                    {
                        mavlink_ardupilotmega::DATA32 m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_data32_t data32_in;
                        memset(&data32_in, 0, sizeof(data32_in));
                        mavlink_msg_data32_decode(&mav_msg, &data32_in);
                        
                        	m.type = data32_in.type;
	m.len = data32_in.len;
	memcpy(&(m.data), &(data32_in.data), sizeof(uint8_t)*32);

                        
                        from_mav_data32_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_DATA64:
                    {
                        mavlink_ardupilotmega::DATA64 m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_data64_t data64_in;
                        memset(&data64_in, 0, sizeof(data64_in));
                        mavlink_msg_data64_decode(&mav_msg, &data64_in);
                        
                        	m.type = data64_in.type;
	m.len = data64_in.len;
	memcpy(&(m.data), &(data64_in.data), sizeof(uint8_t)*64);

                        
                        from_mav_data64_pub.publish(m);     
                    }
                    break;
                    case MAVLINK_MSG_ID_DATA96:
                    {
                        mavlink_ardupilotmega::DATA96 m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_data96_t data96_in;
                        memset(&data96_in, 0, sizeof(data96_in));
                        mavlink_msg_data96_decode(&mav_msg, &data96_in);
                        
                        	m.type = data96_in.type;
	m.len = data96_in.len;
	memcpy(&(m.data), &(data96_in.data), sizeof(uint8_t)*96);

                        
                        from_mav_data96_pub.publish(m);     
                    }
                    break;
                default:
                    //Do nothing
                    break;
            }
        }
    }
}

 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavlink_ardupilotmega_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    to_mav_raw_publisher     = n.advertise<mavlink_ardupilotmega::MAV_RAW>("/to_mav_raw", 10);
    from_mav_raw_subscriber  = n.subscribe("/from_mav_raw", 10, from_mav_raw_callback);

    /**
     * Messages Publishers Initialization
     */
	from_mav_heartbeat_pub = n.advertise<mavlink_common::HEARTBEAT>("/from_mav_heartbeat", 10);
	from_mav_sys_status_pub = n.advertise<mavlink_common::SYS_STATUS>("/from_mav_sys_status", 10);
	from_mav_system_time_pub = n.advertise<mavlink_common::SYSTEM_TIME>("/from_mav_system_time", 10);
	from_mav_ping_pub = n.advertise<mavlink_common::PING>("/from_mav_ping", 10);
	from_mav_change_operator_control_pub = n.advertise<mavlink_common::CHANGE_OPERATOR_CONTROL>("/from_mav_change_operator_control", 10);
	from_mav_change_operator_control_ack_pub = n.advertise<mavlink_common::CHANGE_OPERATOR_CONTROL_ACK>("/from_mav_change_operator_control_ack", 10);
	from_mav_auth_key_pub = n.advertise<mavlink_common::AUTH_KEY>("/from_mav_auth_key", 10);
	from_mav_set_mode_pub = n.advertise<mavlink_common::SET_MODE>("/from_mav_set_mode", 10);
	from_mav_param_request_read_pub = n.advertise<mavlink_common::PARAM_REQUEST_READ>("/from_mav_param_request_read", 10);
	from_mav_param_request_list_pub = n.advertise<mavlink_common::PARAM_REQUEST_LIST>("/from_mav_param_request_list", 10);
	from_mav_param_value_pub = n.advertise<mavlink_common::PARAM_VALUE>("/from_mav_param_value", 10);
	from_mav_param_set_pub = n.advertise<mavlink_common::PARAM_SET>("/from_mav_param_set", 10);
	from_mav_gps_raw_int_pub = n.advertise<mavlink_common::GPS_RAW_INT>("/from_mav_gps_raw_int", 10);
	from_mav_gps_status_pub = n.advertise<mavlink_common::GPS_STATUS>("/from_mav_gps_status", 10);
	from_mav_scaled_imu_pub = n.advertise<mavlink_common::SCALED_IMU>("/from_mav_scaled_imu", 10);
	from_mav_raw_imu_pub = n.advertise<mavlink_common::RAW_IMU>("/from_mav_raw_imu", 10);
	from_mav_raw_pressure_pub = n.advertise<mavlink_common::RAW_PRESSURE>("/from_mav_raw_pressure", 10);
	from_mav_scaled_pressure_pub = n.advertise<mavlink_common::SCALED_PRESSURE>("/from_mav_scaled_pressure", 10);
	from_mav_attitude_pub = n.advertise<mavlink_common::ATTITUDE>("/from_mav_attitude", 10);
	from_mav_attitude_quaternion_pub = n.advertise<mavlink_common::ATTITUDE_QUATERNION>("/from_mav_attitude_quaternion", 10);
	from_mav_local_position_ned_pub = n.advertise<mavlink_common::LOCAL_POSITION_NED>("/from_mav_local_position_ned", 10);
	from_mav_global_position_int_pub = n.advertise<mavlink_common::GLOBAL_POSITION_INT>("/from_mav_global_position_int", 10);
	from_mav_rc_channels_scaled_pub = n.advertise<mavlink_common::RC_CHANNELS_SCALED>("/from_mav_rc_channels_scaled", 10);
	from_mav_rc_channels_raw_pub = n.advertise<mavlink_common::RC_CHANNELS_RAW>("/from_mav_rc_channels_raw", 10);
	from_mav_servo_output_raw_pub = n.advertise<mavlink_common::SERVO_OUTPUT_RAW>("/from_mav_servo_output_raw", 10);
	from_mav_mission_request_partial_list_pub = n.advertise<mavlink_common::MISSION_REQUEST_PARTIAL_LIST>("/from_mav_mission_request_partial_list", 10);
	from_mav_mission_write_partial_list_pub = n.advertise<mavlink_common::MISSION_WRITE_PARTIAL_LIST>("/from_mav_mission_write_partial_list", 10);
	from_mav_mission_item_pub = n.advertise<mavlink_common::MISSION_ITEM>("/from_mav_mission_item", 10);
	from_mav_mission_request_pub = n.advertise<mavlink_common::MISSION_REQUEST>("/from_mav_mission_request", 10);
	from_mav_mission_set_current_pub = n.advertise<mavlink_common::MISSION_SET_CURRENT>("/from_mav_mission_set_current", 10);
	from_mav_mission_current_pub = n.advertise<mavlink_common::MISSION_CURRENT>("/from_mav_mission_current", 10);
	from_mav_mission_request_list_pub = n.advertise<mavlink_common::MISSION_REQUEST_LIST>("/from_mav_mission_request_list", 10);
	from_mav_mission_count_pub = n.advertise<mavlink_common::MISSION_COUNT>("/from_mav_mission_count", 10);
	from_mav_mission_clear_all_pub = n.advertise<mavlink_common::MISSION_CLEAR_ALL>("/from_mav_mission_clear_all", 10);
	from_mav_mission_item_reached_pub = n.advertise<mavlink_common::MISSION_ITEM_REACHED>("/from_mav_mission_item_reached", 10);
	from_mav_mission_ack_pub = n.advertise<mavlink_common::MISSION_ACK>("/from_mav_mission_ack", 10);
	from_mav_set_gps_global_origin_pub = n.advertise<mavlink_common::SET_GPS_GLOBAL_ORIGIN>("/from_mav_set_gps_global_origin", 10);
	from_mav_gps_global_origin_pub = n.advertise<mavlink_common::GPS_GLOBAL_ORIGIN>("/from_mav_gps_global_origin", 10);
	from_mav_set_local_position_setpoint_pub = n.advertise<mavlink_common::SET_LOCAL_POSITION_SETPOINT>("/from_mav_set_local_position_setpoint", 10);
	from_mav_local_position_setpoint_pub = n.advertise<mavlink_common::LOCAL_POSITION_SETPOINT>("/from_mav_local_position_setpoint", 10);
	from_mav_global_position_setpoint_int_pub = n.advertise<mavlink_common::GLOBAL_POSITION_SETPOINT_INT>("/from_mav_global_position_setpoint_int", 10);
	from_mav_set_global_position_setpoint_int_pub = n.advertise<mavlink_common::SET_GLOBAL_POSITION_SETPOINT_INT>("/from_mav_set_global_position_setpoint_int", 10);
	from_mav_safety_set_allowed_area_pub = n.advertise<mavlink_common::SAFETY_SET_ALLOWED_AREA>("/from_mav_safety_set_allowed_area", 10);
	from_mav_safety_allowed_area_pub = n.advertise<mavlink_common::SAFETY_ALLOWED_AREA>("/from_mav_safety_allowed_area", 10);
	from_mav_set_roll_pitch_yaw_thrust_pub = n.advertise<mavlink_common::SET_ROLL_PITCH_YAW_THRUST>("/from_mav_set_roll_pitch_yaw_thrust", 10);
	from_mav_set_roll_pitch_yaw_speed_thrust_pub = n.advertise<mavlink_common::SET_ROLL_PITCH_YAW_SPEED_THRUST>("/from_mav_set_roll_pitch_yaw_speed_thrust", 10);
	from_mav_roll_pitch_yaw_thrust_setpoint_pub = n.advertise<mavlink_common::ROLL_PITCH_YAW_THRUST_SETPOINT>("/from_mav_roll_pitch_yaw_thrust_setpoint", 10);
	from_mav_roll_pitch_yaw_speed_thrust_setpoint_pub = n.advertise<mavlink_common::ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT>("/from_mav_roll_pitch_yaw_speed_thrust_setpoint", 10);
	from_mav_set_quad_motors_setpoint_pub = n.advertise<mavlink_common::SET_QUAD_MOTORS_SETPOINT>("/from_mav_set_quad_motors_setpoint", 10);
	from_mav_set_quad_swarm_roll_pitch_yaw_thrust_pub = n.advertise<mavlink_common::SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST>("/from_mav_set_quad_swarm_roll_pitch_yaw_thrust", 10);
	from_mav_nav_controller_output_pub = n.advertise<mavlink_common::NAV_CONTROLLER_OUTPUT>("/from_mav_nav_controller_output", 10);
	from_mav_set_quad_swarm_led_roll_pitch_yaw_thrust_pub = n.advertise<mavlink_common::SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST>("/from_mav_set_quad_swarm_led_roll_pitch_yaw_thrust", 10);
	from_mav_state_correction_pub = n.advertise<mavlink_common::STATE_CORRECTION>("/from_mav_state_correction", 10);
	from_mav_request_data_stream_pub = n.advertise<mavlink_common::REQUEST_DATA_STREAM>("/from_mav_request_data_stream", 10);
	from_mav_data_stream_pub = n.advertise<mavlink_common::DATA_STREAM>("/from_mav_data_stream", 10);
	from_mav_manual_control_pub = n.advertise<mavlink_common::MANUAL_CONTROL>("/from_mav_manual_control", 10);
	from_mav_rc_channels_override_pub = n.advertise<mavlink_common::RC_CHANNELS_OVERRIDE>("/from_mav_rc_channels_override", 10);
	from_mav_vfr_hud_pub = n.advertise<mavlink_common::VFR_HUD>("/from_mav_vfr_hud", 10);
	from_mav_command_long_pub = n.advertise<mavlink_common::COMMAND_LONG>("/from_mav_command_long", 10);
	from_mav_command_ack_pub = n.advertise<mavlink_common::COMMAND_ACK>("/from_mav_command_ack", 10);
	from_mav_roll_pitch_yaw_rates_thrust_setpoint_pub = n.advertise<mavlink_common::ROLL_PITCH_YAW_RATES_THRUST_SETPOINT>("/from_mav_roll_pitch_yaw_rates_thrust_setpoint", 10);
	from_mav_manual_setpoint_pub = n.advertise<mavlink_common::MANUAL_SETPOINT>("/from_mav_manual_setpoint", 10);
	from_mav_local_position_ned_system_global_offset_pub = n.advertise<mavlink_common::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET>("/from_mav_local_position_ned_system_global_offset", 10);
	from_mav_hil_state_pub = n.advertise<mavlink_common::HIL_STATE>("/from_mav_hil_state", 10);
	from_mav_hil_controls_pub = n.advertise<mavlink_common::HIL_CONTROLS>("/from_mav_hil_controls", 10);
	from_mav_hil_rc_inputs_raw_pub = n.advertise<mavlink_common::HIL_RC_INPUTS_RAW>("/from_mav_hil_rc_inputs_raw", 10);
	from_mav_optical_flow_pub = n.advertise<mavlink_common::OPTICAL_FLOW>("/from_mav_optical_flow", 10);
	from_mav_global_vision_position_estimate_pub = n.advertise<mavlink_common::GLOBAL_VISION_POSITION_ESTIMATE>("/from_mav_global_vision_position_estimate", 10);
	from_mav_vision_position_estimate_pub = n.advertise<mavlink_common::VISION_POSITION_ESTIMATE>("/from_mav_vision_position_estimate", 10);
	from_mav_vision_speed_estimate_pub = n.advertise<mavlink_common::VISION_SPEED_ESTIMATE>("/from_mav_vision_speed_estimate", 10);
	from_mav_vicon_position_estimate_pub = n.advertise<mavlink_common::VICON_POSITION_ESTIMATE>("/from_mav_vicon_position_estimate", 10);
	from_mav_highres_imu_pub = n.advertise<mavlink_common::HIGHRES_IMU>("/from_mav_highres_imu", 10);
	from_mav_file_transfer_start_pub = n.advertise<mavlink_common::FILE_TRANSFER_START>("/from_mav_file_transfer_start", 10);
	from_mav_file_transfer_dir_list_pub = n.advertise<mavlink_common::FILE_TRANSFER_DIR_LIST>("/from_mav_file_transfer_dir_list", 10);
	from_mav_file_transfer_res_pub = n.advertise<mavlink_common::FILE_TRANSFER_RES>("/from_mav_file_transfer_res", 10);
	from_mav_battery_status_pub = n.advertise<mavlink_common::BATTERY_STATUS>("/from_mav_battery_status", 10);
	from_mav_setpoint_8dof_pub = n.advertise<mavlink_common::SETPOINT_8DOF>("/from_mav_setpoint_8dof", 10);
	from_mav_setpoint_6dof_pub = n.advertise<mavlink_common::SETPOINT_6DOF>("/from_mav_setpoint_6dof", 10);
	from_mav_memory_vect_pub = n.advertise<mavlink_common::MEMORY_VECT>("/from_mav_memory_vect", 10);
	from_mav_debug_vect_pub = n.advertise<mavlink_common::DEBUG_VECT>("/from_mav_debug_vect", 10);
	from_mav_named_value_float_pub = n.advertise<mavlink_common::NAMED_VALUE_FLOAT>("/from_mav_named_value_float", 10);
	from_mav_named_value_int_pub = n.advertise<mavlink_common::NAMED_VALUE_INT>("/from_mav_named_value_int", 10);
	from_mav_statustext_pub = n.advertise<mavlink_common::STATUSTEXT>("/from_mav_statustext", 10);
	from_mav_debug_pub = n.advertise<mavlink_common::DEBUG>("/from_mav_debug", 10);
	from_mav_sensor_offsets_pub = n.advertise<mavlink_ardupilotmega::SENSOR_OFFSETS>("/from_mav_sensor_offsets", 10);
	from_mav_set_mag_offsets_pub = n.advertise<mavlink_ardupilotmega::SET_MAG_OFFSETS>("/from_mav_set_mag_offsets", 10);
	from_mav_meminfo_pub = n.advertise<mavlink_ardupilotmega::MEMINFO>("/from_mav_meminfo", 10);
	from_mav_ap_adc_pub = n.advertise<mavlink_ardupilotmega::AP_ADC>("/from_mav_ap_adc", 10);
	from_mav_digicam_configure_pub = n.advertise<mavlink_ardupilotmega::DIGICAM_CONFIGURE>("/from_mav_digicam_configure", 10);
	from_mav_digicam_control_pub = n.advertise<mavlink_ardupilotmega::DIGICAM_CONTROL>("/from_mav_digicam_control", 10);
	from_mav_mount_configure_pub = n.advertise<mavlink_ardupilotmega::MOUNT_CONFIGURE>("/from_mav_mount_configure", 10);
	from_mav_mount_control_pub = n.advertise<mavlink_ardupilotmega::MOUNT_CONTROL>("/from_mav_mount_control", 10);
	from_mav_mount_status_pub = n.advertise<mavlink_ardupilotmega::MOUNT_STATUS>("/from_mav_mount_status", 10);
	from_mav_fence_point_pub = n.advertise<mavlink_ardupilotmega::FENCE_POINT>("/from_mav_fence_point", 10);
	from_mav_fence_fetch_point_pub = n.advertise<mavlink_ardupilotmega::FENCE_FETCH_POINT>("/from_mav_fence_fetch_point", 10);
	from_mav_fence_status_pub = n.advertise<mavlink_ardupilotmega::FENCE_STATUS>("/from_mav_fence_status", 10);
	from_mav_ahrs_pub = n.advertise<mavlink_ardupilotmega::AHRS>("/from_mav_ahrs", 10);
	from_mav_simstate_pub = n.advertise<mavlink_ardupilotmega::SIMSTATE>("/from_mav_simstate", 10);
	from_mav_hwstatus_pub = n.advertise<mavlink_ardupilotmega::HWSTATUS>("/from_mav_hwstatus", 10);
	from_mav_radio_pub = n.advertise<mavlink_ardupilotmega::RADIO>("/from_mav_radio", 10);
	from_mav_limits_status_pub = n.advertise<mavlink_ardupilotmega::LIMITS_STATUS>("/from_mav_limits_status", 10);
	from_mav_wind_pub = n.advertise<mavlink_ardupilotmega::WIND>("/from_mav_wind", 10);
	from_mav_data16_pub = n.advertise<mavlink_ardupilotmega::DATA16>("/from_mav_data16", 10);
	from_mav_data32_pub = n.advertise<mavlink_ardupilotmega::DATA32>("/from_mav_data32", 10);
	from_mav_data64_pub = n.advertise<mavlink_ardupilotmega::DATA64>("/from_mav_data64", 10);
	from_mav_data96_pub = n.advertise<mavlink_ardupilotmega::DATA96>("/from_mav_data96", 10);

    /**
     * Messages Subscribers Declaration
     */
	ros::Subscriber to_mav_heartbeat_sub = n.subscribe("/to_mav_heartbeat", 10, to_mav_heartbeat_callback);
	ros::Subscriber to_mav_sys_status_sub = n.subscribe("/to_mav_sys_status", 10, to_mav_sys_status_callback);
	ros::Subscriber to_mav_system_time_sub = n.subscribe("/to_mav_system_time", 10, to_mav_system_time_callback);
	ros::Subscriber to_mav_ping_sub = n.subscribe("/to_mav_ping", 10, to_mav_ping_callback);
	ros::Subscriber to_mav_change_operator_control_sub = n.subscribe("/to_mav_change_operator_control", 10, to_mav_change_operator_control_callback);
	ros::Subscriber to_mav_change_operator_control_ack_sub = n.subscribe("/to_mav_change_operator_control_ack", 10, to_mav_change_operator_control_ack_callback);
	ros::Subscriber to_mav_auth_key_sub = n.subscribe("/to_mav_auth_key", 10, to_mav_auth_key_callback);
	ros::Subscriber to_mav_set_mode_sub = n.subscribe("/to_mav_set_mode", 10, to_mav_set_mode_callback);
	ros::Subscriber to_mav_param_request_read_sub = n.subscribe("/to_mav_param_request_read", 10, to_mav_param_request_read_callback);
	ros::Subscriber to_mav_param_request_list_sub = n.subscribe("/to_mav_param_request_list", 10, to_mav_param_request_list_callback);
	ros::Subscriber to_mav_param_value_sub = n.subscribe("/to_mav_param_value", 10, to_mav_param_value_callback);
	ros::Subscriber to_mav_param_set_sub = n.subscribe("/to_mav_param_set", 10, to_mav_param_set_callback);
	ros::Subscriber to_mav_gps_raw_int_sub = n.subscribe("/to_mav_gps_raw_int", 10, to_mav_gps_raw_int_callback);
	ros::Subscriber to_mav_gps_status_sub = n.subscribe("/to_mav_gps_status", 10, to_mav_gps_status_callback);
	ros::Subscriber to_mav_scaled_imu_sub = n.subscribe("/to_mav_scaled_imu", 10, to_mav_scaled_imu_callback);
	ros::Subscriber to_mav_raw_imu_sub = n.subscribe("/to_mav_raw_imu", 10, to_mav_raw_imu_callback);
	ros::Subscriber to_mav_raw_pressure_sub = n.subscribe("/to_mav_raw_pressure", 10, to_mav_raw_pressure_callback);
	ros::Subscriber to_mav_scaled_pressure_sub = n.subscribe("/to_mav_scaled_pressure", 10, to_mav_scaled_pressure_callback);
	ros::Subscriber to_mav_attitude_sub = n.subscribe("/to_mav_attitude", 10, to_mav_attitude_callback);
	ros::Subscriber to_mav_attitude_quaternion_sub = n.subscribe("/to_mav_attitude_quaternion", 10, to_mav_attitude_quaternion_callback);
	ros::Subscriber to_mav_local_position_ned_sub = n.subscribe("/to_mav_local_position_ned", 10, to_mav_local_position_ned_callback);
	ros::Subscriber to_mav_global_position_int_sub = n.subscribe("/to_mav_global_position_int", 10, to_mav_global_position_int_callback);
	ros::Subscriber to_mav_rc_channels_scaled_sub = n.subscribe("/to_mav_rc_channels_scaled", 10, to_mav_rc_channels_scaled_callback);
	ros::Subscriber to_mav_rc_channels_raw_sub = n.subscribe("/to_mav_rc_channels_raw", 10, to_mav_rc_channels_raw_callback);
	ros::Subscriber to_mav_servo_output_raw_sub = n.subscribe("/to_mav_servo_output_raw", 10, to_mav_servo_output_raw_callback);
	ros::Subscriber to_mav_mission_request_partial_list_sub = n.subscribe("/to_mav_mission_request_partial_list", 10, to_mav_mission_request_partial_list_callback);
	ros::Subscriber to_mav_mission_write_partial_list_sub = n.subscribe("/to_mav_mission_write_partial_list", 10, to_mav_mission_write_partial_list_callback);
	ros::Subscriber to_mav_mission_item_sub = n.subscribe("/to_mav_mission_item", 10, to_mav_mission_item_callback);
	ros::Subscriber to_mav_mission_request_sub = n.subscribe("/to_mav_mission_request", 10, to_mav_mission_request_callback);
	ros::Subscriber to_mav_mission_set_current_sub = n.subscribe("/to_mav_mission_set_current", 10, to_mav_mission_set_current_callback);
	ros::Subscriber to_mav_mission_current_sub = n.subscribe("/to_mav_mission_current", 10, to_mav_mission_current_callback);
	ros::Subscriber to_mav_mission_request_list_sub = n.subscribe("/to_mav_mission_request_list", 10, to_mav_mission_request_list_callback);
	ros::Subscriber to_mav_mission_count_sub = n.subscribe("/to_mav_mission_count", 10, to_mav_mission_count_callback);
	ros::Subscriber to_mav_mission_clear_all_sub = n.subscribe("/to_mav_mission_clear_all", 10, to_mav_mission_clear_all_callback);
	ros::Subscriber to_mav_mission_item_reached_sub = n.subscribe("/to_mav_mission_item_reached", 10, to_mav_mission_item_reached_callback);
	ros::Subscriber to_mav_mission_ack_sub = n.subscribe("/to_mav_mission_ack", 10, to_mav_mission_ack_callback);
	ros::Subscriber to_mav_set_gps_global_origin_sub = n.subscribe("/to_mav_set_gps_global_origin", 10, to_mav_set_gps_global_origin_callback);
	ros::Subscriber to_mav_gps_global_origin_sub = n.subscribe("/to_mav_gps_global_origin", 10, to_mav_gps_global_origin_callback);
	ros::Subscriber to_mav_set_local_position_setpoint_sub = n.subscribe("/to_mav_set_local_position_setpoint", 10, to_mav_set_local_position_setpoint_callback);
	ros::Subscriber to_mav_local_position_setpoint_sub = n.subscribe("/to_mav_local_position_setpoint", 10, to_mav_local_position_setpoint_callback);
	ros::Subscriber to_mav_global_position_setpoint_int_sub = n.subscribe("/to_mav_global_position_setpoint_int", 10, to_mav_global_position_setpoint_int_callback);
	ros::Subscriber to_mav_set_global_position_setpoint_int_sub = n.subscribe("/to_mav_set_global_position_setpoint_int", 10, to_mav_set_global_position_setpoint_int_callback);
	ros::Subscriber to_mav_safety_set_allowed_area_sub = n.subscribe("/to_mav_safety_set_allowed_area", 10, to_mav_safety_set_allowed_area_callback);
	ros::Subscriber to_mav_safety_allowed_area_sub = n.subscribe("/to_mav_safety_allowed_area", 10, to_mav_safety_allowed_area_callback);
	ros::Subscriber to_mav_set_roll_pitch_yaw_thrust_sub = n.subscribe("/to_mav_set_roll_pitch_yaw_thrust", 10, to_mav_set_roll_pitch_yaw_thrust_callback);
	ros::Subscriber to_mav_set_roll_pitch_yaw_speed_thrust_sub = n.subscribe("/to_mav_set_roll_pitch_yaw_speed_thrust", 10, to_mav_set_roll_pitch_yaw_speed_thrust_callback);
	ros::Subscriber to_mav_roll_pitch_yaw_thrust_setpoint_sub = n.subscribe("/to_mav_roll_pitch_yaw_thrust_setpoint", 10, to_mav_roll_pitch_yaw_thrust_setpoint_callback);
	ros::Subscriber to_mav_roll_pitch_yaw_speed_thrust_setpoint_sub = n.subscribe("/to_mav_roll_pitch_yaw_speed_thrust_setpoint", 10, to_mav_roll_pitch_yaw_speed_thrust_setpoint_callback);
	ros::Subscriber to_mav_set_quad_motors_setpoint_sub = n.subscribe("/to_mav_set_quad_motors_setpoint", 10, to_mav_set_quad_motors_setpoint_callback);
	ros::Subscriber to_mav_set_quad_swarm_roll_pitch_yaw_thrust_sub = n.subscribe("/to_mav_set_quad_swarm_roll_pitch_yaw_thrust", 10, to_mav_set_quad_swarm_roll_pitch_yaw_thrust_callback);
	ros::Subscriber to_mav_nav_controller_output_sub = n.subscribe("/to_mav_nav_controller_output", 10, to_mav_nav_controller_output_callback);
	ros::Subscriber to_mav_set_quad_swarm_led_roll_pitch_yaw_thrust_sub = n.subscribe("/to_mav_set_quad_swarm_led_roll_pitch_yaw_thrust", 10, to_mav_set_quad_swarm_led_roll_pitch_yaw_thrust_callback);
	ros::Subscriber to_mav_state_correction_sub = n.subscribe("/to_mav_state_correction", 10, to_mav_state_correction_callback);
	ros::Subscriber to_mav_request_data_stream_sub = n.subscribe("/to_mav_request_data_stream", 10, to_mav_request_data_stream_callback);
	ros::Subscriber to_mav_data_stream_sub = n.subscribe("/to_mav_data_stream", 10, to_mav_data_stream_callback);
	ros::Subscriber to_mav_manual_control_sub = n.subscribe("/to_mav_manual_control", 10, to_mav_manual_control_callback);
	ros::Subscriber to_mav_rc_channels_override_sub = n.subscribe("/to_mav_rc_channels_override", 10, to_mav_rc_channels_override_callback);
	ros::Subscriber to_mav_vfr_hud_sub = n.subscribe("/to_mav_vfr_hud", 10, to_mav_vfr_hud_callback);
	ros::Subscriber to_mav_command_long_sub = n.subscribe("/to_mav_command_long", 10, to_mav_command_long_callback);
	ros::Subscriber to_mav_command_ack_sub = n.subscribe("/to_mav_command_ack", 10, to_mav_command_ack_callback);
	ros::Subscriber to_mav_roll_pitch_yaw_rates_thrust_setpoint_sub = n.subscribe("/to_mav_roll_pitch_yaw_rates_thrust_setpoint", 10, to_mav_roll_pitch_yaw_rates_thrust_setpoint_callback);
	ros::Subscriber to_mav_manual_setpoint_sub = n.subscribe("/to_mav_manual_setpoint", 10, to_mav_manual_setpoint_callback);
	ros::Subscriber to_mav_local_position_ned_system_global_offset_sub = n.subscribe("/to_mav_local_position_ned_system_global_offset", 10, to_mav_local_position_ned_system_global_offset_callback);
	ros::Subscriber to_mav_hil_state_sub = n.subscribe("/to_mav_hil_state", 10, to_mav_hil_state_callback);
	ros::Subscriber to_mav_hil_controls_sub = n.subscribe("/to_mav_hil_controls", 10, to_mav_hil_controls_callback);
	ros::Subscriber to_mav_hil_rc_inputs_raw_sub = n.subscribe("/to_mav_hil_rc_inputs_raw", 10, to_mav_hil_rc_inputs_raw_callback);
	ros::Subscriber to_mav_optical_flow_sub = n.subscribe("/to_mav_optical_flow", 10, to_mav_optical_flow_callback);
	ros::Subscriber to_mav_global_vision_position_estimate_sub = n.subscribe("/to_mav_global_vision_position_estimate", 10, to_mav_global_vision_position_estimate_callback);
	ros::Subscriber to_mav_vision_position_estimate_sub = n.subscribe("/to_mav_vision_position_estimate", 10, to_mav_vision_position_estimate_callback);
	ros::Subscriber to_mav_vision_speed_estimate_sub = n.subscribe("/to_mav_vision_speed_estimate", 10, to_mav_vision_speed_estimate_callback);
	ros::Subscriber to_mav_vicon_position_estimate_sub = n.subscribe("/to_mav_vicon_position_estimate", 10, to_mav_vicon_position_estimate_callback);
	ros::Subscriber to_mav_highres_imu_sub = n.subscribe("/to_mav_highres_imu", 10, to_mav_highres_imu_callback);
	ros::Subscriber to_mav_file_transfer_start_sub = n.subscribe("/to_mav_file_transfer_start", 10, to_mav_file_transfer_start_callback);
	ros::Subscriber to_mav_file_transfer_dir_list_sub = n.subscribe("/to_mav_file_transfer_dir_list", 10, to_mav_file_transfer_dir_list_callback);
	ros::Subscriber to_mav_file_transfer_res_sub = n.subscribe("/to_mav_file_transfer_res", 10, to_mav_file_transfer_res_callback);
	ros::Subscriber to_mav_battery_status_sub = n.subscribe("/to_mav_battery_status", 10, to_mav_battery_status_callback);
	ros::Subscriber to_mav_setpoint_8dof_sub = n.subscribe("/to_mav_setpoint_8dof", 10, to_mav_setpoint_8dof_callback);
	ros::Subscriber to_mav_setpoint_6dof_sub = n.subscribe("/to_mav_setpoint_6dof", 10, to_mav_setpoint_6dof_callback);
	ros::Subscriber to_mav_memory_vect_sub = n.subscribe("/to_mav_memory_vect", 10, to_mav_memory_vect_callback);
	ros::Subscriber to_mav_debug_vect_sub = n.subscribe("/to_mav_debug_vect", 10, to_mav_debug_vect_callback);
	ros::Subscriber to_mav_named_value_float_sub = n.subscribe("/to_mav_named_value_float", 10, to_mav_named_value_float_callback);
	ros::Subscriber to_mav_named_value_int_sub = n.subscribe("/to_mav_named_value_int", 10, to_mav_named_value_int_callback);
	ros::Subscriber to_mav_statustext_sub = n.subscribe("/to_mav_statustext", 10, to_mav_statustext_callback);
	ros::Subscriber to_mav_debug_sub = n.subscribe("/to_mav_debug", 10, to_mav_debug_callback);
	ros::Subscriber to_mav_sensor_offsets_sub = n.subscribe("/to_mav_sensor_offsets", 10, to_mav_sensor_offsets_callback);
	ros::Subscriber to_mav_set_mag_offsets_sub = n.subscribe("/to_mav_set_mag_offsets", 10, to_mav_set_mag_offsets_callback);
	ros::Subscriber to_mav_meminfo_sub = n.subscribe("/to_mav_meminfo", 10, to_mav_meminfo_callback);
	ros::Subscriber to_mav_ap_adc_sub = n.subscribe("/to_mav_ap_adc", 10, to_mav_ap_adc_callback);
	ros::Subscriber to_mav_digicam_configure_sub = n.subscribe("/to_mav_digicam_configure", 10, to_mav_digicam_configure_callback);
	ros::Subscriber to_mav_digicam_control_sub = n.subscribe("/to_mav_digicam_control", 10, to_mav_digicam_control_callback);
	ros::Subscriber to_mav_mount_configure_sub = n.subscribe("/to_mav_mount_configure", 10, to_mav_mount_configure_callback);
	ros::Subscriber to_mav_mount_control_sub = n.subscribe("/to_mav_mount_control", 10, to_mav_mount_control_callback);
	ros::Subscriber to_mav_mount_status_sub = n.subscribe("/to_mav_mount_status", 10, to_mav_mount_status_callback);
	ros::Subscriber to_mav_fence_point_sub = n.subscribe("/to_mav_fence_point", 10, to_mav_fence_point_callback);
	ros::Subscriber to_mav_fence_fetch_point_sub = n.subscribe("/to_mav_fence_fetch_point", 10, to_mav_fence_fetch_point_callback);
	ros::Subscriber to_mav_fence_status_sub = n.subscribe("/to_mav_fence_status", 10, to_mav_fence_status_callback);
	ros::Subscriber to_mav_ahrs_sub = n.subscribe("/to_mav_ahrs", 10, to_mav_ahrs_callback);
	ros::Subscriber to_mav_simstate_sub = n.subscribe("/to_mav_simstate", 10, to_mav_simstate_callback);
	ros::Subscriber to_mav_hwstatus_sub = n.subscribe("/to_mav_hwstatus", 10, to_mav_hwstatus_callback);
	ros::Subscriber to_mav_radio_sub = n.subscribe("/to_mav_radio", 10, to_mav_radio_callback);
	ros::Subscriber to_mav_limits_status_sub = n.subscribe("/to_mav_limits_status", 10, to_mav_limits_status_callback);
	ros::Subscriber to_mav_wind_sub = n.subscribe("/to_mav_wind", 10, to_mav_wind_callback);
	ros::Subscriber to_mav_data16_sub = n.subscribe("/to_mav_data16", 10, to_mav_data16_callback);
	ros::Subscriber to_mav_data32_sub = n.subscribe("/to_mav_data32", 10, to_mav_data32_callback);
	ros::Subscriber to_mav_data64_sub = n.subscribe("/to_mav_data64", 10, to_mav_data64_callback);
	ros::Subscriber to_mav_data96_sub = n.subscribe("/to_mav_data96", 10, to_mav_data96_callback);


    ros::spin();
    return 0;
}
