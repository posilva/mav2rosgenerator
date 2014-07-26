'''
Created on Jul 17, 2014

@author: posilva
'''
#===============================================================================
# Constant variables
#===============================================================================
MAV_RAW_DATA_MSG = "MAV_RAW_DATA"
PACKAGE_VERSION="0.1.9"
#===============================================================================
# Constants for place holders
#===============================================================================
PKG_NAME_PLACEHOLDER = "$PACKAGE_NAME$"
PKG_DEFINITIONS_NAME_PLACEHOLDER = "$PACKAGE_DEFINITIONS_NAME$"
CMAKE_PKG_DEPS_PLACEHOLDER = "$CMAKE_PACKAGE_DEPS$"
CMAKE_PKG_ADD_MESSAGE_PLACEHOLDER = "$CMAKE_PACKAGE_ADD_MESSAGES$"
CMAKE_PKG_ADD_RAW_MESSAGE_PLACEHOLDER = "$CMAKE_PACKAGE_ADD_RAW_MESSAGES$"
CMAKE_PKG_GENERATE_MESSAGES_PLACEHOLDER = "$CMAKE_PACKAGE_GENERATE_MESSAGES$"
CMAKE_PKG_ADD_NODE_PLACEHOLDER = "$CMAKE_PACKAGE_ADD_NODE$"
PKG_XML_RUN_DEPS_PLACEHOLDER = "$PKG_XML_RUN_DEPS$"
PKG_XML_BUILD_DEPS_PLACEHOLDER = "$PKG_XML_BUILD_DEPS$"
NODE_PUBLISHERS_DECL_PLACE_HOLDER = "$NODE_PUBLISHERS_DECL$"
NODE_SUBSCRIBERS_DECL_PLACE_HOLDER = "$NODE_SUBSCRIBERS_DECL$"
NODE_PUBLISHERS_INIT_PLACE_HOLDER = "$NODE_PUBLISHERS_INIT$"

NODE_MESSAGES_CALLBACKS_PLACE_HOLDER = "$NODE_MESSAGES_CALLBACKS$"
NODE_MESSAGES_CALLBACK_FIELDS_PLACE_HOLDER = "$NODE_MESSAGES_CALLBACK_FIELDS$"
NODE_MAV_SWITCH_CASES_PLACE_HOLDER = "$NODE_MAV_SWITCH_CASES$"
NODE_MAV_SWITCH_CASE_FIELDS_PLACE_HOLDER = "$NODE_MAV_SWITCH_CASE_FIELDS$"
NODE_MESSAGE_NAME_PLACE_HOLDER = "$NODE_MESSAGE_NAME$"
NODE_MESSAGE_NAME_LOWER_PLACE_HOLDER = "$NODE_MESSAGE_NAME_LOWER$"

#===============================================================================
# CMakeLists.txt template
#===============================================================================
cmakelists_template = """cmake_minimum_required(VERSION 2.8.3)
project($PACKAGE_NAME$)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  $CMAKE_PACKAGE_DEPS$
)

add_message_files(
   FILES
   $CMAKE_PACKAGE_ADD_RAW_MESSAGES$
   $CMAKE_PACKAGE_ADD_MESSAGES$
)
generate_messages(
    DEPENDENCIES
    std_msgs
    $CMAKE_PACKAGE_GENERATE_MESSAGES$
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
$CMAKE_PACKAGE_ADD_NODE$

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
"""

cmake_node_template = """
    add_executable($PACKAGE_NAME$_node src/$PACKAGE_NAME$_node.cpp)
    target_link_libraries($PACKAGE_NAME$_node ${catkin_LIBRARIES})
    add_dependencies($PACKAGE_NAME$_node  $PACKAGE_NAME$_generate_messages_cpp)
    
install(TARGETS $PACKAGE_NAME$_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
"""
#===============================================================================
# package.xml template
#===============================================================================
package_xml_template = """<?xml version="1.0"?>
<package>
    <name>$PACKAGE_NAME$</name>
    <version>"""+PACKAGE_VERSION+"""</version>
    <description>Mavlink generated messages for package $PACKAGE_NAME$</description>
    <maintainer email="posilva@gmail.com">Pedro Marque da Silva</maintainer>
    <license>MIT</license>
    
    <author email="posilva@academiafa.edu.pt">Pedro Marques da Silva</author>
    <build_depend>message_generation</build_depend>
    <run_depend>message_runtime</run_depend>
    <run_depend>roscpp</run_depend>
    <run_depend>rospy</run_depend>
    <buildtool_depend>catkin</buildtool_depend>
    <build_depend>roscpp</build_depend>
    <build_depend>rospy</build_depend>
    
    $PKG_XML_RUN_DEPS$
    $PKG_XML_BUILD_DEPS$
</package>
"""

mav_raw_message_template="""
uint8 channel
    uint8 CH_COMM0=0
    uint8 CH_COMM1=1
    uint8 CH_COMM2=2
    uint8 CH_COMM3=3
uint8[] data
"""
#===============================================================================
# Source code template for ROS node
#===============================================================================
ros_node_template = """
/**
 * 
 **/
#include <ros/ros.h>
#include <$PACKAGE_NAME$/$PACKAGE_DEFINITIONS_NAME$/mavlink.h>
#include <$PACKAGE_NAME$/mavlink2ros.h>

mavlink_message_t mav_msg;                  //! Global mavlink message 
mavlink_status_t status;                //! Global mavlink status    

ros::Publisher  to_mav_""" +MAV_RAW_DATA_MSG.lower() +"""_publisher;        //! ROS publisher to write to mavlink interface
ros::Subscriber from_mav_""" +MAV_RAW_DATA_MSG.lower() +"""_subscriber;     //! ROS subscriber to read from mavlink interface

$NODE_PUBLISHERS_DECL$

/**
 * 
 **/
int write_to_mav (uint8_t * b, int sz){

    $PACKAGE_NAME$::""" +MAV_RAW_DATA_MSG +""" m;
    
    m.channel=$PACKAGE_NAME$::""" +MAV_RAW_DATA_MSG +"""::CH_COMM0;
    m.data.assign(b, b+sz);
    int rc = m.data.size();
    to_mav_""" +MAV_RAW_DATA_MSG.lower() +"""_publisher.publish(m);
    ROS_INFO("Writen to MAV %d bytes",rc);
    return rc;
}        

$NODE_MESSAGES_CALLBACKS$

/**
 *
 */            
void from_mav_""" +MAV_RAW_DATA_MSG.lower() +"""_callback(const $PACKAGE_NAME$::""" +MAV_RAW_DATA_MSG +"""::ConstPtr& msg)
{
    ROS_INFO("[$PACKAGE_NAME$] Received a  'from_mav_""" +MAV_RAW_DATA_MSG.lower() +"""_callback request");
    
    for (int i = 0;i<msg->data.size();++i){
        // Try to get a new message
        if(mavlink_parse_char(msg->channel, msg->data[i], &mav_msg, &status)) {
            // Handle message
            switch(mav_msg.msgid)
            {
                $NODE_MAV_SWITCH_CASES$
                default:
                    //Do nothing
                    break;
            }
        }
    }
}

 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "$PACKAGE_NAME$_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    to_mav_""" +MAV_RAW_DATA_MSG.lower() +"""_publisher     = n.advertise<$PACKAGE_NAME$::""" +MAV_RAW_DATA_MSG +""">("/to_mav_""" +MAV_RAW_DATA_MSG.lower() +"""", 10);
    from_mav_""" +MAV_RAW_DATA_MSG.lower() +"""_subscriber  = n.subscribe("/from_mav_""" +MAV_RAW_DATA_MSG +"""", 10, from_mav_""" +MAV_RAW_DATA_MSG.lower() +"""_callback);

    /**
     * Messages Publishers Initialization
     */
$NODE_PUBLISHERS_INIT$
    /**
     * Messages Subscribers Declaration
     */
$NODE_SUBSCRIBERS_DECL$

    ros::spin();
    return 0;
}
"""

#===============================================================================
# ROS messages callback template
#===============================================================================
messages_callback_template = """
/**
 *
 */            
void to_mav_$NODE_MESSAGE_NAME_LOWER$_callback(const $PACKAGE_NAME$::$NODE_MESSAGE_NAME$::ConstPtr& msg)
{
    ROS_INFO("[$PACKAGE_NAME$] Received a  'to_mav_$NODE_MESSAGE_NAME$ request");
    
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t m;
    
    m.sysid=msg->sysid;
    m.compid=msg->compid;
    
    mavlink_$NODE_MESSAGE_NAME_LOWER$_t $NODE_MESSAGE_NAME_LOWER$_out;
    
    /** ASSIGN FIELDS **/
    
$NODE_MESSAGES_CALLBACK_FIELDS$
    
    mavlink_msg_$NODE_MESSAGE_NAME_LOWER$_encode(msg->sysid, msg->compid, &m, &$NODE_MESSAGE_NAME_LOWER$_out);
    uint16_t len=mavlink_msg_to_send_buffer(data, &m);
    write_to_mav(data,len);
    
}"""
#===============================================================================
# Switch case template to mavlink messages reception
#===============================================================================
switch_case_template = """
                    case MAVLINK_MSG_ID_$NODE_MESSAGE_NAME$:
                    {
                        $PACKAGE_NAME$::$NODE_MESSAGE_NAME$ m;
                        
                        m.sysid=mav_msg.sysid;
                        m.compid=mav_msg.compid;
                        
                        mavlink_$NODE_MESSAGE_NAME_LOWER$_t $NODE_MESSAGE_NAME_LOWER$_in;
                        memset(&$NODE_MESSAGE_NAME_LOWER$_in, 0, sizeof($NODE_MESSAGE_NAME_LOWER$_in));
                        mavlink_msg_$NODE_MESSAGE_NAME_LOWER$_decode(&mav_msg, &$NODE_MESSAGE_NAME_LOWER$_in);
                        
                        $NODE_MAV_SWITCH_CASE_FIELDS$
                        
                        from_mav_$NODE_MESSAGE_NAME_LOWER$_pub.publish(m);     
                    }
                    break;"""
#===============================================================================
# ROS Publisher declaration template
#===============================================================================
publisher_decl_template = """\tros::Publisher from_mav_$NODE_MESSAGE_NAME_LOWER$_pub;
"""
publisher_init_template = """\tfrom_mav_$NODE_MESSAGE_NAME_LOWER$_pub = n.advertise<$PACKAGE_NAME$::$NODE_MESSAGE_NAME$>("/from_mav_$NODE_MESSAGE_NAME_LOWER$", 10);
"""

#===============================================================================
# ROS Subscriber declaration template
#===============================================================================
subscriber_decl_template = """\tros::Subscriber to_mav_$NODE_MESSAGE_NAME_LOWER$_sub = n.subscribe("/to_mav_$NODE_MESSAGE_NAME_LOWER$", 10, to_mav_$NODE_MESSAGE_NAME_LOWER$_callback);
"""

