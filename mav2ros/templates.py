'''
Created on Jul 17, 2014

@author: posilva
'''
#===============================================================================
# Constants
#===============================================================================
PKG_NAME_PLACEHOLDER="$PACKAGE_NAME$"
PKG_DEFINITIONS_NAME_PLACEHOLDER="$PACKAGE_DEFINITIONS_NAME$"
CMAKE_PKG_DEPS_PLACEHOLDER="$CMAKE_PACKAGE_DEPS$"
CMAKE_PKG_ADD_MESSAGE_PLACEHOLDER="$CMAKE_PACKAGE_ADD_MESSAGES$"
CMAKE_PKG_GENERATE_MESSAGES_PLACEHOLDER="$CMAKE_PACKAGE_GENERATE_MESSAGES$"
CMAKE_PKG_ADD_NODE_PLACEHOLDER="$CMAKE_PACKAGE_ADD_NODE$"
PKG_XML_RUN_DEPS_PLACEHOLDER="$PKG_XML_RUN_DEPS$"
PKG_XML_BUILD_DEPS_PLACEHOLDER="$PKG_XML_BUILD_DEPS$"
NODE_PUBLISHERS_DECL_PLACE_HOLDER ="$NODE_PUBLISHERS_DECL$"
NODE_SUBSCRIBERS_DECL_PLACE_HOLDER ="$NODE_SUBSCRIBERS_DECL$"
NODE_MESSAGES_CALLBACKS_PLACE_HOLDER ="$NODE_MESSAGES_CALLBACKS$"
NODE_MESSAGES_CALLBACK_FIELDS_PLACE_HOLDER ="$NODE_MESSAGES_CALLBACK_FIELDS$"
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

cmake_node_template="""
    add_executable(test_$PACKAGE_NAME$ src/$PACKAGE_NAME$_node.cpp)
    target_link_libraries(test_$PACKAGE_NAME$ ${catkin_LIBRARIES})
"""
#===============================================================================
# package.xml template
#===============================================================================
package_xml_template = """<?xml version="1.0"?>
<package>
    <name>$PACKAGE_NAME$</name>
    <version>0.0.1</version>
    <description>Mavlink generated messages for package $PACKAGE_NAME$</description>
    <maintainer email="posilva@academiafa.edu.pt">Pedro Marque da Silva</maintainer>
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
#===============================================================================
# Source code template for ROS node
#===============================================================================
ros_node_template="""
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
/**
 * 
 **/
#include <ros/ros.h>
#include <$PACKAGE_NAME$/$PACKAGE_DEFINITIONS_NAME$/mavlink.h>
#include <$PACKAGE_NAME$/mavlink2ros.h>

/**
 * 
 **/
int sockfd;
int portno=5763;
int n_bytes=0;
int buff_readed=0;
struct sockaddr_in serv_addr;
struct hostent *server;

char buffer[256];

/**
 * 
 **/
void error(const char *msg) {
    perror(msg);
    exit(0);
}

/**
 * 
 **/
char read_char()
{
    if (sockfd<0)
        error("Socket not ready");

    if (buff_readed>=n_bytes){
        buff_readed=0;
        memset(buffer,0,256);
        n_bytes = read(sockfd,buffer,255);
        if(n_bytes < 0)
            error("Failed to read from socket");
    }
    return buffer[buff_readed++];
}

/**
 * 
 **/
int write_to_mav (uint8_t * b, int sz){
    if (sockfd<0)
        error("Socket not ready");
    int rc = write(sockfd,b,sz);
    ROS_INFO("Writen to MAV %d bytes",rc);
    return rc;
}        

/**
 * 
 **/
void setup_comms(){
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    
    if (sockfd < 0) 
        error("ERROR opening socket");
    
    server = gethostbyname("localhost");
    if (server == NULL) {
        error("ERROR no such host");
    }

    memset((char *) &serv_addr,0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    memcpy((char *)server->h_addr,(char *)&serv_addr.sin_addr.s_addr,server->h_length);
    serv_addr.sin_port = htons(portno);
    
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
}

/**
 *
 **/
void close_comms(){
    if(sockfd>=0){
        close(sockfd);
    }
}

$NODE_MESSAGES_CALLBACKS$

/**
 *
 *
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "$PACKAGE_NAME$_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);


    setup_comms();

    /**
     * Messages Publishers Declaration
     */
$NODE_PUBLISHERS_DECL$
    /**
     * Messages Subscribers Declaration
     */
$NODE_SUBSCRIBERS_DECL$

    mavlink_message_t msg;
    mavlink_status_t status;
            
    while (ros::ok())
    {
        
        uint8_t c = read_char();
        // Try to get a new message
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            // Handle message
            switch(msg.msgid)
            {
                $NODE_MAV_SWITCH_CASES$
                default:
                    //Do nothing
                    break;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    close_comms();
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
                        
                        m.sysid=msg.sysid;
                        m.compid=msg.compid;
                        
                        mavlink_$NODE_MESSAGE_NAME_LOWER$_t $NODE_MESSAGE_NAME_LOWER$_in;
                        memset(&$NODE_MESSAGE_NAME_LOWER$_in, 0, sizeof($NODE_MESSAGE_NAME_LOWER$_in));
                        mavlink_msg_$NODE_MESSAGE_NAME_LOWER$_decode(&msg, &$NODE_MESSAGE_NAME_LOWER$_in);
                        
$NODE_MAV_SWITCH_CASE_FIELDS$
                        
                        from_mav_$NODE_MESSAGE_NAME_LOWER$_pub.publish(m);     
                    }
                    break;"""
#===============================================================================
# ROS Publisher declaration template
#===============================================================================
publisher_decl_template ="""\tros::Publisher from_mav_$NODE_MESSAGE_NAME_LOWER$_pub = n.advertise<$PACKAGE_NAME$::$NODE_MESSAGE_NAME$>("/from_mav_$NODE_MESSAGE_NAME_LOWER$", 10);
"""

#===============================================================================
# ROS Subscriber declaration template
#===============================================================================
subscriber_decl_template ="""\tros::Subscriber to_mav_$NODE_MESSAGE_NAME_LOWER$_sub = n.subscribe("/to_mav_$NODE_MESSAGE_NAME_LOWER$", 10, to_mav_$NODE_MESSAGE_NAME_LOWER$_callback);
"""