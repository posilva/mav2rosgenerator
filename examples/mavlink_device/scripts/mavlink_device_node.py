#!/usr/bin/env python


import rospy
import serial
from mavlink_ardupilotmega.msg import MAV_RAW


ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)


def to_mav_raw_callback(message):
    rospy.loginfo("Write to mav "+ str(len(message.data)) + " bytes")
    ser.write(message.data)

def node():
    rospy.init_node('mavlink_device_node', anonymous=True)
    
    pub = rospy.Publisher('/from_mav_raw', MAV_RAW, queue_size=10)
    rospy.Subscriber("/to_mav_raw", MAV_RAW, to_mav_raw_callback,queue_size=10)
    
    r = rospy.Rate(100) # 100hz
    
    m = MAV_RAW()
    m.channel=0
    
    while not rospy.is_shutdown():
      m.data = ser.read(255) 
      pub.publish(m)
      r.sleep()
      
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException: pass
