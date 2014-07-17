# Mavlink to ROS API Generator Package

This is a python package that enable us to create a entire Mavlink API under ROS middleware

## Main Features 

* Generate a ROS Package ( for the main definition and others for each include in definition file).
* Generate all msg files to enable to receive and send mavlink messages.
* Generate a node with callbacks, subscribers and callbacks and a socket client to enable to connect to the ardupilot SITL for testing purposes. 
* Generate the mavlink code based on the definition file ready to be used by the generated package

## Example

```python

from mav2ros.tools import *

        definitions_file = '/home/posilva/ros_workspaces/roscon14_ws/libs/mavlink-1.0.11/message_definitions/v1.0/ardupilotmega.xml'
        output_dir = '/home/posilva/ros_workspaces/roscon14_ws/src'

        generator = MAVGenerator( definitions_file, output_dir )
        generator.generate(True,True);
```
## Future features

1. Create a ROS abstraction message/topic to provide a transparent way to send/receive data from the generated API

Example:

- Create a message MavlinkRawData (uint8 channel, uint8[] data)
- Create two topics (/to_mav_raw_data, /from_mav_raw_data)

With this design we can left outside of the generated API the type of connection used. We can create one node connected to a radio that publishes and subscribe
this topics to received and send data to ROS.


