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
        # mavlink definitions file 
        definitions_file = '/home/posilva/libs/mavlink-1.0.11/message_definitions/v1.0/ardupilotmega.xml'
        # where the rospackage will be generated 
        output_dir = '/home/posilva/ros_workspaces/roscon14_ws/src'

        generator = MAVGenerator( definitions_file, output_dir )
        with_node = True        # generate API code inside a node
        with_mavlink = True     # invoke mavlink generator (mavgen.py) to generate mavlink headers
        generator.generate(with_node, with_mavlink); 
```
## Future features

1. Create a ROS abstraction message/topic to provide a transparent way to send/receive data from the generated API

Example:

- Create a message MavlinkRawData (uint8 channel, uint8[] data)
- Create two topics (/to_mav_raw_data, /from_mav_raw_data)

With this design we can left outside of the generated API the type of connection used. We can create one node connected to a radio that publishes and subscribe
this topics to received and send data to ROS.

2. Create a configuration or semantic (up, down, both) in the direction of the mavlink messages to optimize callbacks and subscribers ( for UP, BOTH directions messages) and publishers (for DOWN direction)
Maybe mavlink definition file can contain a message tag attribute with direction:

<message id="11" name="SET_MODE" direction="UP">

Example: 

* HEARTBEAT message direction = BOTH
* SET_MODE message direction = UP
* GPS_RAW_INT message direction = down

