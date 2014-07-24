# Mavlink to ROS API Generator Package

This is a python package that enable us to create a entire Mavlink API under ROS middleware

## Main Features 

* Generate a ROS Package ( for the main definition and others for each include in definition file).
* Generate all msg files to enable to receive and send mavlink messages.
* Generate a node with callbacks, subscribers and callbacks and a socket client to enable to connect to the ardupilot SITL for testing purposes. 
* Generate the mavlink code based on the definition file ready to be used by the generated package

## Installation 

* Install python-pip ie: sudo apt-get install python-pip
* Run command: sudo pip install --upgrade mav2rosgenerator
* Run command to see execution options: run_mav2ros.py -h

## Usage
* Run Command:
```bash
run_mav2ros.py -m $HOME/libs/mavlink-1.0.11/message_definitions/v1.0/ardupilotmega.xml -o $HOME/ros_workspace/src
```
## Future features
1. Create a configuration or semantic (up, down, both) in the direction of the mavlink messages to optimize callbacks and subscribers ( for UP, BOTH directions messages) and publishers (for DOWN direction)
Maybe mavlink definition file can contain a message tag attribute with direction:
```xml
    <message id="11" name="SET_MODE" direction="UP">
```
Example: 

* HEARTBEAT message direction = BOTH
* SET_MODE message direction = UP
* GPS_RAW_INT message direction = down

