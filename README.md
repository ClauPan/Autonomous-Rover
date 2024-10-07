# Autonomous Rover
## Introduction
An autonomous indoor rover, capable of mapping and navigating a dynamic environment, as well as monitoring a chosen asset in said space.

## Features
Manual mapping: The rover can be moved manually using keyboard commands or a joystick to map a (preferably empty) room.

Autonomous naviagation and avoidance of new obects placed in the environment: A goal is given to the rover. It will calculate the best route to take to reach said goal. Depending on wether new objects were added to the environment, it will recalculate the route to avoid said objects. 

Visual recognition of a user chosen asset

Following of a user chosen asset

## Technologies Used
ROS2 Iron

ROS2 Control

Robot Localization

SLAM Toolbox

Nav2

V4L2 Camera
