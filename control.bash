#!/bin/bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/skid_steer/cmd_vel_unstamped

