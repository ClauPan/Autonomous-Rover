#!/bin/bash
source install/setup.bash
ros2 launch vox launch_sim_no_joystick.launch.py world:=./src/worlds/sim.world
