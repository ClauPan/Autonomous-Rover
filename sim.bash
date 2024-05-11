#!/bin/bash
source install/setup.bash
ros2 launch vox launch_sim.launch.py world:=./src/worlds/sim.world
