#!/bin/bash
ros2 launch slam_toolbox localization_launch.py slam_params_file:=./src/config/mapper_params_localization.yaml use_sim_time:=true
