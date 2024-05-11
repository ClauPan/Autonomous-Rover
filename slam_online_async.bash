#!/bin/bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/vox/config/mapper_params_online_async.yaml use_sim_time:=false
