#!/bin/bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml -p use_sim_time:=true

