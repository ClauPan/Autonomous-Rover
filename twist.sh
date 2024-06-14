#!/bin/bash

ros2 run twist_mux twist_mux --ros-args --params-file ./src/vox/config/twist_mux.yaml -r cmd_vel_out:=skid_steer/cmd_vel_unstamped
