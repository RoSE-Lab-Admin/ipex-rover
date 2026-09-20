#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /home/ipex-rp5/ipex_ws/install/setup.bash

export ROS_DOMAIN_ID=42

exec ros2 launch ipex_bringup rover.launch.py
