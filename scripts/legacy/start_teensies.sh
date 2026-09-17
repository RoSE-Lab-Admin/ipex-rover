#!/bin/bash

# Source ROS 2 Jazzy and workspace
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash

# Force Domain 42 & FastDDS for ALL processes
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

TEENSY1_PORT="/dev/serial/by-id/usb-Teensyduino_USB_Serial_19972490-if00"
TEENSY2_PORT="/dev/serial/by-id/usb-Teensyduino_USB_Serial_20406990-if00"

trap 'echo "Stopping all processes..."; kill 0' EXIT INT TERM

echo "Starting Micro-ROS Agent 1..."
( while true; do
    ros2 run micro_ros_agent micro_ros_agent serial --dev "$TEENSY1_PORT" -b 115200
    sleep 1
  done ) &

echo "Starting Micro-ROS Agent 2..."
( while true; do
    ros2 run micro_ros_agent micro_ros_agent serial --dev "$TEENSY2_PORT" -b 115200
    sleep 1
  done ) &

echo "Starting Foxglove Bridge on Domain 42..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765 &

wait

