#!/bin/bash

# Rover boot stack after drivetrain migration to ros2_control + direct USB serial.

# 1. Load ROS environments.
source /opt/ros/jazzy/setup.bash
source /home/ipex-rp5/microros_ws/install/setup.bash
source /home/ipex-rp5/ipex_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 2. Clear any stale micro-ROS agents / graph cache.
pkill -9 -f micro_ros_agent 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1

# 3. Shoulder Teensies still use micro-ROS for now.
PORT_SHOULDER1="/dev/serial/by-id/usb-Teensyduino_USB_Serial_20404840-if00"
PORT_SHOULDER2="/dev/serial/by-id/usb-Teensyduino_USB_Serial_19972490-if00"

manage_agent() {
    local port="$1"
    local node_name="$2"
    local agent_pid=""

    while true; do
        if [ -c "$port" ]; then
            if [ -z "$agent_pid" ] || ! kill -0 "$agent_pid" 2>/dev/null; then
                echo "[CONNECTED] Hardware detected on $node_name. Spawning agent..."
                ros2 run micro_ros_agent micro_ros_agent serial \
                    --dev "$port" -b 115200 \
                    --ros-args -r __node:="$node_name" &
                agent_pid=$!
            fi
        else
            if [ -n "$agent_pid" ] && kill -0 "$agent_pid" 2>/dev/null; then
                echo "[DISCONNECTED] $node_name unplugged. Killing PID $agent_pid..."
                kill -9 "$agent_pid" 2>/dev/null
                agent_pid=""
                ros2 daemon stop >/dev/null 2>&1
                ros2 daemon start >/dev/null 2>&1
            fi
        fi
        sleep 1
    done
}

cleanup() {
    echo "Shutting down rover stack..."
    pkill -9 -f micro_ros_agent 2>/dev/null
    jobs -pr | xargs -r kill 2>/dev/null
}
trap cleanup EXIT INT TERM

# 4. Keep shoulder micro-ROS connectivity.
manage_agent "$PORT_SHOULDER1" "agent_shoulder_1" &
SHOULDER1_MONITOR_PID=$!
manage_agent "$PORT_SHOULDER2" "agent_shoulder_2" &
SHOULDER2_MONITOR_PID=$!

# 5. New drivetrain stack: ros2_control -> IpexSystem -> direct USB serial.
ros2 launch ipex_bringup control.launch.py &
CONTROL_PID=$!

# 6. Preserve legacy Foxglove /cmd_vel workflow.
ros2 run ipex_motion cmd_vel_bridge &
CMD_VEL_BRIDGE_PID=$!

# 7. Foxglove stays exactly where users expect it.
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    address:=0.0.0.0 port:=8765 &
FOXGLOVE_PID=$!

# Keep the service alive while the rover processes run.
wait
