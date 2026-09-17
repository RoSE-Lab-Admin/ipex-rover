#!/bin/bash

#### Backup of start_rover.sh for testing purposes#######




# 1. Load ROS 2 Environment
source /opt/ros/jazzy/setup.bash
source /home/ipex-rp5/microros_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 2. Hard Reset: Kill existing agents and flush ghost graph cache
pkill -9 -f micro_ros_agent 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1

# 3. Define USB Serial Identifiers
PORT_DRIVETRAIN="/dev/serial/by-id/usb-Teensyduino_USB_Serial_20406990-if00"
PORT_SHOULDER1="/dev/serial/by-id/usb-Teensyduino_USB_Serial_20404840-if00"
PORT_SHOULDER2="/dev/serial/by-id/usb-Teensyduino_USB_Serial_19972490-if00"

# 4. Low-Level Device Lifecycle Monitor
manage_agent() {
    local port="$1"
    local node_name="$2"
    local agent_pid=""

    while true; do
        # Check if Linux sees the physical USB character device
        if [ -c "$port" ]; then
            # Device exists: start agent if not already running
            if [ -z "$agent_pid" ] || ! kill -0 "$agent_pid" 2>/dev/null; then
                echo "[CONNECTED] Hardware detected on $node_name. Spawning agent..."
                ros2 run micro_ros_agent micro_ros_agent serial --dev "$port" -b 115200 --ros-args -r __node:="$node_name" &
                agent_pid=$!
            fi
        else
            # Device absent: kill agent immediately if running
            if [ -n "$agent_pid" ] && kill -0 "$agent_pid" 2>/dev/null; then
                echo "[DISCONNECTED] $node_name unplugged. Killing PID $agent_pid..."
                kill -9 "$agent_pid" 2>/dev/null
                agent_pid=""
                
                # Flush ROS 2 discovery daemon to remove ghost nodes
                ros2 daemon stop >/dev/null 2>&1
                ros2 daemon start >/dev/null 2>&1
            fi
        fi
        sleep 1
    done
}

# Clean shutdown on Ctrl+C or script termination
trap 'echo "Shutting down manager..."; pkill -9 -f micro_ros_agent; exit 0' EXIT INT TERM

# 5. Launch independent monitors for each port
manage_agent "$PORT_DRIVETRAIN" "agent_drivetrain" &
manage_agent "$PORT_SHOULDER1" "agent_shoulder_1" &
manage_agent "$PORT_SHOULDER2" "agent_shoulder_2" &

# Launch Foxglove Bridge separately
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765 &

wait

