from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # Hardware + ros2_control bringup.
    # Arm args (rear_arm, front_arm, rear_arm_serial_device,
    # front_arm_serial_device, home_arms_on_start) are declared in
    # control.launch.py and can be passed straight to this file.
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ipex_bringup'),
                'launch',
                'control.launch.py',
            ])
        ),
    )

    # Foxglove WebSocket bridge.
    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            ])
        ),
        launch_arguments={
            'port': '8765',
        }.items(),
    )

    # Existing /cmd_vel -> diff_drive_controller bridge.
    cmd_vel_bridge = Node(
        package='ipex_motion',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
    )

    # F710 hand controller: tank drive, arms, drums, speed bounds.
    # Mapping and limits in ipex_bringup/config/teleop.yaml.
    controller = Node(
        package='ipex_motion',
        executable='controller',
        name='controller',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('ipex_bringup'),
                'config',
                'teleop.yaml',
            ])
        ],
        output='screen',
    )

    # Logitech F710.
    #
    # autorepeat matters because joy_linux otherwise defaults to sending
    # only when an input changes. Repeating at 20 Hz keeps held-stick
    # commands alive for the diff-drive controller timeout.
    #
    # respawn means if the F710 isn't present at boot and joy_linux exits,
    # ROS launch will keep trying rather than requiring an SSH session.
    joy = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
    )

    # Avionics and battery MCP9808 temperature sensors.
    temperature_node = Node(
        package='ipex_sensors',
        executable='temperature_node',
        name='temperature_node',
        output='screen',
    )

    return LaunchDescription([
        control,
        foxglove,
        cmd_vel_bridge,
        controller,
        temperature_node,
        joy,
    ])
