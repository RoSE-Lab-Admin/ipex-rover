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

    # Existing hardware + ros2_control bringup.
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ipex_bringup'),
                'launch',
                'control.launch.py',
            ])
        )
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

    # Existing KNOWN-GOOD controller.py.
    # We are deliberately NOT replacing this with today's modified version yet.
    controller = Node(
        package='ipex_motion',
        executable='controller',
        name='controller',
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

    return LaunchDescription([
        control,
        foxglove,
        cmd_vel_bridge,
        controller,
        joy,
    ])
