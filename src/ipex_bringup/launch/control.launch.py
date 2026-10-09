from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    xacro_file = PathJoinSubstitution([
        FindPackageShare("ipex_description"),
        "urdf",
        "ipex.xacro",
    ])

    controllers_file = PathJoinSubstitution([
        FindPackageShare("ipex_bringup"),
        "config",
        "controllers.yaml",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " rear_arm:=", LaunchConfiguration("rear_arm"),
                " front_arm:=", LaunchConfiguration("front_arm"),
                " rear_arm_serial_device:=", LaunchConfiguration("rear_arm_serial_device"),
                " front_arm_serial_device:=", LaunchConfiguration("front_arm_serial_device"),
                " home_arms_on_start:=", LaunchConfiguration("home_arms_on_start"),
            ]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Rear arm controllers: only when rear_arm:=true.
    rear_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rear_shoulder_controller",
            "rear_drum_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rear_arm")),
    )

    # Front arm controllers: only when front_arm:=true
    # (front arm Teensy not yet installed).
    front_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "front_shoulder_controller",
            "front_drum_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("front_arm")),
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Defaults ARE the rover configuration: systemd autostart
    # (scripts/start_rover.sh) launches with no arguments.
    # Enable an arm only when its Teensy is connected.
    args = [
        DeclareLaunchArgument(
            "rear_arm",
            default_value="true",
            description="Enable rear arm hardware + controllers.",
        ),
        DeclareLaunchArgument(
            "front_arm",
            default_value="true",
            description="Enable front arm hardware + controllers.",
        ),
        DeclareLaunchArgument(
            "rear_arm_serial_device",
            default_value="/dev/serial/by-id/usb-Teensyduino_USB_Serial_19972490-if00",
            description="Rear arm Teensy serial device (ls /dev/serial/by-id/).",
        ),
        DeclareLaunchArgument(
            "front_arm_serial_device",
            default_value="/dev/serial/by-id/usb-Teensyduino_USB_Serial_20404840-if00",
            description="Front arm Teensy serial device (ls /dev/serial/by-id/).",
        ),
        DeclareLaunchArgument(
            "home_arms_on_start",
            default_value="false",
            description="Home arms automatically on startup (arms will MOVE).",
        ),
    ]

    return LaunchDescription(args + [
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        rear_arm_spawner,
        front_arm_spawner,
        diff_drive_controller_spawner,
    ])
