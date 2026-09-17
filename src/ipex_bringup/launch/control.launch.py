from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
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

    shoulder_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "shoulder_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    drum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "drum_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
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

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        shoulder_controller_spawner,
        drum_controller_spawner,
        diff_drive_controller_spawner,
    ])
