import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.expanduser(
        '~/ros2_ws/src/ipex_simple.urdf'
    )

    # Start Gazebo with our known-good default world.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_dir,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r default.sdf',
            'on_exit_shutdown': 'true',
        }.items()
    )

    # Spawn rover.
    spawn_rover = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'default',
            '-name', 'ipex_simple',
            '-file', urdf_path,
            '-x', '0',
            '-y', '0',
            '-z', '0.3',
        ],
        output='screen'
    )

    # ROS -> Gazebo cmd_vel
    # Gazebo -> ROS odometry
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

	    '/model/ipex_simple/odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',

	    '/front_arm/target_angle'
    	    '@std_msgs/msg/Float64]gz.msgs.Double',

    	    '/rear_arm/target_angle'
    	    '@std_msgs/msg/Float64]gz.msgs.Double',

    	    '/front_drum/speed'
    	    '@std_msgs/msg/Float64]gz.msgs.Double',

    	    '/rear_drum/speed'
    	    '@std_msgs/msg/Float64]gz.msgs.Double',

            '/joint_states'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
	],
        output='screen'
    )

    # Our first motion primitive.
    drive_straight_server = Node(
        package='ipex_motion',
        executable='drive_straight_server',
        output='screen',
	)
    set_arm_angles_server = Node(
        package='ipex_motion',
        executable='set_arm_angles_server',
        output='screen',
	)
    set_drum_speeds_server = Node(
        package='ipex_motion',
        executable='set_drum_speeds_server',
        output='screen',
        )
    fake_payload_sensor = Node(
        package='ipex_motion',
        executable='fake_payload_sensor',
        output='screen',
        )
    excavate_server = Node(
        package='ipex_motion',
        executable='excavate_server',
	output='screen',
        )
    deposit_server = Node(
        package='ipex_motion',
        executable='deposit_server',
        output='screen',
        )

    return LaunchDescription([
        gazebo,

        # Give Gazebo a moment to create the world before spawning.
        TimerAction(
            period=2.0,
            actions=[spawn_rover]
        ),

        bridge,
        drive_straight_server,
	set_arm_angles_server,
	set_drum_speeds_server,
	fake_payload_sensor,
	excavate_server,
	deposit_server,
    ])
