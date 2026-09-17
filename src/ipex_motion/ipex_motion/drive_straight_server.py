import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from ipex_interfaces.action import DriveStraight


class DriveStraightServer(Node):

    def __init__(self):
        super().__init__('drive_straight_server')

        # Make these configurable so simulation / hardware can use
        # different topic names without changing our action code.
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter(
            'odom_topic',
            '/model/ipex_simple/odometry'
        )

        cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).get_parameter_value().string_value

        odom_topic = self.get_parameter(
            'odom_topic'
        ).get_parameter_value().string_value

        self.callback_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        self.action_server = ActionServer(
            self,
            DriveStraight,
            'drive_straight',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )

        self.current_x = None
        self.current_y = None

        self.get_logger().info('DriveStraight action server ready')


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y


    def stop_rover(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)


    def execute_callback(self, goal_handle):
        self.get_logger().info('DriveStraight goal received')

        target_distance = goal_handle.request.distance
        requested_speed = goal_handle.request.speed

        result = DriveStraight.Result()
        feedback = DriveStraight.Feedback()

        # Wait briefly for odometry if we have not received any yet.
        while self.current_x is None or self.current_y is None:
            time.sleep(0.05)

        start_x = self.current_x
        start_y = self.current_y

        # Positive distance = forward
        # Negative distance = reverse
        direction = 1.0 if target_distance >= 0.0 else -1.0

        target_distance = abs(target_distance)
        speed = abs(requested_speed) * direction

        cmd = Twist()
        cmd.linear.x = speed

        while rclpy.ok():

            # Allow the action to be cancelled.
            if goal_handle.is_cancel_requested:
                self.stop_rover()
                goal_handle.canceled()

                result.success = False
                result.final_distance = 0.0
                result.message = 'Goal canceled'

                return result

            dx = self.current_x - start_x
            dy = self.current_y - start_y

            distance_traveled = math.sqrt(dx * dx + dy * dy)

            # Finished?
            if distance_traveled >= target_distance:
                break

            self.cmd_vel_pub.publish(cmd)

            feedback.distance_traveled = distance_traveled
            feedback.distance_remaining = (
                target_distance - distance_traveled
            )

            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)

        self.stop_rover()

        goal_handle.succeed()

        result.success = True
        result.final_distance = distance_traveled
        result.message = 'Target distance reached'

        self.get_logger().info(
            f'Goal complete: {distance_traveled:.3f} m'
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    node = DriveStraightServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
