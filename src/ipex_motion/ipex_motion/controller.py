#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


# PowerA / Xbox controller mapping
# ROS arrays are zero-indexed.
LEFT_Y_AXIS = 1
LEFT_TRIGGER_AXIS = 2
RIGHT_Y_AXIS = 4

# Start slow for physical testing.
MAX_SIDE_SPEED = 0.10  # m/s

# Replace this with the value from:
# ros2 param get /diff_drive_controller wheel_separation
WHEEL_SEPARATION = 0.582


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info('IPEX hand controller ready')


    def joy_callback(self, msg):

        cmd = Twist()

        # Left trigger:
        # +1.0 = released
        # -1.0 = fully pressed
        #
        # Rover only moves while trigger is held past halfway.
        deadman = msg.axes[LEFT_TRIGGER_AXIS] < 0.0

        if not deadman:
            self.cmd_pub.publish(cmd)
            return

        # Your sticks report negative when pushed forward,
        # so invert them.
        left = msg.axes[LEFT_Y_AXIS]
        right = msg.axes[RIGHT_Y_AXIS]

        # Treat each stick as the desired speed of that side.
        left_speed = left * MAX_SIDE_SPEED
        right_speed = right * MAX_SIDE_SPEED

        # Convert the left/right side commands into the
        # differential-drive Twist representation.
        cmd.linear.x = (left_speed + right_speed) / 2.0
        cmd.angular.z = (
            right_speed - left_speed
        ) / WHEEL_SEPARATION

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
