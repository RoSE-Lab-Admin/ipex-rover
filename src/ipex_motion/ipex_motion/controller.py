#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


# Logitech F710 -- X mode
LEFT_Y_AXIS = 1
LEFT_TRIGGER_AXIS = 2
RIGHT_Y_AXIS = 4

LEFT_BUMPER_BUTTON = 4
RIGHT_BUMPER_BUTTON = 5

# Adjustable tank-drive speed
MIN_SIDE_SPEED = 0.10
MAX_SIDE_SPEED = 0.90
SPEED_STEP = 0.10

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

        # Start at lowest speed.
        self.max_side_speed = MIN_SIDE_SPEED

        # Previous button states for rising-edge detection.
        self.previous_left_bumper = False
        self.previous_right_bumper = False

        self.get_logger().info(
            f'IPEX hand controller ready | '
            f'max speed = {self.max_side_speed:.1f} m/s'
        )


    def joy_callback(self, msg):

        cmd = Twist()

        # -------------------------------------------------
        # Speed adjustment
        # -------------------------------------------------

        left_bumper = (
            len(msg.buttons) > LEFT_BUMPER_BUTTON
            and msg.buttons[LEFT_BUMPER_BUTTON] == 1
        )

        right_bumper = (
            len(msg.buttons) > RIGHT_BUMPER_BUTTON
            and msg.buttons[RIGHT_BUMPER_BUTTON] == 1
        )

        # RB pressed: +0.1 m/s
        if right_bumper and not self.previous_right_bumper:
            self.max_side_speed = min(
                MAX_SIDE_SPEED,
                self.max_side_speed + SPEED_STEP
            )

            self.get_logger().info(
                f'Max speed: {self.max_side_speed:.1f} m/s'
            )

        # LB pressed: -0.1 m/s
        if left_bumper and not self.previous_left_bumper:
            self.max_side_speed = max(
                MIN_SIDE_SPEED,
                self.max_side_speed - SPEED_STEP
            )

            self.get_logger().info(
                f'Max speed: {self.max_side_speed:.1f} m/s'
            )

        self.previous_left_bumper = left_bumper
        self.previous_right_bumper = right_bumper


        # -------------------------------------------------
        # Deadman
        # -------------------------------------------------

        # LT:
        # +1.0 = released
        # -1.0 = fully pressed
        deadman = msg.axes[LEFT_TRIGGER_AXIS] < 0.0

        if not deadman:
            self.cmd_pub.publish(cmd)
            return


        # -------------------------------------------------
        # Tank drive
        # -------------------------------------------------

        left = msg.axes[LEFT_Y_AXIS]
        right = msg.axes[RIGHT_Y_AXIS]

        left_speed = left * self.max_side_speed
        right_speed = right * self.max_side_speed

        cmd.linear.x = (
            left_speed + right_speed
        ) / 2.0

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
