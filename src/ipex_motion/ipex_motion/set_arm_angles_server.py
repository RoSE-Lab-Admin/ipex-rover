import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from ipex_interfaces.action import SetArmAngles


class SetArmAnglesServer(Node):

    def __init__(self):
        super().__init__('set_arm_angles_server')

        self.callback_group = ReentrantCallbackGroup()

        self.front_angle = None
        self.rear_angle = None

        self.min_angle = -1.20
        self.max_angle = 1.20
        self.tolerance = 0.02

        self.front_pub = self.create_publisher(
            Float64,
            '/front_arm/target_angle',
            10
        )

        self.rear_pub = self.create_publisher(
            Float64,
            '/rear_arm/target_angle',
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )

        self.action_server = ActionServer(
            self,
            SetArmAngles,
            '/set_arm_angles',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('SetArmAngles action server ready')


    def joint_state_callback(self, msg):

        if 'front_arm_joint' in msg.name:
            i = msg.name.index('front_arm_joint')

            if i < len(msg.position):
                self.front_angle = -msg.position[i]

        if 'rear_arm_joint' in msg.name:
            i = msg.name.index('rear_arm_joint')

            if i < len(msg.position):
                self.rear_angle = msg.position[i]


    def command_angles(self, front, rear):


        front_msg = Float64()
        front_msg.data = -front
        self.front_pub.publish(front_msg)

        rear_msg = Float64()
        rear_msg.data = rear
        self.rear_pub.publish(rear_msg)


    def execute_callback(self, goal_handle):

        target_front = goal_handle.request.front_angle
        target_rear = goal_handle.request.rear_angle

        result = SetArmAngles.Result()
        feedback = SetArmAngles.Feedback()

        # Validate targets
        if (
            target_front < self.min_angle or
            target_front > self.max_angle or
            target_rear < self.min_angle or
            target_rear > self.max_angle
        ):
            goal_handle.abort()

            result.success = False
            result.final_front_angle = self.front_angle or 0.0
            result.final_rear_angle = self.rear_angle or 0.0
            result.message = 'Requested arm angle outside joint limits'

            return result

        # Wait for feedback
        while (
            self.front_angle is None or
            self.rear_angle is None
        ):
            time.sleep(0.05)

        self.command_angles(
            target_front,
            target_rear
        )

        start_time = time.time()

        while rclpy.ok():

            front_error = target_front - self.front_angle
            rear_error = target_rear - self.rear_angle

            if goal_handle.is_cancel_requested:

                self.command_angles(
                    self.front_angle,
                    self.rear_angle
                )

                goal_handle.canceled()

                result.success = False
                result.final_front_angle = self.front_angle
                result.final_rear_angle = self.rear_angle
                result.message = 'Arm motion canceled'

                return result

            # Both arms must reach target
            if (
                abs(front_error) <= self.tolerance and
                abs(rear_error) <= self.tolerance
            ):
                break

            if time.time() - start_time > 10.0:

                self.command_angles(
                    self.front_angle,
                    self.rear_angle
                )

                goal_handle.abort()

                result.success = False
                result.final_front_angle = self.front_angle
                result.final_rear_angle = self.rear_angle
                result.message = 'Arm motion timed out'

                return result

            feedback.current_front_angle = self.front_angle
            feedback.current_rear_angle = self.rear_angle
            feedback.front_error = front_error
            feedback.rear_error = rear_error

            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)

        goal_handle.succeed()

        result.success = True
        result.final_front_angle = self.front_angle
        result.final_rear_angle = self.rear_angle
        result.message = 'Both arm targets reached'

        return result


def main(args=None):

    rclpy.init(args=args)

    node = SetArmAnglesServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
