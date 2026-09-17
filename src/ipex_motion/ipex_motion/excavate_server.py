import time

import rclpy

from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64

from ipex_interfaces.action import Excavate, SetArmAngles
from ipex_interfaces.srv import SetDrumSpeeds


class ExcavateServer(Node):

    def __init__(self):
        super().__init__('excavate_server')

        self.callback_group = ReentrantCallbackGroup()

        self.current_mass = None

        # Excavation configuration
        # Easy to tune later without changing the Action interface.
        self.front_excavation_angle = 0.2
        self.rear_excavation_angle = 0.2

        self.front_drum_speed = 3.0
        self.rear_drum_speed = -3.0

        self.drive_speed = 0.10

        self.mass_rate = 0.25
        self.mass_tolerance = 0.01

        self.front_reset_angle = 0.6
        self.rear_reset_angle = 0.6

        # Drivetrain
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Fake excavation model
        self.excavation_active_pub = self.create_publisher(
            Bool,
            '/excavation_active',
            10
        )

        # Payload feedback
        self.payload_sub = self.create_subscription(
            Float64,
            '/payload_mass',
            self.payload_callback,
            10,
            callback_group=self.callback_group
        )

        # Existing mechanism interfaces
        self.arm_client = ActionClient(
            self,
            SetArmAngles,
            '/set_arm_angles',
            callback_group=self.callback_group
        )

        self.drum_client = self.create_client(
            SetDrumSpeeds,
            '/set_drum_speeds',
            callback_group=self.callback_group
        )

        self.action_server = ActionServer(
            self,
            Excavate,
            '/excavate',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Excavate action server ready')


    def payload_callback(self, msg):
        self.current_mass = msg.data


    def set_excavation_active(self, active):

        msg = Bool()
        msg.data = active

        self.excavation_active_pub.publish(msg)


    def drive_forward(self):

        msg = Twist()
        msg.linear.x = self.drive_speed

        self.cmd_vel_pub.publish(msg)


    def stop_rover(self):

        self.cmd_vel_pub.publish(Twist())


    def set_drum_speeds(self, front, rear):

        if not self.drum_client.wait_for_service(timeout_sec=5.0):
            return False

        request = SetDrumSpeeds.Request()

        request.front_rad_s = front
        request.rear_rad_s = rear

        future = self.drum_client.call_async(request)

        while rclpy.ok() and not future.done():
            time.sleep(0.05)

        if not future.done():
            return False

        response = future.result()

        return response is not None and response.success


    def move_arms_to_excavation_pose(self, arm_angle):

        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            return False

        goal = SetArmAngles.Goal()

        goal.front_angle = arm_angle
        goal.rear_angle = arm_angle

        send_future = self.arm_client.send_goal_async(goal)

        while rclpy.ok() and not send_future.done():
            time.sleep(0.05)

        if not send_future.done():
            return False

        arm_goal_handle = send_future.result()

        if (
            arm_goal_handle is None or
            not arm_goal_handle.accepted
        ):
            return False

        result_future = arm_goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            time.sleep(0.05)

        if not result_future.done():
            return False

        arm_result = result_future.result().result

        return arm_result.success


    def stop_excavation(self):

        self.stop_rover()

        self.set_drum_speeds(
            0.0,
            0.0
        )

        self.set_excavation_active(False)

    def reset_arms(self):

        goal = SetArmAngles.Goal()

        goal.front_angle = self.front_reset_angle
        goal.rear_angle = self.rear_reset_angle

        send_future = self.arm_client.send_goal_async(goal)

        while rclpy.ok() and not send_future.done():
            time.sleep(0.05)

        if not send_future.done():
            return False

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            time.sleep(0.05)

        if not result_future.done():
            return False

        return result_future.result().result.success


    def execute_callback(self, goal_handle):

        target_mass = goal_handle.request.target_mass
        arm_angle = goal_handle.request.arm_angle

        result = Excavate.Result()
        feedback = Excavate.Feedback()

        if target_mass <= 0.0:

            goal_handle.abort()

            result.success = False
            result.excavated_mass = 0.0
            result.message = 'Target mass must be greater than zero'

            return result


        # Wait for payload feedback
        while self.current_mass is None and rclpy.ok():
            time.sleep(0.05)


        # 1. Position both arms
        self.get_logger().info(
            'Moving arms to excavation pose'
        )

        if not self.move_arms_to_excavation_pose(arm_angle):

            goal_handle.abort()

            result.success = False
            result.excavated_mass = 0.0
            result.message = 'Failed to position arms'

            return result


        # 2. Start both drums
        self.get_logger().info(
            'Starting excavation drums'
        )

        if not self.set_drum_speeds(
            self.front_drum_speed,
            self.rear_drum_speed
        ):

            goal_handle.abort()

            result.success = False
            result.excavated_mass = 0.0
            result.message = 'Failed to start drums'

            return result


        starting_mass = self.current_mass

        # 3. Enable simulated excavation
        self.set_excavation_active(True)

        start_time = time.time()

        self.get_logger().info(
            f'Excavating {target_mass:.2f} kg'
        )


        try:

            while rclpy.ok():

                excavated_mass = (
                    self.current_mass - starting_mass
                )

                remaining = max(
                    0.0,
                    target_mass - excavated_mass
                )


                # Cancellation
                if goal_handle.is_cancel_requested:

                    goal_handle.canceled()

                    result.success = False
                    result.excavated_mass = excavated_mass
                    result.message = 'Excavation canceled'

                    return result


                # Success condition
                if excavated_mass >= (
                    target_mass - self.mass_tolerance
                ):
                    break


                # Keep drivetrain command alive
                self.drive_forward()


                feedback.excavated_mass = excavated_mass
                feedback.mass_remaining = remaining

                goal_handle.publish_feedback(feedback)


                # Expected duration + generous safety margin
                expected_time = (
                    target_mass / self.mass_rate
                )

                if (
                    time.time() - start_time >
                    expected_time + 10.0
                ):

                    goal_handle.abort()

                    result.success = False
                    result.excavated_mass = excavated_mass
                    result.message = 'Excavation timed out'

                    return result


                time.sleep(0.05)


        finally:

            # Always leave the rover safe
            self.stop_excavation()


        excavated_mass = (
            self.current_mass - starting_mass
        )

        self.get_logger().info(
	    f'Resetting arms to '
	    f'front={self.front_reset_angle:.2f}, '
	    f'rear={self.rear_reset_angle:.2f}'
	)

        if not self.reset_arms():
            goal_handle.abort()

            result.success = False
            result.excavated_mass = excavated_mass
            result.message = 'Excavation completed, but failed to reset arms'

            return result

        goal_handle.succeed()

        result.success = True
        result.excavated_mass = excavated_mass
        result.message = 'Target mass excavated'

        self.get_logger().info(
            f'Excavation complete: '
            f'{excavated_mass:.2f} kg'
        )

        return result


def main(args=None):

    rclpy.init(args=args)

    node = ExcavateServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
