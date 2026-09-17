import time

import rclpy

from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool, Float64

from ipex_interfaces.action import Deposit, SetArmAngles
from ipex_interfaces.srv import SetDrumSpeeds


class DepositServer(Node):

    def __init__(self):
        super().__init__('deposit_server')

        self.callback_group = ReentrantCallbackGroup()

        self.current_mass = None

        # -----------------------------
        # Deposit configuration
        # -----------------------------

        self.front_deposit_angle = -0.2
        self.rear_deposit_angle = 0.2

        # Reverse of excavation direction
        self.front_drum_speed = -3.0
        self.rear_drum_speed = 3.0

        # Where arms return afterward
        self.reset_angle = 0.6

        self.mass_rate = 0.25
        self.mass_tolerance = 0.01

        # -----------------------------
        # ROS interfaces
        # -----------------------------

        self.payload_sub = self.create_subscription(
            Float64,
            '/payload_mass',
            self.payload_callback,
            10,
            callback_group=self.callback_group
        )

        self.deposit_active_pub = self.create_publisher(
            Bool,
            '/deposit_active',
            10
        )

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
            Deposit,
            '/deposit',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            'Deposit action server ready'
        )


    def payload_callback(self, msg):

        self.current_mass = msg.data


    def set_deposit_active(self, active):

        msg = Bool()
        msg.data = active

        self.deposit_active_pub.publish(msg)


    def set_drum_speeds(self, front, rear):

        if not self.drum_client.wait_for_service(
            timeout_sec=5.0
        ):
            return False

        request = SetDrumSpeeds.Request()

        request.front_rad_s = front
        request.rear_rad_s = rear

        future = self.drum_client.call_async(
            request
        )

        while rclpy.ok() and not future.done():
            time.sleep(0.05)

        if not future.done():
            return False

        response = future.result()

        return (
            response is not None and
            response.success
        )


    def move_arms(self, arm_angle):

        if not self.arm_client.wait_for_server(
            timeout_sec=5.0
        ):
            return False

        goal = SetArmAngles.Goal()

        goal.front_angle = arm_angle
        goal.rear_angle = arm_angle

        send_future = (
            self.arm_client.send_goal_async(goal)
        )

        while (
            rclpy.ok() and
            not send_future.done()
        ):
            time.sleep(0.05)

        if not send_future.done():
            return False

        goal_handle = send_future.result()

        if (
            goal_handle is None or
            not goal_handle.accepted
        ):
            return False

        result_future = (
            goal_handle.get_result_async()
        )

        while (
            rclpy.ok() and
            not result_future.done()
        ):
            time.sleep(0.05)

        if not result_future.done():
            return False

        return (
            result_future.result()
            .result
            .success
        )


    def stop_deposit(self):

        self.set_deposit_active(False)

        self.set_drum_speeds(
            0.0,
            0.0
        )


    def reset_arms(self):

        return self.move_arms(
            self.reset_angle
        )


    def execute_callback(self, goal_handle):

        result = Deposit.Result()
        feedback = Deposit.Feedback()

        deposit_mass = goal_handle.request.deposit_mass
        arm_angle = goal_handle.request.arm_angle

        # Wait until payload feedback exists
        while (
            self.current_mass is None and
            rclpy.ok()
        ):
            time.sleep(0.05)


        starting_mass = self.current_mass

        #check deposit > 0
        if deposit_mass <= 0.0:

            goal_handle.abort()

            result.success = False
            result.deposited_mass = 0.0
            result.message = (
                'Deposit mass must be greater than zero'
            )

            return result

        # Nothing to dump
        if starting_mass <= self.mass_tolerance:

            goal_handle.succeed()

            result.success = False
            result.deposited_mass = 0.0
            result.message = 'Payload already empty'

            return result

        if deposit_mass > (
            starting_mass + self.mass_tolerance
        ):

            goal_handle.abort()

            result.success = False
            result.deposited_mass = 0.0

            result.message = (
                f'Requested {deposit_mass:.2f} kg, '
                f'but payload only contains '
                f'{starting_mass:.2f} kg'
            )

            return result


        self.get_logger().info(
            f'Depositing {deposit_mass:.2f} kg'
            f'from {starting_mass:.2f} kg payload'
        )


        # -----------------------------
        # 1. Move arms to deposit pose
        # -----------------------------

        if not self.move_arms(
            arm_angle
        ):

            goal_handle.abort()

            result.success = False
            result.deposited_mass = 0.0
            result.message = (
                'Failed to position arms '
                'for deposit'
            )

            return result


        # -----------------------------
        # 2. Start drums
        # -----------------------------

        if not self.set_drum_speeds(
            self.front_drum_speed,
            self.rear_drum_speed
        ):

            goal_handle.abort()

            result.success = False
            result.deposited_mass = 0.0
            result.message = (
                'Failed to start deposit drums'
            )

            return result


        # -----------------------------
        # 3. Begin simulated deposition
        # -----------------------------

        self.set_deposit_active(True)

        start_time = time.time()

        try:

            while rclpy.ok():

                deposited_mass = (
                    starting_mass -
                    self.current_mass
                )

                mass_remaining = max(
                    0.0,
                    deposit_mass -
                    deposited_mass
                )

                # -------------------------
                # Cancellation
                # -------------------------

                if goal_handle.is_cancel_requested:

                    goal_handle.canceled()

                    result.success = False
                    result.deposited_mass = (
                        deposited_mass
                    )
                    result.message = (
                        'Deposit canceled'
                    )

                    return result


                # -------------------------
                # Success
                # -------------------------

                if (
                    deposited_mass >=
                    deposit_mass - self.mass_tolerance
                ):
                    break


                feedback.current_mass = (
                    self.current_mass
                )

                feedback.deposited_mass = (
                    deposited_mass
                )

                feedback.mass_remaining = (
                    mass_remaining
                )

                goal_handle.publish_feedback(
                    feedback
                )


                # -------------------------
                # Timeout
                # -------------------------

                expected_time = (
                    starting_mass /
                    self.mass_rate
                )

                if (
                    time.time() - start_time >
                    expected_time + 10.0
                ):

                    goal_handle.abort()

                    result.success = False
                    result.deposited_mass = (
                        deposited_mass
                    )
                    result.message = (
                        'Deposit timed out'
                    )

                    return result


                time.sleep(0.05)


        finally:

            self.stop_deposit()


        deposited_mass = (
            starting_mass -
            self.current_mass
        )


        # -----------------------------
        # 4. Reset arms
        # -----------------------------

        self.get_logger().info(
            'Resetting arms after deposit'
        )

        if not self.reset_arms():

            goal_handle.abort()

            result.success = False
            result.deposited_mass = (
                deposited_mass
            )
            result.message = (
                'Payload deposited, but '
                'failed to reset arms'
            )

            return result


        # -----------------------------
        # Done
        # -----------------------------

        goal_handle.succeed()

        result.success = True
        result.deposited_mass = (
            deposited_mass
        )
        result.message = (
            'Requested payload deposited and arms reset'
        )

        self.get_logger().info(
            f'Deposit complete: '
            f'{deposited_mass:.2f} kg'
        )

        return result


def main(args=None):

    rclpy.init(args=args)

    node = DepositServer()

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
