#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray


# Logitech F710 -- X mode (joy_linux). Defaults; overridden by
# ipex_bringup/config/teleop.yaml.
#
# User-facing units: arms in degrees, drums in firmware steps/s.
# ros2_control uses SI (rad, rad/s); conversion happens here on publish.
DEFAULT_PARAMS = {
    'left_y_axis': 1,
    'right_y_axis': 4,
    'left_trigger_axis': 2,
    'right_trigger_axis': 5,
    'dpad_x_axis': 6,
    'dpad_y_axis': 7,
    'home_button': 0,
    'all_stop_button': 1,
    'left_bumper_button': 4,
    'right_bumper_button': 5,
    'trigger_pressed_threshold': 0.0,

    'min_side_speed': 0.10,
    'max_side_speed': 0.90,
    'speed_step': 0.10,
    'wheel_separation': 0.582,

    'drum_speed_step': 500.0,
    'drum_speed_limit': 3000.0,
    'drum_steps_per_rev': 400.0,
    'front_drum_direction': 1.0,
    'rear_drum_direction': 1.0,

    'arm_step_deg': 5.0,
    'front_arm_min_deg': -40.0,
    'front_arm_max_deg': 45.0,
    'rear_arm_min_deg': -40.0,
    'rear_arm_max_deg': 45.0,
}

FRONT_SHOULDER_JOINT = 'front_shoulder_rev'
REAR_SHOULDER_JOINT = 'back_shoulder_rev'


def clamp(value, low, high):
    return max(low, min(high, value))


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        for name, default in DEFAULT_PARAMS.items():
            self.declare_parameter(name, default)

        self.p = {
            name: self.get_parameter(name).value
            for name in DEFAULT_PARAMS
        }

        # -------------------------------------------------
        # Interfaces
        # -------------------------------------------------

        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ALL STOP -> every Teensy (drivetrain + arms) receives STOP.
        self.all_stop_pub = self.create_publisher(Empty, '/ipex/all_stop', 10)

        # HOME -> every connected arm Teensy receives CAL (arms MOVE).
        self.home_pub = self.create_publisher(Empty, '/ipex/home_arms', 10)

        # Arm homed status from IpexArmSystem (latched).
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/ipex/front_arm/homed',
            lambda msg: self.homed_callback('front', msg), latched)
        self.create_subscription(
            Bool, '/ipex/rear_arm/homed',
            lambda msg: self.homed_callback('rear', msg), latched)

        self.front_shoulder_pub = self.create_publisher(
            Float64MultiArray, '/front_shoulder_controller/commands', 10)
        self.rear_shoulder_pub = self.create_publisher(
            Float64MultiArray, '/rear_shoulder_controller/commands', 10)

        self.front_drum_pub = self.create_publisher(
            Float64MultiArray, '/front_drum_controller/commands', 10)
        self.rear_drum_pub = self.create_publisher(
            Float64MultiArray, '/rear_drum_controller/commands', 10)

        # -------------------------------------------------
        # State
        # -------------------------------------------------

        # Start at lowest drive speed, drums stopped.
        self.max_side_speed = self.p['min_side_speed']
        self.drum_speed = 0.0

        # Latest measured shoulder angles (deg) from /joint_states.
        self.front_arm_measured = None
        self.rear_arm_measured = None

        # Commanded shoulder targets (deg). None = seed from the measured
        # angle on the next press, so presses step from where the arm is.
        self.front_arm_target = None
        self.rear_arm_target = None

        # Arms only accept presses once homed (reported by hardware).
        self.homed = {'front': False, 'rear': False}

        # Previous input states for rising-edge detection.
        self.prev = {
            'a': False,
            'b': False,
            'lb': False, 'rb': False, 'lt': False, 'rt': False,
            'dpad_up': False, 'dpad_down': False,
            'dpad_left': False, 'dpad_right': False,
        }

        self.get_logger().info(
            f'IPEX hand controller ready | '
            f'max speed = {self.max_side_speed:.1f} m/s | '
            f'drum speed = {self.drum_speed:.0f} steps/s | '
            f'B = ALL STOP'
        )

    # -----------------------------------------------------
    # Input helpers
    # -----------------------------------------------------

    @staticmethod
    def axis(msg, index):
        return msg.axes[index] if len(msg.axes) > index else 0.0

    @staticmethod
    def button(msg, index):
        return len(msg.buttons) > index and msg.buttons[index] == 1

    def rising(self, key, pressed):
        edge = pressed and not self.prev[key]
        self.prev[key] = pressed
        return edge

    # -----------------------------------------------------
    # Callbacks
    # -----------------------------------------------------

    def joint_state_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))

        if FRONT_SHOULDER_JOINT in positions:
            self.front_arm_measured = math.degrees(
                positions[FRONT_SHOULDER_JOINT])

        if REAR_SHOULDER_JOINT in positions:
            self.rear_arm_measured = math.degrees(
                positions[REAR_SHOULDER_JOINT])

    def homed_callback(self, arm, msg):
        if msg.data == self.homed[arm]:
            return

        self.homed[arm] = msg.data

        # Position reference changed: reseed target from measured angle.
        if arm == 'front':
            self.front_arm_target = None
        else:
            self.rear_arm_target = None

        self.get_logger().info(
            f'{arm.capitalize()} arm {"HOMED" if msg.data else "NOT homed"}.')

    def joy_callback(self, msg):
        # B: all stop. Takes priority over every other input this cycle.
        if self.rising('b', self.button(msg, self.p['all_stop_button'])):
            self.all_stop()
            return

        # A: home (calibrate) all connected arms.
        if self.rising('a', self.button(msg, self.p['home_button'])):
            self.home_pub.publish(Empty())
            self.get_logger().warn('HOME: calibrating arms (arms will move).')

        self.handle_speed_bounds(msg)
        self.handle_drums(msg)
        self.handle_arms(msg)
        self.handle_drive(msg)

    # -----------------------------------------------------
    # All stop
    # -----------------------------------------------------

    def all_stop(self):
        # Every Teensy: STOP (arms stop then brake; drums ramp to 0;
        # drivetrain motors stop).
        self.all_stop_pub.publish(Empty())

        # Drive: zero twist.
        self.cmd_pub.publish(Twist())

        # Drums: zero and stay zero until the D-pad changes them.
        self.drum_speed = 0.0
        self.publish_drums()

        # Arms: next press steps from wherever the arm stopped.
        self.front_arm_target = None
        self.rear_arm_target = None

        self.get_logger().warn('ALL STOP: drive, drums, and arms stopped.')

    # -----------------------------------------------------
    # Mapping
    # -----------------------------------------------------

    def handle_speed_bounds(self, msg):
        dpad_y = self.axis(msg, self.p['dpad_y_axis'])

        # D-pad up: + speed_step
        if self.rising('dpad_up', dpad_y > 0.5):
            self.max_side_speed = min(
                self.p['max_side_speed'],
                self.max_side_speed + self.p['speed_step'])
            self.get_logger().info(
                f'Max speed: {self.max_side_speed:.1f} m/s')

        # D-pad down: - speed_step
        if self.rising('dpad_down', dpad_y < -0.5):
            self.max_side_speed = max(
                self.p['min_side_speed'],
                self.max_side_speed - self.p['speed_step'])
            self.get_logger().info(
                f'Max speed: {self.max_side_speed:.1f} m/s')

    def handle_drums(self, msg):
        # joy_linux D-pad X: +1 = left, -1 = right
        dpad_x = self.axis(msg, self.p['dpad_x_axis'])
        limit = self.p['drum_speed_limit']
        changed = False

        # D-pad right: + drum_speed_step
        if self.rising('dpad_right', dpad_x < -0.5):
            self.drum_speed = clamp(
                self.drum_speed + self.p['drum_speed_step'], -limit, limit)
            changed = True

        # D-pad left: - drum_speed_step
        if self.rising('dpad_left', dpad_x > 0.5):
            self.drum_speed = clamp(
                self.drum_speed - self.p['drum_speed_step'], -limit, limit)
            changed = True

        # Drums are continuous: publish only on change.
        if changed:
            self.publish_drums()
            self.get_logger().info(
                f'Drum speed: {self.drum_speed:.0f} steps/s')

    def handle_arms(self, msg):
        threshold = self.p['trigger_pressed_threshold']
        step = self.p['arm_step_deg']

        lb = self.button(msg, self.p['left_bumper_button'])
        rb = self.button(msg, self.p['right_bumper_button'])
        lt = self.axis(msg, self.p['left_trigger_axis']) < threshold
        rt = self.axis(msg, self.p['right_trigger_axis']) < threshold

        rear_delta = 0.0
        front_delta = 0.0

        # Rear arm: LB up, LT down
        if self.rising('lb', lb):
            rear_delta += step
        if self.rising('lt', lt):
            rear_delta -= step

        # Front arm: RB up, RT down
        if self.rising('rb', rb):
            front_delta += step
        if self.rising('rt', rt):
            front_delta -= step

        if rear_delta != 0.0:
            self.rear_arm_target = self.step_arm(
                'rear', self.rear_arm_target, self.rear_arm_measured,
                rear_delta, self.p['rear_arm_min_deg'],
                self.p['rear_arm_max_deg'], self.rear_shoulder_pub)

        if front_delta != 0.0:
            self.front_arm_target = self.step_arm(
                'front', self.front_arm_target, self.front_arm_measured,
                front_delta, self.p['front_arm_min_deg'],
                self.p['front_arm_max_deg'], self.front_shoulder_pub)

    def step_arm(self, arm, target, measured, delta, low, high, pub):
        label = arm.capitalize()

        if not self.homed[arm]:
            self.get_logger().warn(
                f'{label} arm not homed; ignoring command. Press A to home.')
            return None

        if target is None:
            if measured is None:
                self.get_logger().warn(
                    f'{label} arm position unknown (no /joint_states yet); '
                    f'ignoring command.')
                return None
            target = measured

        target = clamp(target + delta, low, high)
        self.publish_arm(pub, target)
        self.get_logger().info(f'{label} arm target: {target:.1f} deg')
        return target

    def handle_drive(self, msg):
        # Tank drive
        left_speed = self.axis(msg, self.p['left_y_axis']) * self.max_side_speed
        right_speed = self.axis(msg, self.p['right_y_axis']) * self.max_side_speed

        cmd = Twist()
        cmd.linear.x = (left_speed + right_speed) / 2.0
        cmd.angular.z = (right_speed - left_speed) / self.p['wheel_separation']

        self.cmd_pub.publish(cmd)

    # -----------------------------------------------------
    # Output helpers
    # -----------------------------------------------------

    @staticmethod
    def publish_arm(pub, target_deg):
        pub.publish(Float64MultiArray(data=[math.radians(target_deg)]))

    def publish_drums(self):
        # steps/s -> rad/s for ros2_control. Hardware converts back.
        rad_s = self.drum_speed * 2.0 * math.pi / self.p['drum_steps_per_rev']

        front = rad_s * self.p['front_drum_direction']
        rear = rad_s * self.p['rear_drum_direction']

        # Both drums on an arm get the same command (turn in sync).
        self.front_drum_pub.publish(Float64MultiArray(data=[front, front]))
        self.rear_drum_pub.publish(Float64MultiArray(data=[rear, rear]))


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
