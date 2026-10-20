#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, Int8


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

    'min_side_speed': 0.35,
    'max_side_speed': 3.50,
    'speed_step': 0.35,
    'wheel_separation': 0.582,

    'drum_speed_step': 500.0,
    'drum_speed_limit': 3000.0,
    'drum_steps_per_rev': 400.0,
    'front_drum_direction': 1.0,
    'rear_drum_direction': -1.0,

    # Arms: tap (< hold_threshold_s) = one step; hold = jog until release.
    # Limits are enforced on the arm Teensy (ARM_MIN_DEG / ARM_MAX_DEG).
    'arm_step_deg': 2.0,
    'hold_threshold_s': 0.3,
}


def clamp(value, low, high):
    return max(low, min(high, value))


class ArmButtons:
    """Tap / hold / conflict state for one arm's up + down inputs."""

    def __init__(self, name):
        self.name = name
        self.press_start = None   # Time the current single-direction press began
        self.press_dir = 0        # +1 up, -1 down, 0 none
        self.jogging = False      # Jog commands are being published
        self.locked = False       # Conflict / all-stop: wait for both released


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

        # Drive goes through cmd_mux (all stop > teleop > primitive > zero).
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel/teleop', 10)

        # Operator input while a primitive runs cancels it (primitive server).
        self.override_pub = self.create_publisher(Empty, '/ipex/teleop_override', 10)
        self.last_override_s = -1e9

        # ALL STOP -> every Teensy (drivetrain + arms) receives STOP.
        self.all_stop_pub = self.create_publisher(Empty, '/ipex/all_stop', 10)

        # HOME -> every connected arm Teensy receives CAL (arms MOVE).
        self.home_pub = self.create_publisher(Empty, '/ipex/home_arms', 10)

        # Arm jog (+1/-1 while held, 0 on release) and tap steps (deg).
        # IpexArmSystem stops a jog if /jog is not republished within 200 ms.
        self.jog_pub = {
            arm: self.create_publisher(Int8, f'/ipex/{arm}_arm/jog', 10)
            for arm in ('front', 'rear')
        }
        self.step_pub = {
            arm: self.create_publisher(Float64, f'/ipex/{arm}_arm/step', 10)
            for arm in ('front', 'rear')
        }

        # Arm homed status from IpexArmSystem (latched).
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/ipex/front_arm/homed',
            lambda msg: self.homed_callback('front', msg), latched)
        self.create_subscription(
            Bool, '/ipex/rear_arm/homed',
            lambda msg: self.homed_callback('rear', msg), latched)

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

        # Arms only accept presses once homed (reported by hardware).
        self.homed = {'front': False, 'rear': False}
        self.arms = {'front': ArmButtons('front'), 'rear': ArmButtons('rear')}

        # Previous input states for rising-edge detection.
        self.prev = {
            'a': False,
            'b': False,
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

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -----------------------------------------------------
    # Callbacks
    # -----------------------------------------------------

    def homed_callback(self, arm, msg):
        if msg.data == self.homed[arm]:
            return

        self.homed[arm] = msg.data
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

        self.signal_override_if_active(msg)

        self.handle_speed_bounds(msg)
        self.handle_drums(msg)
        self.handle_arms(msg)
        self.handle_drive(msg)

    def signal_override_if_active(self, msg):
        """Publish /ipex/teleop_override (max 2 Hz) while the operator is
        driving, moving an arm, or changing drum speed. A running motion
        primitive treats this as a cancel (OPERATOR_OVERRIDE)."""
        threshold = self.p['trigger_pressed_threshold']
        active = (
            abs(self.axis(msg, self.p['left_y_axis'])) > 0.15 or
            abs(self.axis(msg, self.p['right_y_axis'])) > 0.15 or
            abs(self.axis(msg, self.p['dpad_x_axis'])) > 0.5 or
            self.button(msg, self.p['left_bumper_button']) or
            self.button(msg, self.p['right_bumper_button']) or
            self.axis(msg, self.p['left_trigger_axis']) < threshold or
            self.axis(msg, self.p['right_trigger_axis']) < threshold
        )
        now = self.now_s()
        if active and now - self.last_override_s >= 0.5:
            self.override_pub.publish(Empty())
            self.last_override_s = now

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

        # Arms: end any jog and require all arm buttons released before
        # they respond again.
        for arm in self.arms.values():
            if arm.jogging:
                self.jog_pub[arm.name].publish(Int8(data=0))
            arm.jogging = False
            arm.press_start = None
            arm.press_dir = 0
            arm.locked = True

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

        # Rear arm: LB up, LT down. Front arm: RB up, RT down.
        rear_up = self.button(msg, self.p['left_bumper_button'])
        rear_down = self.axis(msg, self.p['left_trigger_axis']) < threshold
        front_up = self.button(msg, self.p['right_bumper_button'])
        front_down = self.axis(msg, self.p['right_trigger_axis']) < threshold

        now = self.now_s()
        self.update_arm(self.arms['rear'], rear_up, rear_down, now)
        self.update_arm(self.arms['front'], front_up, front_down, now)

    def update_arm(self, arm, up, down, now):
        label = arm.name.capitalize()

        # Locked after conflict / all stop: wait until both are released.
        if arm.locked:
            if not up and not down:
                arm.locked = False
            return

        # Conflict: both pressed -> stop, then lock until both released.
        if up and down:
            if arm.jogging:
                self.jog_pub[arm.name].publish(Int8(data=0))
                self.get_logger().warn(f'{label} arm: conflicting input, jog stopped.')
            arm.jogging = False
            arm.press_start = None
            arm.press_dir = 0
            arm.locked = True
            return

        direction = 1 if up else (-1 if down else 0)

        # Released.
        if direction == 0:
            if arm.jogging:
                self.jog_pub[arm.name].publish(Int8(data=0))
                self.get_logger().info(f'{label} arm: jog released.')
            elif arm.press_start is not None:
                # Released before the hold threshold: a tap.
                if now - arm.press_start < self.p['hold_threshold_s']:
                    self.tap_arm(arm, arm.press_dir)
            arm.jogging = False
            arm.press_start = None
            arm.press_dir = 0
            return

        # Direction switched between messages without both being seen
        # pressed: treat like a conflict.
        if arm.press_dir != 0 and direction != arm.press_dir:
            if arm.jogging:
                self.jog_pub[arm.name].publish(Int8(data=0))
                self.get_logger().warn(f'{label} arm: direction switched, jog stopped.')
            arm.jogging = False
            arm.press_start = None
            arm.press_dir = 0
            arm.locked = True
            return

        # New press: start timing.
        if arm.press_start is None:
            arm.press_start = now
            arm.press_dir = direction
            return

        # Held: past the threshold -> jog, republished every message.
        if now - arm.press_start >= self.p['hold_threshold_s']:
            if not self.homed[arm.name]:
                if not arm.jogging:
                    self.get_logger().warn(
                        f'{label} arm not homed; jog ignored. Press A to home.')
                    arm.locked = True    # One warning per press
                return
            if not arm.jogging:
                self.get_logger().info(
                    f'{label} arm: jog {"UP" if direction > 0 else "DOWN"}.')
            arm.jogging = True
            self.jog_pub[arm.name].publish(Int8(data=direction))

    def tap_arm(self, arm, direction):
        label = arm.name.capitalize()

        if not self.homed[arm.name]:
            self.get_logger().warn(
                f'{label} arm not homed; ignoring tap. Press A to home.')
            return

        step = direction * self.p['arm_step_deg']
        self.step_pub[arm.name].publish(Float64(data=step))
        self.get_logger().info(f'{label} arm: step {step:+.1f} deg.')

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
