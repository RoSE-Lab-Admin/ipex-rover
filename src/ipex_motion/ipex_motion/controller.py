#!/usr/bin/env python3

import math
import os
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, Int8


# Logitech F710 -- X mode (joy_linux). Defaults; overridden by
# ipex_bringup/config/teleop.yaml.
#
# User-facing units: arms in degrees, drums in drum RPM (after the gearbox).
# ros2_control uses SI (rad, rad/s at the drum); conversion happens here on publish.
DEFAULT_PARAMS = {
    'left_y_axis': 1,
    'right_y_axis': 4,
    'left_trigger_axis': 2,
    'right_trigger_axis': 5,
    'dpad_x_axis': 6,
    'dpad_y_axis': 7,
    'home_button': 0,
    'all_stop_button': 1,
    'excavate_button': 2,      # X
    'excavate_reset_button': 3,  # Y (mode off): forget saved dig settings
    'left_bumper_button': 4,
    'right_bumper_button': 5,
    'trigger_pressed_threshold': 0.0,

    'min_side_speed': 0.35,
    'max_side_speed': 3.50,
    'speed_step': 0.35,
    'wheel_separation': 0.582,

    'drum_rpm_step': 1.0,              # drum RPM per D-pad press
    'drum_max_steps_s': 3200.0,        # firmware cap (DRUM_MAX_STEPS_S) = 6.0 drum RPM
    'drum_motor_steps_per_rev': 400.0,  # firmware MOTOR_STEPS_PER_REV
    'drum_gear_ratio': 80.0,           # motor turns per drum turn
    'front_drum_direction': -1.0,
    'rear_drum_direction': -1.0,

    # Arms: tap (< hold_threshold_s) = one step; hold = jog until release.
    # Limits are enforced on the arm Teensy (ARM_MIN_DEG / ARM_MAX_DEG).
    'arm_step_deg': 0.25,
    'hold_threshold_s': 0.3,
    'arm_speed_deg_s': 5.625,          # normal arm speed (firmware default)

    # X: excavation mode. Drums spin up, then both arms lower to the dig
    # angle, then the rover crawls. While in the mode the D-pad, arm buttons
    # and sticks use the fine increments below. X again (or B) ends it:
    # arms stay where they are, wheels stop, then drums stop (always last).
    # However the run ends (X, B, or the lowering timeout), each arm's angle,
    # the drum RPM and the crawl speed are saved to excavate_settings_file;
    # the next X starts from them. Y (mode off) goes back to the defaults below.
    'excavate_settings_file': '~/roxey_data/excavation_mode.yaml',
    'excavate_dig_angle_deg': -25.0,   # default, both arms
    'excavate_drum_rpm': 2.0,          # default drum RPM, dig direction (= D-pad right)
    'excavate_drum_rpm_step': 0.25,    # D-pad left/right
    'excavate_crawl_mps': 0.03,        # default crawl speed
    'excavate_crawl_step_mps': 0.01,   # D-pad up/down
    'excavate_crawl_max_mps': 0.10,
    'excavate_steer_max_rad_s': 0.05,  # sticks: steering trim, (right - left) / 2 x this
    'excavate_arm_step_deg': 0.5,     # tap
    'excavate_arm_speed_deg_s': 2.5,   # jog / tap speed once digging
    'excavate_spinup_s': 1.0,          # drums spin this long before the arms lower
    'excavate_arm_tolerance_deg': 0.5,
    'excavate_lower_timeout_s': 30.0,  # arms not at the dig angle by then -> leave the mode
    'excavate_stop_wait_s': 3.0,       # on exit: max wait for odometry to show wheels stopped
    'excavate_stop_delay_s': 0.5,      # then drums keep spinning this long
}

SHOULDER_JOINTS = {'front': 'front_shoulder_rev', 'rear': 'back_shoulder_rev'}
STOPPED_MPS = 0.005      # odometry below these = wheels stopped
STOPPED_RAD_S = 0.01


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

        # Excavation mode: arm goto / speed, arm angles and odometry for its
        # sequencing, and a latched on/off flag so bags can be split by it.
        self.goto_pub = {
            arm: self.create_publisher(Float64, f'/ipex/{arm}_arm/goto', 10)
            for arm in ('front', 'rear')
        }
        self.arm_speed_pub = {
            arm: self.create_publisher(Float64, f'/ipex/{arm}_arm/speed', 10)
            for arm in ('front', 'rear')
        }
        self.create_subscription(JointState, '/joint_states', self.joints_callback, 10)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom',
            lambda msg: setattr(self, 'odom', (msg, time.monotonic())), 10)
        self.excavation_pub = self.create_publisher(Bool, '/ipex/excavation_mode', latched)
        self.excavation_pub.publish(Bool(data=False))
        self.create_timer(0.05, self.excavation_tick)

        # -------------------------------------------------
        # State
        # -------------------------------------------------

        # Start at lowest drive speed, drums stopped.
        self.max_side_speed = self.p['min_side_speed']
        self.drum_rpm = 0.0     # signed drum RPM (+ = D-pad right)

        # Arms only accept presses once homed (reported by hardware).
        self.homed = {'front': False, 'rear': False}
        self.arms = {'front': ArmButtons('front'), 'rear': ArmButtons('rear')}
        self.arm_angle = {}               # arm -> (deg, monotonic time)
        self.odom = (None, -1e9)

        # Excavation mode phase: off | spinup | lowering | digging | stopping
        self.dig_phase = 'off'
        self.dig_phase_start = 0.0
        self.crawl_mps = self.p['excavate_crawl_mps']
        self.wheels_stopped_at = None
        self.dig_target = {}              # arm -> dig angle (deg) this cycle
        self.dig_settings = self.load_dig_settings()

        # Previous input states for rising-edge detection.
        self.prev = {
            'a': False,
            'b': False,
            'x': False,
            'y': False,
            'dpad_up': False, 'dpad_down': False,
            'dpad_left': False, 'dpad_right': False,
        }

        self.get_logger().info(
            f'IPEX hand controller ready | '
            f'max speed = {self.max_side_speed:.1f} m/s | '
            f'drum speed = {self.drum_rpm:.1f} RPM (step {self.p["drum_rpm_step"]:g}, '
            f'max {self.drum_max_rpm():.1f}) | '
            f'X = excavation mode | B = ALL STOP'
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

        # X: excavation mode on / off.
        x_pressed = self.rising('x', self.button(msg, self.p['excavate_button']))
        if x_pressed:
            if self.dig_phase == 'off':
                self.start_excavation()
            elif self.dig_phase != 'stopping':
                self.stop_excavation('X pressed')

        # Y (mode off): back to the default dig settings.
        if self.rising('y', self.button(msg, self.p['excavate_reset_button'])):
            if self.dig_phase == 'off':
                self.dig_settings = self.default_dig_settings()
                self.save_dig_settings()
                self.get_logger().warn(
                    f'Excavation settings reset to defaults: {self.describe_dig_settings()}.')
            else:
                self.get_logger().warn('Y ignored: end excavation mode (X) first.')

        self.signal_override_if_active(msg, x_pressed)

        self.handle_speed_bounds(msg)
        self.handle_drums(msg)
        # Arm buttons wait until the arms reach the dig angle.
        if self.dig_phase not in ('spinup', 'lowering'):
            self.handle_arms(msg)
        self.handle_drive(msg)

    def joints_callback(self, msg):
        now = time.monotonic()
        positions = dict(zip(msg.name, msg.position))
        for arm, joint in SHOULDER_JOINTS.items():
            if joint in positions and math.isfinite(positions[joint]):
                self.arm_angle[arm] = (math.degrees(positions[joint]), now)

    def arm_angle_deg(self, arm):
        value, t = self.arm_angle.get(arm, (None, -1e9))
        return value if time.monotonic() - t <= 1.0 else None

    def signal_override_if_active(self, msg, x_pressed=False):
        """Publish /ipex/teleop_override (max 2 Hz) while the operator is
        driving, moving an arm, or changing drum speed, and throughout
        excavation mode. A running motion primitive treats this as a cancel
        (OPERATOR_OVERRIDE)."""
        threshold = self.p['trigger_pressed_threshold']
        active = (
            x_pressed or self.dig_phase != 'off' or
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

        # Excavation run in progress: keep its tuning for the next X.
        if self.dig_phase not in ('off', 'stopping'):
            self.save_run_settings()

        # Drive: zero twist.
        self.cmd_pub.publish(Twist())

        # Drums: zero and stay zero until the D-pad changes them.
        self.drum_rpm = 0.0
        self.publish_drums()

        # Arms: end any jog and require all arm buttons released before
        # they respond again.
        self.release_arm_inputs()

        if self.dig_phase != 'off':
            self.set_arm_speed(self.p['arm_speed_deg_s'])
            self.set_dig_phase('off')

        self.get_logger().warn('ALL STOP: drive, drums, and arms stopped.')

    def release_arm_inputs(self):
        """End any jog; arm buttons respond again once all are released."""
        for arm in self.arms.values():
            if arm.jogging:
                self.jog_pub[arm.name].publish(Int8(data=0))
            arm.jogging = False
            arm.press_start = None
            arm.press_dir = 0
            arm.locked = True

    # -----------------------------------------------------
    # Excavation mode (X)
    # -----------------------------------------------------
    # spinup   drums at the saved drum RPM, rover still, arms still
    # lowering each arm goes to its saved dig angle, rover still
    # digging  saved crawl speed; fine D-pad / arm / steering adjustments
    # stopping wheels stop, then drums stop (drums are always last)
    #
    # Saved settings (dig_settings): {front_deg, rear_deg, drum_rpm, crawl_mps},
    # drum_rpm as a dig-direction magnitude. Written however a run ends.

    def default_dig_settings(self):
        angle = float(self.p['excavate_dig_angle_deg'])
        return {'front_deg': angle, 'rear_deg': angle,
                'drum_rpm': abs(float(self.p['excavate_drum_rpm'])),
                'crawl_mps': float(self.p['excavate_crawl_mps'])}

    def settings_path(self):
        return os.path.expanduser(self.p['excavate_settings_file'])

    def load_dig_settings(self):
        settings = self.default_dig_settings()
        path = self.settings_path()
        try:
            with open(path) as f:
                saved = yaml.safe_load(f) or {}
            for key in settings:
                if key in saved:
                    settings[key] = float(saved[key])
            self.get_logger().info(
                f'Excavation settings from {path}: {self.describe_dig_settings(settings)}')
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, yaml.YAMLError) as e:
            self.get_logger().warn(f'Could not read {path} ({e}); using defaults.')
        return settings

    def save_dig_settings(self):
        path = self.settings_path()
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w') as f:
                f.write('# Excavation mode (X) start settings; written by controller.py\n'
                        '# when a run ends. Y on the controller resets to defaults.\n')
                yaml.safe_dump({k: round(v, 3) for k, v in self.dig_settings.items()},
                               f, sort_keys=False)
        except OSError as e:
            self.get_logger().error(f'Could not save excavation settings to {path}: {e}')

    def describe_dig_settings(self, s=None):
        s = s or self.dig_settings
        return (f'front {s["front_deg"]:.2f} deg, rear {s["rear_deg"]:.2f} deg, '
                f'drums {s["drum_rpm"]:.2f} RPM, crawl {s["crawl_mps"]:.2f} m/s')

    def set_dig_phase(self, phase):
        self.dig_phase = phase
        self.dig_phase_start = time.monotonic()
        if phase in ('off', 'spinup'):
            self.excavation_pub.publish(Bool(data=phase != 'off'))

    def set_arm_speed(self, deg_s):
        for arm in ('front', 'rear'):
            self.arm_speed_pub[arm].publish(Float64(data=float(deg_s)))

    def start_excavation(self):
        missing = [a for a in ('front', 'rear') if not self.homed[a]]
        if missing:
            self.get_logger().warn(
                f'Excavation mode: {" and ".join(missing)} arm not homed. Press A to home.')
            return
        if any(self.arm_angle_deg(a) is None for a in ('front', 'rear')):
            self.get_logger().warn('Excavation mode: no arm angle feedback (/joint_states).')
            return

        s = self.dig_settings
        drum = clamp(s['drum_rpm'], 0.0, self.drum_max_rpm())
        if drum <= 0.0:
            # Never lower the arms onto stopped drums.
            drum = abs(self.p['excavate_drum_rpm'])
            self.get_logger().warn(f'Saved drum speed is 0; using the default {drum:g} RPM.')

        self.release_arm_inputs()
        self.dig_target = {'front': s['front_deg'], 'rear': s['rear_deg']}
        self.crawl_mps = clamp(s['crawl_mps'], 0.0, self.p['excavate_crawl_max_mps'])
        self.drum_rpm = drum      # dig = D-pad right
        self.publish_drums()
        self.set_dig_phase('spinup')
        self.get_logger().warn(
            f'EXCAVATION MODE: {self.describe_dig_settings()}. Drums spinning up; '
            f'arms lower next. X or B = stop (both save settings).')

    def save_run_settings(self):
        """Save the current run's tuning for the next X, whatever phase it is in."""
        # Arm angles: measured once digging (includes the taps / jogs); before
        # that the arms haven't reached their targets, so keep the targets.
        if self.dig_phase == 'digging':
            for arm in ('front', 'rear'):
                angle = self.arm_angle_deg(arm)
                if angle is not None:
                    self.dig_settings[f'{arm}_deg'] = angle
        if self.drum_rpm > 0.0:     # still digging direction
            self.dig_settings['drum_rpm'] = self.drum_rpm
        self.dig_settings['crawl_mps'] = self.crawl_mps
        self.save_dig_settings()
        self.get_logger().warn(f'Saved for the next X: {self.describe_dig_settings()}.')

    def stop_excavation(self, reason):
        self.save_run_settings()

        # Arms stay where they are (re-target to the current angle if moving).
        if self.dig_phase == 'lowering':
            for arm in ('front', 'rear'):
                angle = self.arm_angle_deg(arm)
                if angle is not None:
                    self.goto_pub[arm].publish(Float64(data=angle))
        self.release_arm_inputs()
        self.set_arm_speed(self.p['arm_speed_deg_s'])
        self.cmd_pub.publish(Twist())
        self.wheels_stopped_at = None
        self.set_dig_phase('stopping')
        self.get_logger().warn(f'Excavation mode ending ({reason}): stopping wheels, then drums.')

    def excavation_tick(self):
        phase = self.dig_phase
        now = time.monotonic()
        elapsed = now - self.dig_phase_start

        if phase == 'spinup' and elapsed >= self.p['excavate_spinup_s']:
            for arm, target in self.dig_target.items():
                self.goto_pub[arm].publish(Float64(data=float(target)))
            self.set_dig_phase('lowering')
            self.get_logger().info(
                f'Excavation mode: lowering arms to front {self.dig_target["front"]:.2f}, '
                f'rear {self.dig_target["rear"]:.2f} deg.')

        elif phase == 'lowering':
            tol = self.p['excavate_arm_tolerance_deg']
            angles = {a: self.arm_angle_deg(a) for a in self.dig_target}
            if all(angles[a] is not None and abs(angles[a] - t) <= tol
                   for a, t in self.dig_target.items()):
                self.set_arm_speed(self.p['excavate_arm_speed_deg_s'])
                self.set_dig_phase('digging')
                self.get_logger().warn(
                    f'Excavation mode: digging, crawl {self.crawl_mps:.2f} m/s. '
                    f'D-pad L/R drums +/-{self.p["excavate_drum_rpm_step"]:g} RPM, '
                    f'U/D crawl +/-{self.p["excavate_crawl_step_mps"]:g} m/s, '
                    f'arm taps {self.p["excavate_arm_step_deg"]:g} deg.')
            elif elapsed > self.p['excavate_lower_timeout_s']:
                self.stop_excavation(
                    f'arms not at the dig angle after {elapsed:.0f} s')

        elif phase == 'stopping':
            if self.wheels_stopped_at is None:
                odom, t = self.odom
                stopped = (odom is not None and now - t <= 0.5 and
                           abs(odom.twist.twist.linear.x) < STOPPED_MPS and
                           abs(odom.twist.twist.angular.z) < STOPPED_RAD_S)
                if stopped or elapsed >= self.p['excavate_stop_wait_s']:
                    self.wheels_stopped_at = now
            elif now - self.wheels_stopped_at >= self.p['excavate_stop_delay_s']:
                self.drum_rpm = 0.0
                self.publish_drums()
                self.set_dig_phase('off')
                self.get_logger().warn('Excavation mode off: drums stopped, arms holding.')

    # -----------------------------------------------------
    # Mapping
    # -----------------------------------------------------

    def handle_speed_bounds(self, msg):
        dpad_y = self.axis(msg, self.p['dpad_y_axis'])

        # Excavation mode: D-pad up/down sets the crawl speed instead.
        if self.dig_phase != 'off':
            step = self.p['excavate_crawl_step_mps']
            change = 0.0
            if self.rising('dpad_up', dpad_y > 0.5):
                change = step
            if self.rising('dpad_down', dpad_y < -0.5):
                change = -step
            if change:
                self.crawl_mps = clamp(round(self.crawl_mps + change, 6),
                                       0.0, self.p['excavate_crawl_max_mps'])
                self.get_logger().info(f'Excavation crawl: {self.crawl_mps:.2f} m/s')
            return

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
        limit = self.drum_max_rpm()
        step = self.p['excavate_drum_rpm_step' if self.dig_phase != 'off' else 'drum_rpm_step']
        changed = False

        # D-pad right: + drum_rpm_step
        if self.rising('dpad_right', dpad_x < -0.5):
            self.drum_rpm = clamp(round(self.drum_rpm + step, 6), -limit, limit)
            changed = True

        # D-pad left: - drum_rpm_step
        if self.rising('dpad_left', dpad_x > 0.5):
            self.drum_rpm = clamp(round(self.drum_rpm - step, 6), -limit, limit)
            changed = True

        # Drums are continuous: publish only on change.
        if changed:
            self.publish_drums()
            self.get_logger().info(
                f'Drum speed: {self.drum_rpm:.2f} RPM '
                f'({abs(self.drum_rpm) * self.steps_s_per_drum_rpm():.0f} steps/s)')

    def steps_s_per_drum_rpm(self):
        # 1 drum RPM = gear_ratio motor revs per minute
        return self.p['drum_motor_steps_per_rev'] * self.p['drum_gear_ratio'] / 60.0

    def drum_max_rpm(self):
        return self.p['drum_max_steps_s'] / self.steps_s_per_drum_rpm()

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

        step = direction * self.p[
            'excavate_arm_step_deg' if self.dig_phase != 'off' else 'arm_step_deg']
        self.step_pub[arm.name].publish(Float64(data=step))
        self.get_logger().info(f'{label} arm: step {step:+.2f} deg.')

    def handle_drive(self, msg):
        # Excavation mode: still until digging, then fixed crawl with a small
        # steering trim from the sticks (right - left). Published here, from
        # /joy, so the crawl stops if the controller goes quiet (cmd_mux timeout).
        if self.dig_phase != 'off':
            cmd = Twist()
            if self.dig_phase == 'digging':
                trim = (self.axis(msg, self.p['right_y_axis']) -
                        self.axis(msg, self.p['left_y_axis'])) / 2.0
                cmd.linear.x = self.crawl_mps
                cmd.angular.z = trim * self.p['excavate_steer_max_rad_s']
            self.cmd_pub.publish(cmd)
            return

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
        # Drum RPM -> drum rad/s for ros2_control. The hardware plugin converts
        # back to motor steps/s with drum_steps_per_rev (motor steps x gear ratio).
        rad_s = self.drum_rpm * 2.0 * math.pi / 60.0

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
