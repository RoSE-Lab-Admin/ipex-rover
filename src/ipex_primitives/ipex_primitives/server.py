#!/usr/bin/env python3
"""Primitive server: runs Roxey motion primitives behind one action.

Action:   /ipex/primitive   (ipex_interfaces/action/Primitive)
          One primitive at a time; a second goal is rejected (BUSY).

For every primitive it handles: parameter validation, preconditions, timing,
energy (t, E, E above idle from /ipex/energy), feedback, cancel, all stop
(/ipex/all_stop), operator override (/ipex/teleop_override), timeouts,
stopping the drive on exit, and appending the result to the run log
(<data_dir>/runs/primitive_runs.csv).

Hardware paths used:
  drive  -> /cmd_vel/primitive (through cmd_mux)
  arms   -> /ipex/<arm>_arm/goto (absolute degrees)
  drums  -> /<arm>_drum_controller/commands (rad/s)
  inputs <- /diff_drive_controller/odom, /joint_states, /ipex/<arm>_arm/homed,
            /rover/<wheel>/current (ACS712; the RoboClaw current reading is unreliable), /ipex/energy
"""

import csv
import datetime
import math
import os
import threading
import time
import traceback

import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float32, Float64, Float64MultiArray, String

from ipex_interfaces.action import Primitive as PrimitiveAction
from ipex_interfaces.msg import EnergyState

from ipex_primitives.base import Codes, PrimitiveFailure
from ipex_primitives.energy_window import EnergyHistory, Sample, window_energy
from ipex_primitives.library import CATALOG
from ipex_primitives.params import ParamError, parse_params_yaml, resolve_params


CONFIG_DEFAULTS = {
    'arm_min_deg': -40.0,
    'arm_max_deg': 45.0,
    'arm_speed_deg_s': 5.625,       # default arm speed (firmware default)
    'arm_speed_min_deg_s': 0.5,     # firmware clamps ARM_SPEED to [min, max]
    'arm_speed_max_deg_s': 5.625,
    'arm_pivot_height_m': 0.3132,   # pivot above ground, loaded (CAD 0.3208 - 7.6 mm sag, fit to rover)
    'arm_length_m': 0.3786,         # shoulder pivot to drum axle (ipex.xacro)
    'drum_cut_radius_m': 0.149,     # drum tooth-tip radius (mesh)
    'arm_tolerance_deg': 0.5,
    'arm_timeout_margin_s': 5.0,
    'max_speed_mps': 1.0,
    'max_turn_rad_s': 1.0,
    'drive_timeout_factor': 1.5,
    'drive_timeout_margin_s': 3.0,
    'stall_current_a': 6.5,
    'stall_time_s': 1.0,
    'drum_dig_sign_front': 1.0,
    'drum_dig_sign_rear': -1.0,
    'drum_max_rad_s': 3200.0 * 2.0 * math.pi / (400.0 * 80.0),   # 3200 steps/s = 6.0 drum RPM
    'drum_spinup_s': 1.0,
    'idle_window_s': 2.0,
    'energy_settle_s': 0.3,
    'data_dir': '~/roxey_data',
}

WHEELS = ('front_left', 'front_right', 'back_left', 'back_right')
SHOULDER_JOINTS = {'front': 'front_shoulder_rev', 'rear': 'back_shoulder_rev'}
FRESH_S = 0.5

LOG_FIELDS = ['time', 'name', 'params', 'tag', 'success', 'failure_code', 'message',
              'duration_s', 'energy_j', 'energy_above_idle_j', 'idle_power_w',
              'drive_j', 'rear_arm_j', 'front_arm_j', 'outputs']


def yaml_inline(data):
    if not data:
        return '{}'
    return yaml.safe_dump(data, default_flow_style=True, sort_keys=False).strip()


class PrimitiveContext:
    """Hardware access + safety checks given to Primitive.run()."""

    def __init__(self, server, goal_handle):
        self.s = server
        self.goal_handle = goal_handle
        self.start = time.monotonic()
        self.deadline = None
        self.stall_since = {}
        self.last_feedback = 0.0
        self.e0 = None
        self.arms_commanded = set()     # arms this primitive sent a goto to

    # -------------------------------------------------------------- control

    def check(self):
        if self.goal_handle.is_cancel_requested:
            raise PrimitiveFailure(Codes.CANCELED, 'canceled by client')
        if self.s.estop.is_set():
            raise PrimitiveFailure(Codes.ESTOP, 'all stop')
        if self.s.override.is_set():
            raise PrimitiveFailure(Codes.OPERATOR_OVERRIDE, 'operator used the hand controller')
        if self.deadline is not None and time.monotonic() > self.deadline:
            raise PrimitiveFailure(Codes.TIMEOUT, 'did not finish within the time limit')

    def sleep(self, seconds):
        end = time.monotonic() + seconds
        while True:
            self.check()
            remaining = end - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(0.02, remaining))

    def set_timeout(self, seconds):
        self.deadline = time.monotonic() + seconds

    def feedback(self, progress, status):
        now = time.monotonic()
        if now - self.last_feedback < 0.2:
            return
        self.last_feedback = now
        fb = PrimitiveAction.Feedback()
        fb.elapsed_s = now - self.start
        latest = self.s.energy.latest()
        fb.energy_j = (latest.total_j - self.e0.total_j) if (latest and self.e0) else math.nan
        fb.progress = float(progress)
        fb.status = status
        self.goal_handle.publish_feedback(fb)

    # ---------------------------------------------------------------- drive

    def drive(self, v, w):
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.s.drive_cmd = twist

    def stop_drive(self):
        self.s.drive_cmd = None
        self.s.drive_pub.publish(Twist())

    def pose(self):
        odom, t = self.s.odom
        if odom is None or time.monotonic() - t > FRESH_S:
            return None
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (odom.pose.pose.position.x, odom.pose.pose.position.y, yaw)

    def check_stall(self):
        now = time.monotonic()
        limit = self.s.cfg['stall_current_a']
        for wheel in WHEELS:
            value, t = self.s.drive_current.get(wheel, (math.nan, -1e9))
            if math.isfinite(value) and now - t <= FRESH_S and abs(value) >= limit:
                self.stall_since.setdefault(wheel, now)
                if now - self.stall_since[wheel] >= self.s.cfg['stall_time_s']:
                    raise PrimitiveFailure(
                        Codes.STALL, f'{wheel} at {abs(value):.1f} A for '
                        f'{self.s.cfg["stall_time_s"]:.1f} s (stall threshold {limit} A)')
            else:
                self.stall_since.pop(wheel, None)

    # ----------------------------------------------------------------- arms

    def arm_homed(self, arm):
        return self.s.homed.get(arm, False)

    def arm_angle_deg(self, arm):
        value, t = self.s.arm_angle.get(arm, (None, -1e9))
        if value is None or time.monotonic() - t > 1.0:
            return None
        return value

    def arm_goto(self, arm, deg):
        self.arms_commanded.add(arm)
        self.s.goto_pub[arm].publish(Float64(data=float(deg)))

    def halt_arms(self):
        """Stop arms this primitive moved: re-target them to where they are now.

        The arm Teensy finishes a goto on its own, so ending the primitive
        (cancel, override, timeout, stall) does not stop a move by itself.
        """
        for arm in self.arms_commanded:
            angle = self.arm_angle_deg(arm)
            if angle is not None:
                self.s.goto_pub[arm].publish(Float64(data=float(angle)))

    def arm_speed(self, arm, deg_s):
        """Arm speed for the next moves (persists on the Teensy until changed)."""
        self.s.speed_pub[arm].publish(Float64(data=float(deg_s)))

    def drums(self, arm, rad_s):
        self.s.drum_pub[arm].publish(Float64MultiArray(data=[float(rad_s), float(rad_s)]))


class PrimitiveServer(Node):

    def __init__(self):
        super().__init__('primitive_server')

        for key, default in CONFIG_DEFAULTS.items():
            self.declare_parameter(key, default)
        self.cfg = {k: self.get_parameter(k).value for k in CONFIG_DEFAULTS}

        self.group = ReentrantCallbackGroup()
        self.busy = threading.Lock()
        self.estop = threading.Event()
        self.override = threading.Event()

        self.energy = EnergyHistory()
        self.odom = (None, -1e9)
        self.arm_angle = {}
        self.homed = {}
        self.drive_current = {}
        self.drive_cmd = None

        g = self.group
        self.create_subscription(EnergyState, '/ipex/energy', self.on_energy, 10, callback_group=g)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom',
            lambda m: setattr(self, 'odom', (m, time.monotonic())), 10, callback_group=g)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10, callback_group=g)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for arm in ('front', 'rear'):
            self.create_subscription(
                Bool, f'/ipex/{arm}_arm/homed',
                lambda m, a=arm: self.homed.__setitem__(a, m.data), latched, callback_group=g)
        for wheel in WHEELS:
            self.create_subscription(
                Float32, f'/rover/{wheel}/current',     # ACS712, not roboclaw_current
                lambda m, w=wheel: self.drive_current.__setitem__(w, (m.data, time.monotonic())),
                10, callback_group=g)
        self.create_subscription(Empty, '/ipex/all_stop', lambda _m: self.estop.set(), 10,
                                 callback_group=g)
        self.create_subscription(Empty, '/ipex/teleop_override', lambda _m: self.override.set(), 10,
                                 callback_group=g)

        self.drive_pub = self.create_publisher(Twist, '/cmd_vel/primitive', 10)
        self.goto_pub = {a: self.create_publisher(Float64, f'/ipex/{a}_arm/goto', 10)
                         for a in ('front', 'rear')}
        self.speed_pub = {a: self.create_publisher(Float64, f'/ipex/{a}_arm/speed', 10)
                          for a in ('front', 'rear')}
        self.drum_pub = {a: self.create_publisher(Float64MultiArray, f'/{a}_drum_controller/commands', 10)
                         for a in ('front', 'rear')}
        # Marks which primitive is running (empty = none) so a bag can be split by step.
        self.active_pub = self.create_publisher(String, '/ipex/primitive/active', latched)
        self.active_pub.publish(String(data=''))

        self.create_timer(0.05, self.publish_drive, callback_group=g)

        self.action = ActionServer(
            self, PrimitiveAction, '/ipex/primitive',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=g)

        self.log_path = os.path.join(os.path.expanduser(self.cfg['data_dir']),
                                     'runs', 'primitive_runs.csv')
        self.get_logger().info(
            f'Primitive server ready on /ipex/primitive: {", ".join(CATALOG)}. '
            f'Run log: {self.log_path}')

    # ------------------------------------------------------------ callbacks

    def on_energy(self, msg):
        groups = {g: e for g, e in zip(msg.groups, msg.group_energy_j) if g != 'total'}
        total_j = dict(zip(msg.groups, msg.group_energy_j)).get('total', sum(groups.values()))
        total_w = dict(zip(msg.groups, msg.group_power_w)).get('total', math.nan)
        self.energy.add(Sample(time.monotonic(), total_j, groups, total_w))

    def on_joints(self, msg):
        now = time.monotonic()
        positions = dict(zip(msg.name, msg.position))
        for arm, joint in SHOULDER_JOINTS.items():
            if joint in positions and math.isfinite(positions[joint]):
                self.arm_angle[arm] = (math.degrees(positions[joint]), now)

    def publish_drive(self):
        if self.drive_cmd is not None:
            self.drive_pub.publish(self.drive_cmd)

    def on_goal(self, _goal):
        if self.busy.locked():
            self.get_logger().warn('Rejected goal: another primitive is running (BUSY).')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ------------------------------------------------------------- execute

    def execute(self, goal_handle):
        with self.busy:
            return self._execute(goal_handle)

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = PrimitiveAction.Result()
        outputs, notes = {}, []
        code, message = '', 'done'
        prim, ctx = None, None

        self.estop.clear()
        self.override.clear()
        t0 = time.monotonic()
        idle_w = self.energy.idle_power(t0, self.cfg['idle_window_s'])
        e0 = self.energy.latest()

        try:
            cls = CATALOG.get(goal.name)
            if cls is None:
                raise PrimitiveFailure(
                    Codes.INVALID_PARAMS,
                    f"unknown primitive '{goal.name}'. Available: {', '.join(CATALOG)}")
            try:
                params, notes = resolve_params(cls.PARAMS, parse_params_yaml(goal.params_yaml))
            except (ParamError, yaml.YAMLError) as e:
                raise PrimitiveFailure(Codes.INVALID_PARAMS, str(e)) from None
            if not cls.CAPABLE:
                raise PrimitiveFailure(Codes.UNSUPPORTED, f'{cls.NAME}: {cls.SUMMARY}')

            self.get_logger().info(f'Starting {goal.name} {yaml_inline(params)}')
            self.active_pub.publish(String(data=yaml_inline(
                {'name': goal.name, 'tag': goal.tag, 'params': params})))
            prim = cls(params, self.cfg)
            ctx = PrimitiveContext(self, goal_handle)
            ctx.e0 = e0
            outputs = prim.run(ctx) or {}
        except PrimitiveFailure as f:
            code, message = f.code, f.message
        except Exception as e:   # noqa: BLE001 - report any bug as INTERNAL
            code, message = Codes.INTERNAL, f'{type(e).__name__}: {e}'
            self.get_logger().error(traceback.format_exc())
        finally:
            if prim is not None and ctx is not None:
                try:
                    prim.cleanup(ctx)
                    # ALL STOP already stopped every Teensy; on any other early
                    # end, stop arm moves this primitive started.
                    if code and code != Codes.ESTOP:
                        ctx.halt_arms()
                except Exception:   # noqa: BLE001
                    self.get_logger().error('cleanup failed:\n' + traceback.format_exc())
            self.drive_cmd = None
            self.drive_pub.publish(Twist())
            self.active_pub.publish(String(data=''))

        duration = time.monotonic() - t0

        # Wait briefly for an energy sample taken after the primitive ended.
        settle_end = time.monotonic() + self.cfg['energy_settle_s']
        while time.monotonic() < settle_end:
            latest = self.energy.latest()
            if latest is not None and latest.t >= t0 + duration:
                break
            time.sleep(0.02)
        energy = window_energy(e0, self.energy.latest(), duration, idle_w)
        if e0 is None:
            notes.append('energy meter not running: E not measured')

        result.success = code == ''
        result.failure_code = code
        result.message = message if not notes else f'{message} ({"; ".join(notes)})'
        result.duration_s = duration
        result.energy_j = energy.energy_j
        result.energy_above_idle_j = energy.energy_above_idle_j
        result.idle_power_w = energy.idle_power_w
        result.groups = list(energy.groups.keys())
        result.group_energy_j = list(energy.groups.values())
        result.outputs_yaml = yaml_inline(outputs)

        if code == Codes.CANCELED and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.get_logger().info(
            f'{goal.name}: {"OK" if result.success else code} in {duration:.2f} s, '
            f'E = {energy.energy_j:.1f} J (above idle {energy.energy_above_idle_j:.1f} J) - {message}')
        self.log_run(goal, result, energy.groups)
        return result

    # ----------------------------------------------------------------- log

    def log_run(self, goal, result, groups):
        try:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            new = not os.path.exists(self.log_path)
            with open(self.log_path, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=LOG_FIELDS)
                if new:
                    writer.writeheader()
                writer.writerow({
                    'time': datetime.datetime.now().isoformat(timespec='seconds'),
                    'name': goal.name,
                    'params': goal.params_yaml,
                    'tag': goal.tag,
                    'success': result.success,
                    'failure_code': result.failure_code,
                    'message': result.message,
                    'duration_s': f'{result.duration_s:.3f}',
                    'energy_j': f'{result.energy_j:.2f}',
                    'energy_above_idle_j': f'{result.energy_above_idle_j:.2f}',
                    'idle_power_w': f'{result.idle_power_w:.2f}',
                    'drive_j': f'{groups.get("drive", math.nan):.2f}',
                    'rear_arm_j': f'{groups.get("rear_arm", math.nan):.2f}',
                    'front_arm_j': f'{groups.get("front_arm", math.nan):.2f}',
                    'outputs': result.outputs_yaml,
                })
        except OSError as e:
            self.get_logger().error(f'Could not write run log {self.log_path}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PrimitiveServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
