#!/usr/bin/env python3
"""Drive command mux: one owner of the wheels at a time.

Priority (highest first):
  1. all stop   /ipex/all_stop received -> zero for estop_hold_s
  2. teleop     /cmd_vel/teleop, while the sticks are out of the deadband
  3. primitive  /cmd_vel/primitive, while it keeps publishing
  4. fallback   zero (also used when every source has gone stale)

Output: /cmd_vel at rate_hz (consumed by cmd_vel_bridge).
Active source: /ipex/cmd_mux/source (std_msgs/String, published on change).

Operator input during a primitive cancels the primitive; that is handled by
controller.py (/ipex/teleop_override) and the primitive server, not here.
"""

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String


class CmdMux(Node):

    def __init__(self):
        super().__init__('cmd_mux')

        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('teleop_timeout_s', 0.5)
        self.declare_parameter('primitive_timeout_s', 0.5)
        self.declare_parameter('teleop_deadband', 0.01)   # m/s and rad/s
        self.declare_parameter('estop_hold_s', 1.0)

        self.p = {n: self.get_parameter(n).value for n in (
            'rate_hz', 'teleop_timeout_s', 'primitive_timeout_s',
            'teleop_deadband', 'estop_hold_s')}

        self.teleop = (Twist(), -1e9)       # (msg, receive time)
        self.primitive = (Twist(), -1e9)
        self.estop_until = -1e9
        self.source = None

        self.create_subscription(
            Twist, '/cmd_vel/teleop',
            lambda m: setattr(self, 'teleop', (m, time.monotonic())), 10)
        self.create_subscription(
            Twist, '/cmd_vel/primitive',
            lambda m: setattr(self, 'primitive', (m, time.monotonic())), 10)
        self.create_subscription(Empty, '/ipex/all_stop', self.all_stop, 10)

        self.out_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.source_pub = self.create_publisher(String, '/ipex/cmd_mux/source', 10)

        self.create_timer(1.0 / self.p['rate_hz'], self.tick)
        self.get_logger().info(
            'cmd_mux: all_stop > teleop > primitive > zero  -> /cmd_vel')

    def all_stop(self, _msg):
        self.estop_until = time.monotonic() + self.p['estop_hold_s']

    def _active(self, twist):
        db = self.p['teleop_deadband']
        return abs(twist.linear.x) > db or abs(twist.angular.z) > db

    def tick(self):
        now = time.monotonic()
        teleop_msg, teleop_t = self.teleop
        prim_msg, prim_t = self.primitive
        teleop_fresh = now - teleop_t <= self.p['teleop_timeout_s']
        prim_fresh = now - prim_t <= self.p['primitive_timeout_s']

        if now < self.estop_until:
            out, source = Twist(), 'all_stop'
        elif teleop_fresh and self._active(teleop_msg):
            out, source = teleop_msg, 'teleop'
        elif prim_fresh:
            out, source = prim_msg, 'primitive'
        else:
            out, source = Twist(), 'none'

        self.out_pub.publish(out)

        if source != self.source:
            self.source = source
            self.source_pub.publish(String(data=source))
            self.get_logger().info(f'drive source: {source}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdMux()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
