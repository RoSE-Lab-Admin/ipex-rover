#!/usr/bin/env python3
"""Energy meter: power and cumulative energy for every measured channel.

Subscribes to the per-channel voltage and current topics published by
ipex_hardware (std_msgs/Float32), computes power, integrates energy, and
publishes ipex_interfaces/EnergyState on /ipex/energy.

Energy used over any window = energy at the end - energy at the start.
Publish std_msgs/Empty on /ipex/energy/reset to restart the counters.
"""

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Float32

from ipex_interfaces.msg import EnergyState
from ipex_energy.energy_math import Channel, EnergyAccumulator


WHEELS = ('front_left', 'front_right', 'back_left', 'back_right')
ARMS = ('rear_arm', 'front_arm')
ARM_PARTS = ('drum', 'shoulder')


class EnergyMeter(Node):

    def __init__(self):
        super().__init__('energy_meter')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('stale_timeout_s', 0.5)
        # Arm sensors read negative when drawing power (sensor orientation).
        self.declare_parameter('arm_current_sign', -1.0)

        rate_hz = self.get_parameter('rate_hz').value
        arm_sign = self.get_parameter('arm_current_sign').value

        channels = []
        topics = {}

        # Drive motors: P = V_est * |I| (V_est = battery V x PWM duty).
        for wheel in WHEELS:
            name = f'drive/{wheel}'
            channels.append(Channel(name=name, group='drive', current_mode='abs'))
            topics[name] = (f'/rover/{wheel}/voltage', f'/rover/{wheel}/current')

        # Arm driver supplies: P = V * (sign * I).
        for arm in ARMS:
            for part in ARM_PARTS:
                name = f'{arm}/{part}'
                channels.append(Channel(
                    name=name, group=arm,
                    current_mode='signed', current_sign=arm_sign))
                topics[name] = (f'/rover/{arm}/{part}/voltage',
                                f'/rover/{arm}/{part}/current')

        self.acc = EnergyAccumulator(
            channels=channels,
            stale_timeout_s=self.get_parameter('stale_timeout_s').value)

        for name, (v_topic, i_topic) in topics.items():
            self.create_subscription(
                Float32, v_topic,
                lambda msg, n=name: self.acc.set_voltage(n, msg.data, time.monotonic()),
                10)
            self.create_subscription(
                Float32, i_topic,
                lambda msg, n=name: self.acc.set_current(n, msg.data, time.monotonic()),
                10)

        self.create_subscription(Empty, '/ipex/energy/reset', self.reset_callback, 10)
        self.pub = self.create_publisher(EnergyState, '/ipex/energy', 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self._was_stale = {c.name: None for c in channels}

        self.get_logger().info(
            f'Energy meter running: {len(channels)} channels at {rate_hz:.0f} Hz '
            f'-> /ipex/energy')

    def reset_callback(self, _msg):
        self.acc.reset(time.monotonic())
        self.get_logger().info('Energy counters reset.')

    def tick(self):
        now = time.monotonic()
        self.acc.step(now)
        self.report_stale_changes()

        msg = EnergyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.elapsed_s = self.acc.elapsed(now)

        for ch in self.acc.channels:
            msg.channels.append(ch.name)
            msg.power_w.append(ch.power_w)
            msg.energy_j.append(ch.energy_j)
            msg.stale.append(ch.stale)

        for group, (power, energy) in self.acc.group_totals().items():
            msg.groups.append(group)
            msg.group_power_w.append(power)
            msg.group_energy_j.append(energy)

        self.pub.publish(msg)

    def report_stale_changes(self):
        for ch in self.acc.channels:
            previous = self._was_stale[ch.name]
            if previous is not None and previous != ch.stale:
                if ch.stale:
                    self.get_logger().warn(
                        f'{ch.name}: sensor data missing or stale, counting 0 W.')
                else:
                    self.get_logger().info(f'{ch.name}: sensor data back.')
            self._was_stale[ch.name] = ch.stale


def main(args=None):
    rclpy.init(args=args)
    node = EnergyMeter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
