#!/usr/bin/env python3
"""roxey: command-line interface for operating Roxey.

    roxey energy                live power / energy table (Ctrl+C to quit)
    roxey energy --once         print one snapshot and exit
    roxey energy --reset        restart the energy counters

Coming next (motion primitives):
    roxey run <primitive> [--param value ...] [--record]
    roxey mission <file.yaml> [--record]
    roxey runs

ROS_DOMAIN_ID defaults to 42 (the rover's) if it is not already set.
"""

import argparse
import os
import sys
import time

DEFAULT_DOMAIN_ID = '42'


def _init_ros():
    if 'ROS_DOMAIN_ID' not in os.environ:
        os.environ['ROS_DOMAIN_ID'] = DEFAULT_DOMAIN_ID

    import rclpy
    rclpy.init()
    return rclpy


# ----------------------------------------------------------------------------
# energy
# ----------------------------------------------------------------------------

def _format_energy(msg):
    lines = []
    lines.append(f'Roxey energy  (counting for {msg.elapsed_s:8.1f} s)')
    lines.append('')
    lines.append(f'{"group":<12} {"power W":>10} {"energy J":>12} {"energy Wh":>10}')
    lines.append('-' * 47)
    for group, power, energy in zip(msg.groups, msg.group_power_w, msg.group_energy_j):
        lines.append(f'{group:<12} {power:10.2f} {energy:12.1f} {energy / 3600.0:10.4f}')
    lines.append('')
    lines.append(f'{"channel":<22} {"power W":>10} {"energy J":>12}  ')
    lines.append('-' * 47)
    for name, power, energy, stale in zip(
            msg.channels, msg.power_w, msg.energy_j, msg.stale):
        flag = '  STALE' if stale else ''
        lines.append(f'{name:<22} {power:10.2f} {energy:12.1f}{flag}')
    return '\n'.join(lines)


def cmd_energy(args):
    rclpy = _init_ros()
    from rclpy.node import Node
    from std_msgs.msg import Empty
    from ipex_interfaces.msg import EnergyState

    node = Node('roxey_energy')
    latest = {'msg': None}
    node.create_subscription(
        EnergyState, '/ipex/energy', lambda m: latest.__setitem__('msg', m), 10)

    try:
        if args.reset:
            pub = node.create_publisher(Empty, '/ipex/energy/reset', 10)
            deadline = time.monotonic() + 2.0
            while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            pub.publish(Empty())
            rclpy.spin_once(node, timeout_sec=0.2)
            print('Energy counters reset.')
            return 0

        deadline = time.monotonic() + 3.0
        while latest['msg'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if latest['msg'] is None:
            print('No data on /ipex/energy. Is the rover running? '
                  '(sudo systemctl status ipex-rover)', file=sys.stderr)
            return 1

        if args.once:
            print(_format_energy(latest['msg']))
            return 0

        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest['msg'] is not None:
                print('\033[2J\033[H' + _format_energy(latest['msg']), flush=True)
                latest['msg'] = None
            time.sleep(0.4)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ----------------------------------------------------------------------------
# placeholders for step 2
# ----------------------------------------------------------------------------

def cmd_not_yet(args):
    print(f"'roxey {args.command}' is not implemented yet "
          '(coming with the motion primitives).', file=sys.stderr)
    return 2


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------

def build_parser():
    parser = argparse.ArgumentParser(
        prog='roxey', description='Operate Roxey from the terminal.')
    sub = parser.add_subparsers(dest='command', required=True)

    p_energy = sub.add_parser('energy', help='live power and energy')
    p_energy.add_argument('--once', action='store_true', help='print one snapshot and exit')
    p_energy.add_argument('--reset', action='store_true', help='restart the energy counters')
    p_energy.set_defaults(func=cmd_energy)

    for name, help_text in (('run', 'run a motion primitive'),
                            ('mission', 'run a mission file'),
                            ('runs', 'show recent primitive results')):
        p = sub.add_parser(name, help=f'{help_text} (coming soon)')
        p.set_defaults(func=cmd_not_yet)

    return parser


def main(argv=None):
    args, _unknown = build_parser().parse_known_args(argv)
    sys.exit(args.func(args))


if __name__ == '__main__':
    main()
