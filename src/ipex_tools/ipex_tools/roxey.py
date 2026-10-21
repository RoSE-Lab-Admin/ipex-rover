#!/usr/bin/env python3
"""roxey: operate Roxey from the terminal.

  roxey prims [NAME]                         list primitives / show one's parameters
  roxey run NAME [key=value ...] [--record]  run one primitive, print t and E
  roxey mission FILE|EXAMPLE [--record]      run a mission YAML (validated first)
  roxey mission FILE --check                 validate only, nothing moves
  roxey runs [-n 20]                         recent primitive results (run log)
  roxey record NAME [--duration S]           record a bag (Ctrl+C to stop)
  roxey energy [--once | --reset]            live power / energy
  roxey home                                 home (calibrate) both arms  -- ARMS MOVE
  roxey stop                                 all stop

Angles accept degrees: theta_deg=-10 instead of theta_rad=-0.1745.
Data goes to ~/roxey_data (override with ROXEY_DATA_DIR).
ROS_DOMAIN_ID defaults to 42 (the rover's) if not set.
"""

import argparse
import collections
import csv
import datetime
import math
import os
import sys
import time

import yaml

DEFAULT_DOMAIN_ID = '42'

# A mission always ends on these, whatever its on_failure setting.
ALWAYS_STOP = ('ESTOP', 'OPERATOR_OVERRIDE', 'CANCELED')


# ============================================================================
# helpers
# ============================================================================

def _init_ros():
    if 'ROS_DOMAIN_ID' not in os.environ:
        os.environ['ROS_DOMAIN_ID'] = DEFAULT_DOMAIN_ID
    import rclpy
    rclpy.init()
    return rclpy


def _shutdown(rclpy, node=None):
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _fmt(x, digits=1):
    return '   n/a' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.{digits}f}'


def _print_result(r):
    status = 'OK' if r.success else f'FAILED ({r.failure_code})'
    print(f'\n{r.name}: {status}')
    print(f'  time            {_fmt(r.duration_s, 2)} s')
    print(f'  energy          {_fmt(r.energy_j)} J   ({_fmt(r.energy_j / 3600 if r.energy_j == r.energy_j else math.nan, 4)} Wh)')
    avg_w = r.energy_j / r.duration_s if r.duration_s and r.duration_s > 0 else math.nan
    print(f'  average power   {_fmt(avg_w)} W')
    print(f'  above idle      {_fmt(r.energy_above_idle_j)} J   (idle {_fmt(r.idle_power_w)} W)')
    for group, energy in r.groups.items():
        print(f'    {group:<13} {_fmt(energy)} J')
    if r.outputs:
        print(f'  outputs         {yaml.safe_dump(r.outputs, default_flow_style=True).strip()}')
    if r.message:
        print(f'  message         {r.message}')


def _progress_printer(name, window_s=1.0):
    history = collections.deque()      # (elapsed_s, energy_j) over the last window_s

    def show(fb):
        pct = f'{fb.progress * 100:5.1f}%' if fb.progress >= 0 else '   ...'
        power = math.nan
        if math.isfinite(fb.energy_j):
            history.append((fb.elapsed_s, fb.energy_j))
            while len(history) > 2 and fb.elapsed_s - history[0][0] > window_s:
                history.popleft()
            (t0, e0), (t1, e1) = history[0], history[-1]
            if t1 - t0 > 0.1:
                power = (e1 - e0) / (t1 - t0)
        line = (f'\r[{name}] {pct}  {fb.elapsed_s:6.1f} s  {_fmt(power):>6} W  '
                f'{_fmt(fb.energy_j)} J  {fb.status}')
        sys.stdout.write(line[:110].ljust(110))
        sys.stdout.flush()
    return show


def _pub_once(rclpy, node, msg_type, topic):
    pub = node.create_publisher(msg_type, topic, 10)
    deadline = time.monotonic() + 2.0
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(msg_type())
    rclpy.spin_once(node, timeout_sec=0.2)
    return pub.get_subscription_count()


# ============================================================================
# prims
# ============================================================================

def cmd_prims(args):
    from ipex_primitives.library import CATALOG
    from ipex_primitives.params import REQUIRED

    if args.name:
        cls = CATALOG.get(args.name)
        if cls is None:
            print(f"Unknown primitive '{args.name}'. Available: {', '.join(CATALOG)}", file=sys.stderr)
            return 1
        print(f'{cls.NAME}{"" if cls.CAPABLE else "   (NOT CAPABLE YET)"}')
        print(f'  {cls.SUMMARY}')
        print(f'  stops when: {cls.STOPS_WHEN}')
        print(f'  fails when: {cls.FAILS_WHEN}\n')
        print(f'  {"parameter":<20} {"unit":<7} {"default":<12} description')
        for p in cls.PARAMS:
            if p.unsupported:
                default = 'unsupported'
            elif p.default is REQUIRED:
                default = 'REQUIRED'
            else:
                default = str(p.default if not isinstance(p.default, float) else round(p.default, 4))
            extra = []
            if p.choices:
                extra.append('one of ' + '/'.join(p.choices))
            aliases = p.all_aliases()
            if aliases:
                extra.append('also: ' + ', '.join(aliases))
            desc = p.help + (f" [{'; '.join(extra)}]" if extra else '')
            print(f'  {p.name:<20} {p.unit:<7} {default:<12} {desc}')
        print(f'\n  example: roxey run {cls.NAME} ' + ' '.join(
            f'{p.name}=...' for p in cls.PARAMS if p.default is REQUIRED and not p.unsupported))
        return 0

    print(f'{"primitive":<16} {"capable":<8} summary')
    for name, cls in CATALOG.items():
        print(f'{name:<16} {"yes" if cls.CAPABLE else "no":<8} {cls.SUMMARY}')
    print('\nDetails: roxey prims NAME')
    return 0


# ============================================================================
# run
# ============================================================================

def cmd_run(args):
    from ipex_primitives.params import ParamError, parse_key_values
    from ipex_tools.client import RoxeyClient, RoxeyError
    from ipex_tools.recorder import BagRecorder, new_folder

    try:
        params = parse_key_values(args.params)
    except ParamError as e:
        print(f'roxey run: {e}', file=sys.stderr)
        return 2

    rclpy = _init_ros()
    client = RoxeyClient()
    recorder = None
    try:
        if args.record or args.all:
            folder = new_folder('recordings', f'run_{args.name}')
            recorder = BagRecorder(folder, notes=args.notes, all_topics=args.all,
                                   meta={'command': 'roxey run ' + ' '.join(sys.argv[2:])}).start()
            print(f'Recording to {folder}')

        r = client.run(args.name, tag=args.tag, on_feedback=_progress_printer(args.name), **params)
        _print_result(r)
        if recorder:
            recorder.stop(extra={'result': r.as_row()})
            print(f'  bag             {recorder.bag_dir}')
        return 0 if r.success else 1
    except RoxeyError as e:
        print(f'\nroxey run: {e}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\nInterrupted.')
        return 130
    finally:
        if recorder and recorder.proc and recorder.proc.poll() is None:
            recorder.stop()
        client.close()
        _shutdown(rclpy)


# ============================================================================
# mission
# ============================================================================

def cmd_mission(args):
    from ipex_tools.missions import (
        MissionError, check_save_as, find_mission, installed_examples, load_mission,
        validate_mission)

    try:
        path = find_mission(args.file)
        mission = load_mission(path)
        if args.save_as:
            check_save_as(args.save_as)
            mission['save_as'] = args.save_as
    except MissionError as e:
        print(f'roxey mission: {e}', file=sys.stderr)
        examples = installed_examples()
        if examples:
            print(f"Installed examples: {', '.join(examples)}", file=sys.stderr)
        return 2

    problems = validate_mission(mission)
    print(f"Mission '{mission['name']}': {len(mission['steps'])} steps ({path})")
    if mission['description']:
        print(f"  {mission['description']}")
    for step in mission['steps']:
        print(f"  {step['index']:>2}. {step['name']:<15} "
              f"{yaml.safe_dump(step['params'], default_flow_style=True).strip()}")
    if problems:
        print('\nNot valid:', file=sys.stderr)
        for p in problems:
            print(f'  - {p}', file=sys.stderr)
        return 2
    print('Valid.')
    if args.check:
        return 0

    from ipex_tools.client import RoxeyClient, RoxeyError
    from ipex_tools.recorder import BagRecorder, new_folder

    if mission['save_as']:
        folder = new_folder('missions', mission['save_as'], exact=True)
    else:
        folder = new_folder('missions', mission['name'])
    print(f'Saving to {folder}')
    record = args.record or args.all or mission['record']
    rclpy = _init_ros()
    client = RoxeyClient()
    recorder = None
    results = []
    started = datetime.datetime.now().isoformat(timespec='seconds')
    t_start = time.monotonic()
    try:
        if record:
            recorder = BagRecorder(folder, notes=args.notes, all_topics=args.all,
                                   meta={'mission': mission['name'], 'mission_file': path}).start()
        for step in mission['steps']:
            print(f"\n--- step {step['index']}/{len(mission['steps'])}: {step['name']}")
            r = client.run(step['name'], tag=f"{mission['name']}/{step['tag']}",
                           on_feedback=_progress_printer(step['name']), **step['params'])
            r.tag = step['tag']
            results.append(r)
            _print_result(r)
            if not r.success and r.failure_code in ALWAYS_STOP:
                print(f'\nStopping mission ({r.failure_code}).')
                break
            if not r.success and mission['on_failure'] == 'stop':
                print('\nStopping mission (on_failure: stop).')
                break
    except RoxeyError as e:
        print(f'\nroxey mission: {e}', file=sys.stderr)
    except KeyboardInterrupt:
        print('\nMission interrupted.')
    finally:
        wall = time.monotonic() - t_start
        if recorder:
            recorder.stop()
        client.close()
        _shutdown(rclpy)

    summary = _mission_summary(mission, results, wall)
    _write_mission_report(folder, mission, path, results, summary, bool(recorder), started)
    print('\n=== Mission summary')
    print(f"  steps completed  {summary['steps_succeeded']}/{summary['steps_total']}")
    print(f"  time (steps)     {_fmt(summary['time_s'], 2)} s   (wall {_fmt(wall, 1)} s)")
    print(f"  energy           {_fmt(summary['energy_j'])} J   ({_fmt(summary['energy_j'] / 3600, 4)} Wh)")
    print(f"  above idle       {_fmt(summary['energy_above_idle_j'])} J")
    print(f'  report           {folder}')
    return 0 if summary['steps_succeeded'] == summary['steps_total'] else 1


def _nansum(values):
    vals = [v for v in values if isinstance(v, float) and not math.isnan(v)]
    return sum(vals) if vals else math.nan


def _mission_summary(mission, results, wall):
    return {
        'steps_total': len(mission['steps']),
        'steps_run': len(results),
        'steps_succeeded': sum(r.success for r in results),
        'time_s': _nansum([r.duration_s for r in results]),
        'wall_time_s': wall,
        'energy_j': _nansum([r.energy_j for r in results]),
        'energy_above_idle_j': _nansum([r.energy_above_idle_j for r in results]),
    }


def _write_mission_report(folder, mission, path, results, summary, recorded, started):
    rows = [r.as_row() for r in results]
    with open(os.path.join(folder, 'report.yaml'), 'w') as f:
        yaml.safe_dump({'mission': mission['name'], 'file': path, 'started': started,
                        'recorded_bag': recorded,
                        'summary': summary, 'steps': rows}, f, sort_keys=False)
    if rows:
        fields = []
        for row in rows:
            fields += [k for k in row if k not in fields]
        with open(os.path.join(folder, 'steps.csv'), 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)


# ============================================================================
# runs
# ============================================================================

def cmd_runs(args):
    from ipex_tools.recorder import DATA_DIR
    path = os.path.join(DATA_DIR, 'runs', 'primitive_runs.csv')
    if not os.path.isfile(path):
        print(f'No runs logged yet ({path}).')
        return 0
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))[-args.n:]
    print(f'{"time":<20} {"primitive":<15} {"result":<18} {"t s":>8} {"E J":>9} {"E>idle J":>9}  tag')
    for row in rows:
        result = 'OK' if row['success'] == 'True' else row['failure_code']
        print(f"{row['time']:<20} {row['name']:<15} {result:<18} {row['duration_s']:>8} "
              f"{row['energy_j']:>9} {row['energy_above_idle_j']:>9}  {row['tag']}")
    print(f'\n{path}')
    return 0


# ============================================================================
# record
# ============================================================================

def cmd_record(args):
    from ipex_tools.recorder import BagRecorder, new_folder
    folder = new_folder('recordings', args.name)
    os.environ.setdefault('ROS_DOMAIN_ID', DEFAULT_DOMAIN_ID)
    recorder = BagRecorder(folder, notes=args.notes, all_topics=args.all,
                           meta={'command': 'roxey record'})
    try:
        recorder.start()
    except RuntimeError as e:
        print(f'roxey record: {e}', file=sys.stderr)
        return 1
    print(f'Recording to {folder}  ({"for " + str(args.duration) + " s" if args.duration else "Ctrl+C to stop"})')
    try:
        if args.duration:
            time.sleep(args.duration)
        else:
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    recorder.stop()
    print(f'Saved {recorder.bag_dir}')
    return 0


# ============================================================================
# energy
# ============================================================================

def _format_energy(msg):
    lines = [f'Roxey energy  (counting for {msg.elapsed_s:8.1f} s)', '',
             f'{"group":<12} {"power W":>10} {"energy J":>12} {"energy Wh":>10}', '-' * 47]
    for group, power, energy in zip(msg.groups, msg.group_power_w, msg.group_energy_j):
        lines.append(f'{group:<12} {power:10.2f} {energy:12.1f} {energy / 3600.0:10.4f}')
    lines += ['', f'{"channel":<22} {"power W":>10} {"energy J":>12}', '-' * 47]
    for name, power, energy, stale in zip(msg.channels, msg.power_w, msg.energy_j, msg.stale):
        lines.append(f'{name:<22} {power:10.2f} {energy:12.1f}{"  STALE" if stale else ""}')
    return '\n'.join(lines)


def cmd_energy(args):
    rclpy = _init_ros()
    from rclpy.node import Node
    from std_msgs.msg import Empty
    from ipex_interfaces.msg import EnergyState

    node = Node('roxey_energy')
    latest = {'msg': None}
    node.create_subscription(EnergyState, '/ipex/energy', lambda m: latest.__setitem__('msg', m), 10)
    try:
        if args.reset:
            _pub_once(rclpy, node, Empty, '/ipex/energy/reset')
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
        _shutdown(rclpy, node)


# ============================================================================
# home / stop
# ============================================================================

def cmd_topic(topic, done_text):
    def run(_args):
        rclpy = _init_ros()
        from rclpy.node import Node
        from std_msgs.msg import Empty
        node = Node('roxey_cmd')
        try:
            n = _pub_once(rclpy, node, Empty, topic)
            print(done_text if n else f'Sent, but nothing is listening on {topic}. Is the rover running?')
            return 0 if n else 1
        finally:
            _shutdown(rclpy, node)
    return run


# ============================================================================
# main
# ============================================================================

def build_parser():
    parser = argparse.ArgumentParser(
        prog='roxey', description='Operate Roxey from the terminal.',
        epilog='Example: roxey run set_arm arm=front theta_deg=-10 --record')
    sub = parser.add_subparsers(dest='command', required=True)

    p = sub.add_parser('prims', help='list motion primitives / show parameters')
    p.add_argument('name', nargs='?')
    p.set_defaults(func=cmd_prims)

    p = sub.add_parser('run', help='run one motion primitive')
    p.add_argument('name', help='primitive name (roxey prims)')
    p.add_argument('params', nargs='*', help='key=value parameters, e.g. theta_deg=-10')
    p.add_argument('--tag', default='', help='label stored in the run log')
    p.add_argument('--record', action='store_true', help='record a bag for this run')
    p.add_argument('--all', action='store_true', help='record every topic (implies --record)')
    p.add_argument('--notes', default='', help='notes saved with the recording')
    p.set_defaults(func=cmd_run)

    p = sub.add_parser('mission', help='run a mission YAML file')
    p.add_argument('file', help='path, or an installed example name')
    p.add_argument('--check', action='store_true', help='validate only, nothing moves')
    p.add_argument('--record', action='store_true', help='record a bag for the whole mission')
    p.add_argument('--all', action='store_true', help='record every topic (implies --record)')
    p.add_argument('--save-as', default='', metavar='NAME',
                   help='output folder name (overrides save_as in the file)')
    p.add_argument('--notes', default='', help='notes saved with the recording')
    p.set_defaults(func=cmd_mission)

    p = sub.add_parser('runs', help='recent primitive results')
    p.add_argument('-n', type=int, default=20, help='how many (default 20)')
    p.set_defaults(func=cmd_runs)

    p = sub.add_parser('record', help='record a bag (Ctrl+C to stop)')
    p.add_argument('name', help='recording name, e.g. drum_sweep')
    p.add_argument('--duration', type=float, default=None, help='seconds (default: until Ctrl+C)')
    p.add_argument('--all', action='store_true', help='record every topic, not just the standard set')
    p.add_argument('--notes', default='', help='notes saved with the recording')
    p.set_defaults(func=cmd_record)

    p = sub.add_parser('energy', help='live power and energy')
    p.add_argument('--once', action='store_true', help='print one snapshot and exit')
    p.add_argument('--reset', action='store_true', help='restart the energy counters')
    p.set_defaults(func=cmd_energy)

    p = sub.add_parser('home', help='home (calibrate) both arms -- ARMS MOVE')
    p.set_defaults(func=cmd_topic('/ipex/home_arms', 'Homing arms.'))

    p = sub.add_parser('stop', help='all stop')
    p.set_defaults(func=cmd_topic('/ipex/all_stop', 'ALL STOP sent.'))

    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    sys.exit(args.func(args))


if __name__ == '__main__':
    main()
