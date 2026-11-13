#!/usr/bin/env python3
"""roxey: operate Roxey from the terminal.

  roxey prims [NAME]                         list primitives / show one's parameters
  roxey run NAME [key=value ...] [--record]  run one primitive, print t and E
  roxey mission FILE|EXAMPLE [--record]      run a mission YAML (validated first)
  roxey mission FILE --check                 validate only, nothing moves
  roxey runs [-n 20]                         recent primitive results (run log)
  roxey record NAME [--duration S]           record a bag (Ctrl+C to stop)
  roxey energy [--once | --reset]            live power / energy
  roxey sensors [--window 5]                 live V / I / P of every sensor (sanity checks)
  roxey cal show | drive | arms [--yes]      current sensor zeros: show / re-zero (saved on Teensy)
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
    from rclpy.signals import SignalHandlerOptions
    # Keep Python's own Ctrl+C (KeyboardInterrupt): rclpy's handler shuts the
    # context down first, so the cancel request could never reach the rover.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
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
# sensors (live raw V / I / P per channel, for sanity checks)
# ============================================================================

SENSOR_CHANNELS = (
    [(f'drive/{w}', f'/rover/{w}') for w in ('front_left', 'front_right', 'back_left', 'back_right')] +
    [(f'{a}/{p}', f'/rover/{a}/{p}') for a in ('rear_arm', 'front_arm') for p in ('drum', 'shoulder')]
)
SENSOR_STALE_S = 0.5


class _Stat:
    """Latest value, rolling-window mean, and min / max since start."""

    def __init__(self, window_s):
        self.window_s = window_s
        self.reset()

    def reset(self):
        self.value, self.t = math.nan, -math.inf
        self.recent = []            # (t, value) inside the window
        self.lo, self.hi = math.inf, -math.inf

    def add(self, value, now):
        self.value, self.t = value, now
        self.recent.append((now, value))
        while self.recent and now - self.recent[0][0] > self.window_s:
            self.recent.pop(0)
        self.lo, self.hi = min(self.lo, value), max(self.hi, value)

    def mean(self):
        return sum(v for _, v in self.recent) / len(self.recent) if self.recent else math.nan


def _format_sensors(stats, now, window_s, started):
    def f(x, w, d):
        return f'{x:{w}.{d}f}' if math.isfinite(x) else f'{"-":>{w}}'

    lines = [f'Roxey sensors  (min/max over {now - started:.0f} s, mean over last {window_s:g} s)'
             '   Ctrl+C to quit', '',
             f'{"channel":<20} {"V":>7} {"I now":>8} {"I mean":>8} {"I min":>8} {"I max":>8}'
             f' {"P W":>8} {"speed":>8}',
             '-' * 82]
    for name, _ in SENSOR_CHANNELS:
        v, i, spd = stats[name]['v'], stats[name]['i'], stats[name].get('speed')
        stale = now - v.t > SENSOR_STALE_S or now - i.t > SENSOR_STALE_S
        p = v.value * i.value if not stale else math.nan
        lines.append(
            f'{name:<20} {f(v.value, 7, 2)} {f(i.value, 8, 3)} {f(i.mean(), 8, 3)} '
            f'{f(i.lo, 8, 3)} {f(i.hi, 8, 3)} {f(p, 8, 2)} '
            f'{f(spd.value, 8, 3) if spd else "":>8}{"  STALE" if stale else ""}')
        if name == 'drive/back_right':
            lines.append('')
    lines += ['', 'Drive V = battery x duty (0 when stopped, so an I offset gives 0 W at rest).',
              'Drive speed is the wheel speed from the encoders. Arm V = supply voltage.']
    return '\n'.join(lines)


def cmd_sensors(args):
    rclpy = _init_ros()
    from rclpy.node import Node
    from std_msgs.msg import Float32

    node = Node('roxey_sensors')
    stats = {}
    for name, prefix in SENSOR_CHANNELS:
        stats[name] = {'v': _Stat(args.window), 'i': _Stat(args.window)}
        topics = [('v', f'{prefix}/voltage'), ('i', f'{prefix}/current')]
        if name.startswith('drive/'):
            stats[name]['speed'] = _Stat(args.window)
            topics.append(('speed', f'{prefix}/speed'))
        for key, topic in topics:
            node.create_subscription(
                Float32, topic,
                lambda m, s=stats[name][key]: s.add(float(m.data), time.monotonic()), 10)

    started = time.monotonic()
    next_print = 0.0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_print:
                print('\033[2J\033[H' + _format_sensors(stats, now, args.window, started), flush=True)
                next_print = now + args.period
    except KeyboardInterrupt:
        return 0
    finally:
        _shutdown(rclpy, node)


# ============================================================================
# cal (current sensor zeros on the drivetrain Teensy)
# ============================================================================
# Path: roxey -> /ipex/drivetrain/command (String) -> IpexSystem -> Teensy;
# Teensy "CAL_..." lines -> /ipex/drivetrain/reply (String). Zeros are saved in
# the Teensy EEPROM (no reflash). Firmware channel orders:
CAL_DRIVE_ORDER = ('back_left', 'back_right', 'front_right', 'front_left')
CAL_ARM_ORDER = ('rear_arm/drum', 'rear_arm/shoulder', 'front_arm/drum', 'front_arm/shoulder')

CAL_KINDS = {
    'drive': {
        'command': 'CAL_CURRENT', 'show_key': 'DRIVE',
        'channels': [(w, f'/rover/{w}/current') for w in CAL_DRIVE_ORDER],
        'steps': [
            'Rover on flat ground or on the stand, wheels still (the Teensy also stops the',
            'drive motors during the ~2 s measurement).',
            'Not sure a reading is offset or real holding current? Unplug that motor\'s lead',
            '(motor side of its sensor) and watch roxey sensors first.',
        ],
    },
    'arms': {
        'command': 'CAL_ARM_CURRENT', 'show_key': 'ARM',
        'channels': [(c, f'/rover/{c}/current') for c in CAL_ARM_ORDER],
        'steps': [
            'Rest or support BOTH arms first: unplugging the shoulder drivers drops holding torque.',
            'Unplug all 4 arm driver supply leads (both drums AND both shoulders), downstream',
            'of the current sensors, so no current flows through them. Sensors stay powered.',
            'Afterwards: plug them back in, then home the arms (roxey home) if an arm moved.',
        ],
    },
}


class _CalLink:
    """Send CAL_* commands and collect the Teensy's replies; watch currents and wheel speed."""

    def __init__(self, rclpy, node, channels):
        from std_msgs.msg import Float32, String
        self.rclpy, self.node, self.String = rclpy, node, String
        self.replies = []
        node.create_subscription(String, '/ipex/drivetrain/reply',
                                 lambda m: self.replies.append(m.data), 10)
        self.pub = node.create_publisher(String, '/ipex/drivetrain/command', 10)
        self.current = {name: _Stat(1e9) for name, _ in channels}
        for name, topic in channels:
            node.create_subscription(
                Float32, topic, lambda m, s=self.current[name]: s.add(float(m.data), time.monotonic()), 10)
        self.speed = {w: _Stat(1e9) for w in CAL_DRIVE_ORDER}
        for w in CAL_DRIVE_ORDER:
            node.create_subscription(
                Float32, f'/rover/{w}/speed',
                lambda m, s=self.speed[w]: s.add(float(m.data), time.monotonic()), 10)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def send(self, command, expect_prefixes, timeout=8.0):
        """Publish `command`; return the reply lines starting with any of expect_prefixes."""
        deadline = time.monotonic() + 2.0
        while self.pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.pub.get_subscription_count() == 0:
            raise RuntimeError('nothing listens on /ipex/drivetrain/command '
                               '(rover not running, or ipex_hardware not rebuilt)')
        self.replies.clear()
        self.pub.publish(self.String(data=command))
        deadline = time.monotonic() + timeout
        found = []
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            found = [r for r in self.replies if r.startswith(expect_prefixes)]
            if any(f.startswith(command + ' OK') or ' REJECTED' in f for f in found) or \
               (command == 'CAL_SHOW' and len(found) >= 2):
                return found
        return found

    def show(self):
        """{'DRIVE': [z0..z3], 'ARM': [z0..z3]} in sensor-side volts."""
        zeros = {}
        for line in self.send('CAL_SHOW', ('CAL_SHOW',), timeout=3.0):
            parts = line.split()
            if len(parts) == 6:
                zeros[parts[1]] = [float(x) for x in parts[2:]]
        return zeros

    def mean_currents(self, seconds):
        for s in self.current.values():
            s.reset()
        self.spin(seconds)
        return {name: (sum(v for _, v in s.recent) / len(s.recent) if s.recent else math.nan)
                for name, s in self.current.items()}


def _print_zeros(title, names, zeros):
    print(title)
    for name, z in zip(names, zeros):
        print(f'  {name:<20} {z:.4f} V')


def cmd_cal(args):
    rclpy = _init_ros()
    from rclpy.node import Node
    node = Node('roxey_cal')
    try:
        link = _CalLink(rclpy, node, CAL_KINDS['drive']['channels'] + CAL_KINDS['arms']['channels'])
        if args.what == 'show':
            zeros = link.show()
            if not zeros:
                print('No CAL_SHOW reply from the drivetrain Teensy.', file=sys.stderr)
                return 1
            if 'DRIVE' in zeros:
                _print_zeros('Drive current zeros (sensor V):', CAL_DRIVE_ORDER, zeros['DRIVE'])
            if 'ARM' in zeros:
                _print_zeros('Arm current zeros (sensor V):', CAL_ARM_ORDER, zeros['ARM'])
            print('Nominal 2.50 V. 1 A = 66 mV on the 30 A sensors (drive, drums), 185 mV on 5 A (shoulders).')
            return 0

        kind = CAL_KINDS[args.what]
        names = [n for n, _ in kind['channels']]
        print(f'Zero the {args.what} current sensors ({kind["command"]}, saved on the Teensy):')
        for step in kind['steps']:
            print(f'  - {step}')

        link.spin(1.5)
        moving = [w for w, s in link.speed.items() if math.isfinite(s.value) and abs(s.value) > 0.05]
        if moving:
            print(f'Refusing: wheels are turning ({", ".join(moving)}). Stop the rover first.',
                  file=sys.stderr)
            return 1

        if not args.yes:
            answer = input('Ready? Type y to calibrate: ').strip().lower()
            if answer != 'y':
                print('Cancelled, nothing changed.')
                return 0

        old = link.show().get(kind['show_key'])
        before = link.mean_currents(2.0)
        print(f'Sending {kind["command"]} ...')
        replies = link.send(kind['command'], (kind['command'],))
        result = next((r for r in replies if r.startswith(kind['command'] + ' OK')
                       or ' REJECTED' in r), None)
        if result is None:
            print(f'No answer from the Teensy within 8 s. Replies: {replies or "none"}', file=sys.stderr)
            return 1
        if ' REJECTED' in result:
            print(f'REJECTED, nothing saved: {result}', file=sys.stderr)
            return 1
        new = [float(x) for x in result.split()[-4:]]

        link.spin(1.0)
        after = link.mean_currents(3.0)
        print(f'\nSaved. {"channel":<20} {"old zero V":>10} {"new zero V":>10} '
              f'{"I before A":>11} {"I after A":>10}')
        for i, name in enumerate(names):
            o = f'{old[i]:10.4f}' if old else f'{"?":>10}'
            print(f'       {name:<20} {o} {new[i]:10.4f} {before[name]:11.3f} {after[name]:10.3f}')
        print('\nI after should be within about +/-0.05 A. Check over time with: roxey sensors')
        if args.what == 'arms':
            print('Now plug the arm driver supply leads back in (then roxey home if an arm moved).')
        return 0
    except RuntimeError as e:
        print(f'roxey cal: {e}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 1
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

    p = sub.add_parser('sensors', help='live V / I / P of every drive and arm sensor')
    p.add_argument('--window', type=float, default=5.0, help='seconds for the I mean (default 5)')
    p.add_argument('--period', type=float, default=0.5, help='screen refresh, s (default 0.5)')
    p.set_defaults(func=cmd_sensors)

    p = sub.add_parser('cal', help='current sensor zeros: show, or re-zero drive / arms')
    p.add_argument('what', choices=('show', 'drive', 'arms'),
                   help='show = print zeros in use; drive = CAL_CURRENT; arms = CAL_ARM_CURRENT')
    p.add_argument('--yes', action='store_true', help='skip the "ready?" question')
    p.set_defaults(func=cmd_cal)

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
