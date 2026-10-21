"""Generate one-parameter-at-a-time sweep missions for ipex_tools/missions/.

    python3 scripts/generate_sweeps.py src/ipex_tools/missions

Edit the values below and re-run instead of hand-editing the sweep_*.yaml files.
"""
import os
import sys

OUT = sys.argv[1]
GAP = 3      # s idle between steps (clean 2 s baseline for the next step)


def fmt(v):
    return f'{v:g}'


def step(prim, tag, **params):
    p = ', '.join(f'{k}: {fmt(v) if isinstance(v, float) else v}' for k, v in params.items())
    return f'  - {{primitive: {prim}, tag: {tag}, params: {{{p}}}}}'


def gap():
    return f'  - {{primitive: idle, tag: gap, params: {{t_wait_s: {GAP}}}}}'


ALL = []     # (primitive title, sections) for sweep_all


def write(name, header, description, sections, collect=True):
    if collect:
        ALL.append((name, sections))
    lines = [f'# {"=" * 76}', *[f'# {h}' if h else '#' for h in header], f'# {"=" * 76}', '',
             'save_as:                # output folder name, e.g. arc_sweep_lab_oct21',
             '                        # (empty = <date_time>_<name>; _2, _3 added if it exists)', '',
             f'name: {name}', 'description: >', f'  {description}',
             'record: true', 'on_failure: stop', '', 'steps:',
             step('idle', 'baseline_start', t_wait_s=10)]
    for title, body in sections:
        lines += ['', f'  # --- {title} ' + '-' * max(3, 70 - len(title))]
        lines += body
    lines += ['', step('idle', 'baseline_end', t_wait_s=10), '']
    with open(os.path.join(OUT, f'{name}.yaml'), 'w', newline='\n') as f:
        f.write('\n'.join(lines))
    print(name, sum(1 for l in lines if l.startswith('  - ')), 'steps')


def run_header(name, what, prep, minutes):
    return [f'Sweep: {what}', '',
            f'  roxey mission {name} --check     validate only (nothing moves)',
            f'  roxey mission {name}             run (records a bag)', '',
            'One parameter changes at a time; the others stay at their defaults.',
            f'A {GAP} s idle follows every step so the next step\'s idle baseline',
            '(the 2 s before it starts) is clean.', '',
            f'Before running: {prep}', 'Someone on the controller (B = all stop).',
            f'About {minutes} minutes.']


HOMED = 'arms homed (A or `roxey home`).'

# --- set_arm ---------------------------------------------------------------
angles = [40, 32, 24, 16, 8, 0, -8, -16, -24, -32]          # limits -40..+45
down = []
for a in angles:
    down += [step('set_arm', f'arm_{a}', arm='both', theta_deg=a),
             step('idle', f'hold_{a}', t_wait_s=5)]
back = [step('set_arm', 'arm_return_full', arm='both', theta_deg=45), gap()]
write('sweep_set_arm',
      run_header('sweep_set_arm', 'shoulder angle, both arms, +40 to -32 deg in 8 deg steps.',
                 HOMED + ' Area under the drums clear.', 2) +
      ['', 'Each 8 deg move down is followed by a 5 s hold (holding power at that',
       'angle). One full-range move back up (-32 to +45 deg) gives the upward',
       'direction for comparison. Per-arm energy: group_energy_j in report.yaml.'],
      'Shoulder angle sweep (10 angles) with 5 s holds, then one full move up.',
      [('move down 8 deg at a time, hold 5 s', down), ('return to home in one move', back)])

# --- drums (excavate / deposit with arms up, no driving) ---------------------
speeds = [250 * i for i in range(1, 11)]                      # cap 3000
dig, dump = [], []
for s in speeds:
    dig += [step('excavate', f'dig_drum_{s}', arm='both', dig_angle_deg=45, speed_mps=0,
                 drum_steps_s=s, duration_s=5), gap()]
for s in (500, 1250, 2000):
    dump += [step('deposit', f'dump_drum_{s}', arm='both', dump_angle_deg=45, speed_mps=0,
                  drum_steps_s=s, duration_s=5), gap()]
write('sweep_drums',
      run_header('sweep_drums', 'drum speed, all four drums (10 speeds dig direction, 3 dump direction).',
                 HOMED + ' Drums clear of the ground and of people.', 3) +
      ['', 'Arms stay at +45 deg (home) and the rover does not move, so these are',
       'free-spin (no-load) numbers. Duration is not swept: power is constant',
       'at a given speed, so energy just scales with time. Dump direction has',
       '3 points to check it matches the dig direction. Dig angle and crawl',
       'speed need soil: run those in the sandbox. Drums have no speed',
       'feedback, so a stalled drum is NOT detected; watch and listen.'],
      'Drum speed sweep, drums in the air.',
      [('drum speed, dig direction (250..2500 steps/s)', dig),
       ('drum speed, dump direction (3 points)', dump)])

# --- drive_straight -----------------------------------------------------------
spd, dist = [], []
for i in range(1, 11):
    v = round(0.08 * i, 2)                                    # cap 1.0
    spd += [step('drive_straight', f'fwd_v{fmt(v)}', distance_m=1.0, speed_mps=v), gap(),
            step('drive_straight', f'back_v{fmt(v)}', distance_m=-1.0, speed_mps=v), gap()]
for d in (0.5, 1.0, 2.0):
    dist += [step('drive_straight', f'fwd_d{fmt(d)}', distance_m=d, speed_mps=0.2), gap(),
             step('drive_straight', f'back_d{fmt(d)}', distance_m=-d, speed_mps=0.2), gap()]
write('sweep_drive_straight',
      run_header('sweep_drive_straight', 'drive speed (10 values, 0.08..0.80 m/s) and distance (0.5, 1, 2 m).',
                 'arms homed and up, about 2.5 m of clear straight floor ahead.', 4) +
      ['', 'Every forward run is followed by the same run backward, so the rover',
       'ends near where it started; the back runs double as reverse data.',
       'Distance only needs 3 points: at a fixed speed energy grows linearly',
       'with distance, and 3 points give the slope plus the start/stop',
       'overhead. Distance is open-loop: drift accumulates.'],
      'Drive speed and distance sweeps, forward and back.',
      [('speed (1 m each way)', spd), ('distance (0.2 m/s each way)', dist)])

# --- point_turn ---------------------------------------------------------------
rate, ang = [], []
for i in range(1, 11):
    w = 5 * i                                                 # deg/s, cap 57.3
    rate += [step('point_turn', f'left_w{w}', dpsi_deg=90, omega_deg_s=w), gap(),
             step('point_turn', f'right_w{w}', dpsi_deg=-90, omega_deg_s=w), gap()]
for a in (45, 180, 360):                                     # deg
    ang += [step('point_turn', f'left_a{a}', dpsi_deg=a, omega_deg_s=17.2), gap(),
            step('point_turn', f'right_a{a}', dpsi_deg=-a, omega_deg_s=17.2), gap()]
write('sweep_point_turn',
      run_header('sweep_point_turn', 'turn rate (10 values, 5..50 deg/s) and turn angle (45, 180, 360 deg).',
                 'arms homed and up, about 1.5 x 1.5 m clear around the rover.', 5) +
      ['', 'Every left turn is followed by the same turn right, so the heading',
       'returns; the right turns double as mirror data. Angle only needs 3',
       'points (energy is linear in angle at a fixed rate). Point turns can',
       'STALL on high-grip floors (7 A limit).'],
      'Point turn rate and angle sweeps, left then right.',
      [('turn rate (90 deg each way)', rate), ('turn angle (17.2 deg/s)', ang)])

# --- arc_turn -----------------------------------------------------------------
rad, aspd, aang = [], [], []
for i in range(10):
    r = round(0.3 + 0.2 * i, 1)                               # r >= speed (0.2)
    rad += [step('arc_turn', f'fwd_r{fmt(r)}', r_arc_m=r, speed_mps=0.2, dpsi_deg=45), gap(),
            step('arc_turn', f'back_r{fmt(r)}', r_arc_m=r, speed_mps=-0.2, dpsi_deg=-45), gap()]
for i in range(1, 11):
    v = round(0.08 * i, 2)                                    # turn rate v/1.0 <= 0.8 rad/s
    aspd += [step('arc_turn', f'fwd_v{fmt(v)}', r_arc_m=1.0, speed_mps=v, dpsi_deg=45), gap(),
             step('arc_turn', f'back_v{fmt(v)}', r_arc_m=1.0, speed_mps=-v, dpsi_deg=-45), gap()]
for a in (15, 45, 90):
    aang += [step('arc_turn', f'fwd_a{a}', r_arc_m=1.0, speed_mps=0.2, dpsi_deg=a), gap(),
             step('arc_turn', f'back_a{a}', r_arc_m=1.0, speed_mps=-0.2, dpsi_deg=-a), gap()]
write('sweep_arc_turn',
      run_header('sweep_arc_turn', 'arc radius (10 values, 0.3..2.1 m), speed (10 values, 0.08..0.80 m/s), turn angle (15, 45, 90 deg).',
                 'arms homed and up, about 2.5 x 2.5 m clear ahead and to the left.', 6) +
      ['', 'Every forward arc is retraced in reverse (negative speed and angle),',
       'so the rover ends near where it started. Defaults while sweeping one',
       'parameter: radius 1.0 m, speed 0.2 m/s, angle 45 deg.'],
      'Arc turn radius, speed and angle sweeps, each arc retraced in reverse.',
      [('radius (0.2 m/s, 45 deg)', rad), ('speed (1.0 m radius, 45 deg)', aspd),
       ('turn angle (1.0 m radius, 0.2 m/s)', aang)])

# --- everything, in one mission ------------------------------------------------
write('sweep_all',
      run_header('sweep_all', 'every sweep above in one run: arms, drums, drive, point turns, arcs.',
                 HOMED + ' Drums clear; about 2.5 x 2.5 m clear floor.', 18) +
      ['', 'Same steps and tags as sweep_set_arm, sweep_drums, sweep_drive_straight,',
       'sweep_point_turn and sweep_arc_turn, run back to back. Run those',
       'individually to repeat one sweep. Battery: ~30 min of mostly light load.'],
      'All parameter sweeps (arms, drums, drive, point turns, arcs) in one run.',
      [(f'{name}: {title}', body) for name, sections in ALL for title, body in sections],
      collect=False)
