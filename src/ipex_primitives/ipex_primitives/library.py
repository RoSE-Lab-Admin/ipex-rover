"""Roxey motion primitives (ROS-free logic; hardware access goes through ctx).

Names and parameters follow the primitive spec (drive_straight, point_turn,
arc_turn, set_arm, excavate, deposit, scan_terrain, recharge, idle).
Every primitive returns t and E via the server; run() returns extra outputs.

Context (ctx) API used here (implemented by server.PrimitiveContext):
  ctx.check()                     raise if canceled / all stop / override / timeout
  ctx.sleep(s)                    sleep while checking
  ctx.feedback(progress, status)  progress 0..1 (or -1)
  ctx.set_timeout(s)              time limit from now (TIMEOUT when exceeded)
  ctx.drive(v_mps, w_rad_s)       continuous drive command (via cmd_mux)
  ctx.stop_drive()
  ctx.pose()                      (x, y, yaw) from odometry, or None
  ctx.check_stall()               raise STALL if drive current sits at the limit
  ctx.arm_homed(arm) / ctx.arm_angle_deg(arm) / ctx.arm_goto(arm, deg)
  ctx.drums(arm, rad_s)           drum speed (both drums on that arm)
"""

import math

from ipex_primitives.base import Codes, Primitive, PrimitiveFailure
from ipex_primitives.params import Param


DEG = math.pi / 180.0
ARM_CHOICES = ('front', 'rear', 'both')
# Drum speeds are at the DRUM (after the 80:1 gearbox). drum_steps_s is the
# motor step rate the firmware takes: 400 steps/motor rev x 80 = 32000 steps/drum rev.
DRUM_STEPS_PER_REV = 400.0 * 80.0
DRUM_STEPS_TO_RAD_S = 2.0 * math.pi / DRUM_STEPS_PER_REV
DRUM_RPM_TO_RAD_S = 2.0 * math.pi / 60.0
DEFAULT_DRUM_RAD_S = 2000.0 * DRUM_STEPS_TO_RAD_S       # 2000 steps/s = 3.75 drum RPM


def arms_of(arm):
    return ['front', 'rear'] if arm == 'both' else [arm]


# ----------------------------------------------------------------------------
# cut depth (flat, level ground)
# ----------------------------------------------------------------------------
# Drum axle height above ground = pivot height + arm length * sin(theta), with
# theta the shoulder angle from horizontal (+45 deg = home). The drum cuts
# drum_cut_radius below its axle. Geometry from ipex.xacro; the cut radius is
# the drum tooth-tip radius (mesh); the pivot height is the CAD value lowered 7.6 mm to fit a
# loaded-arm rover measurement (-35 deg -> 53 mm, 2026-10-21): tire compression / sinkage.

def cut_depth_m(theta_deg, config):
    """Depth of the drum's cut below the surface (negative = clearance above it)."""
    axle = config['arm_pivot_height_m'] + config['arm_length_m'] * math.sin(math.radians(theta_deg))
    return config['drum_cut_radius_m'] - axle


def angle_for_depth_deg(depth_m, config):
    """Shoulder angle that cuts depth_m below the surface (NaN if unreachable)."""
    s = (config['drum_cut_radius_m'] - depth_m - config['arm_pivot_height_m']) / config['arm_length_m']
    return math.degrees(math.asin(s)) if -1.0 <= s <= 1.0 else math.nan


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


# ----------------------------------------------------------------------------
# shared motion helpers
# ----------------------------------------------------------------------------

def move_arms(ctx, config, arms, target_deg, progress_span=(0.0, 1.0), speed_deg_s=None):
    """Send arms to an absolute angle and wait until within tolerance.

    speed_deg_s: arm speed for this move (default: config arm_speed_deg_s).
    The caller restores the default afterwards (see SetArm.cleanup).
    """
    lo, hi = config['arm_min_deg'], config['arm_max_deg']
    if not lo <= target_deg <= hi:
        raise PrimitiveFailure(
            Codes.INVALID_PARAMS,
            f'arm angle {target_deg:.1f} deg outside limits [{lo}, {hi}]')

    for arm in arms:
        if not ctx.arm_homed(arm):
            raise PrimitiveFailure(
                Codes.PRECONDITION, f'{arm} arm is not homed (press A or roxey home)')

    start = {arm: ctx.arm_angle_deg(arm) for arm in arms}
    if any(a is None for a in start.values()):
        raise PrimitiveFailure(Codes.PRECONDITION, 'no arm angle feedback (/joint_states)')

    speed = config['arm_speed_deg_s'] if speed_deg_s is None else speed_deg_s
    travel = max(abs(target_deg - a) for a in start.values())
    ctx.set_timeout(travel / speed + config['arm_timeout_margin_s'])

    for arm in arms:
        if speed_deg_s is not None:
            ctx.arm_speed(arm, speed_deg_s)
        ctx.arm_goto(arm, target_deg)

    tol = config['arm_tolerance_deg']
    p0, p1 = progress_span
    while True:
        angles = {arm: ctx.arm_angle_deg(arm) for arm in arms}
        errors = [abs(target_deg - a) for a in angles.values() if a is not None]
        if len(errors) == len(arms) and max(errors) <= tol:
            return angles
        done = 1.0 - (max(errors) / travel if travel > 0 and errors else 0.0)
        ctx.feedback(p0 + (p1 - p0) * max(0.0, min(1.0, done)),
                     'moving arms: ' + ', '.join(f'{a} {v:.1f}' for a, v in angles.items()
                                                 if v is not None))
        ctx.sleep(0.1)


def drive_until_heading(ctx, config, v, w, dpsi):
    """Drive with (v, w) until the heading has changed by |dpsi| (odometry)."""
    pose = ctx.pose()
    if pose is None:
        raise PrimitiveFailure(Codes.PRECONDITION, 'no odometry (/diff_drive_controller/odom)')
    yaw_prev = pose[2]
    turned = 0.0
    target = abs(dpsi)

    ctx.set_timeout(target / max(abs(w), 1e-6) * config['drive_timeout_factor']
                    + config['drive_timeout_margin_s'])
    while turned < target:
        ctx.drive(v, w)
        ctx.check_stall()
        ctx.sleep(0.05)
        pose = ctx.pose()
        if pose is None:
            raise PrimitiveFailure(Codes.HARDWARE, 'odometry lost')
        turned += abs(wrap_pi(pose[2] - yaw_prev))
        yaw_prev = pose[2]
        ctx.feedback(min(turned / target, 1.0), f'turned {math.degrees(turned):.1f} deg')
    ctx.stop_drive()
    return turned


def spin_drums(ctx, config, arms, omega_rad_s):
    """omega > 0 = dig direction, < 0 = dump direction (per config signs)."""
    for arm in arms:
        ctx.drums(arm, omega_rad_s * config[f'drum_dig_sign_{arm}'])


def check_drum_speed(config, omega):
    limit = config['drum_max_rad_s']
    if abs(omega) > limit * (1 + 1e-9):
        raise PrimitiveFailure(
            Codes.INVALID_PARAMS,
            f'drum speed {omega / DRUM_RPM_TO_RAD_S:.2f} drum RPM '
            f'({omega / DRUM_STEPS_TO_RAD_S:.0f} steps/s) exceeds '
            f'{limit / DRUM_RPM_TO_RAD_S:.2f} drum RPM ({limit / DRUM_STEPS_TO_RAD_S:.0f} steps/s)')


# ----------------------------------------------------------------------------
# primitives
# ----------------------------------------------------------------------------

class DriveStraight(Primitive):
    NAME = 'drive_straight'
    SUMMARY = 'Drive a distance forward (+) or backward (-) at a set speed.'
    STOPS_WHEN = 'distance reached (odometry)'
    FAILS_WHEN = 'stall, timeout, odometry lost'
    PARAMS = [
        Param('distance_m', float, unit='m', help='distance; negative drives backward'),
        Param('speed_mps', float, default=0.2, unit='m/s', min=0.02, help='speed (magnitude)'),
    ]

    def run(self, ctx):
        d = self.params['distance_m']
        v = min(abs(self.params['speed_mps']), self.config['max_speed_mps'])
        if d == 0:
            return {'distance_m': 0.0}
        v = math.copysign(v, d)

        start = ctx.pose()
        if start is None:
            raise PrimitiveFailure(Codes.PRECONDITION, 'no odometry (/diff_drive_controller/odom)')

        ctx.set_timeout(abs(d) / abs(v) * self.config['drive_timeout_factor']
                        + self.config['drive_timeout_margin_s'])
        traveled = 0.0
        while traveled < abs(d):
            ctx.drive(v, 0.0)
            ctx.check_stall()
            ctx.sleep(0.05)
            pose = ctx.pose()
            if pose is None:
                raise PrimitiveFailure(Codes.HARDWARE, 'odometry lost')
            traveled = math.hypot(pose[0] - start[0], pose[1] - start[1])
            ctx.feedback(min(traveled / abs(d), 1.0), f'{traveled:.2f} / {abs(d):.2f} m')
        ctx.stop_drive()
        return {'distance_m': math.copysign(traveled, d),
                'odometry': 'wheel encoders (slip not seen)'}


class PointTurn(Primitive):
    NAME = 'point_turn'
    SUMMARY = 'Turn in place by a heading change (+ = counter-clockwise / left).'
    STOPS_WHEN = 'heading change reached (odometry)'
    FAILS_WHEN = 'stall, timeout, odometry lost'
    PARAMS = [
        Param('dpsi_rad', float, unit='rad', help='heading change; + turns left'),
        Param('omega_rad_s', float, default=0.3, unit='rad/s', min=0.02,
              help='turn rate (magnitude)'),
    ]

    def run(self, ctx):
        dpsi = self.params['dpsi_rad']
        w = math.copysign(min(abs(self.params['omega_rad_s']), self.config['max_turn_rad_s']), dpsi)
        turned = drive_until_heading(ctx, self.config, 0.0, w, dpsi) if dpsi else 0.0
        return {'turned_rad': math.copysign(turned, dpsi),
                'odometry': 'wheel encoders (slip not seen)'}


class ArcTurn(Primitive):
    NAME = 'arc_turn'
    SUMMARY = 'Drive along an arc of radius r until the heading has changed by dpsi.'
    STOPS_WHEN = 'heading change reached (odometry)'
    FAILS_WHEN = 'stall, timeout, odometry lost'
    PARAMS = [
        Param('r_arc_m', float, unit='m', min=0.05, help='turning radius'),
        Param('speed_mps', float, default=0.2, unit='m/s', help='forward speed; negative reverses'),
        Param('dpsi_rad', float, unit='rad', help='heading change; + turns left'),
    ]

    def run(self, ctx):
        dpsi = self.params['dpsi_rad']
        v = self.params['speed_mps']
        v = math.copysign(min(abs(v), self.config['max_speed_mps']), v)
        if v == 0 or dpsi == 0:
            raise PrimitiveFailure(Codes.INVALID_PARAMS, 'speed_mps and dpsi_rad must be non-zero')
        # Heading change follows the turn rate sign regardless of travel direction.
        w = math.copysign(abs(v) / self.params['r_arc_m'], dpsi)
        if abs(w) > self.config['max_turn_rad_s']:
            raise PrimitiveFailure(Codes.INVALID_PARAMS,
                                   f'turn rate {abs(w):.2f} rad/s too high; lower speed or raise radius')
        turned = drive_until_heading(ctx, self.config, v, w, dpsi)
        return {'turned_rad': math.copysign(turned, dpsi), 'turn_rate_rad_s': w,
                'odometry': 'wheel encoders (slip not seen)'}


class SetArm(Primitive):
    NAME = 'set_arm'
    SUMMARY = 'Move an arm (front, rear or both) to an absolute shoulder angle.'
    STOPS_WHEN = 'angle reached within tolerance (encoder)'
    FAILS_WHEN = 'not homed, out of limits, timeout'
    PARAMS = [
        Param('arm', str, default='front', choices=ARM_CHOICES),
        Param('theta_rad', float, unit='rad', help='shoulder angle (+45 deg = top / home)'),
        Param('omega_rad_s', float, default=None, unit='rad/s',
              min=0.5 * DEG * (1 - 1e-9), max=5.625 * DEG * (1 + 1e-9),   # deg->rad rounding
              help='shoulder speed, 0.5..5.625 deg/s (default 5.625, the firmware default)'),
    ]

    def run(self, ctx):
        target_deg = math.degrees(self.params['theta_rad'])
        omega = self.params['omega_rad_s']
        speed = None
        if omega is not None:
            lo, hi = self.config['arm_speed_min_deg_s'], self.config['arm_speed_max_deg_s']
            speed = min(max(math.degrees(omega), lo), hi)
        final = move_arms(ctx, self.config, arms_of(self.params['arm']), target_deg,
                          speed_deg_s=speed)
        out = {f'final_{arm}_deg': round(a, 2) for arm, a in final.items()}
        out.update({f'{arm}_cut_depth_m': round(cut_depth_m(a, self.config), 4) for arm, a in final.items()})
        out['arm_speed_deg_s'] = round(speed if speed is not None else self.config['arm_speed_deg_s'], 3)
        return out

    def cleanup(self, ctx):
        # Back to the default speed so later moves and controller jogs are normal.
        if self.params.get('omega_rad_s') is not None:
            for arm in arms_of(self.params['arm']):
                ctx.arm_speed(arm, self.config['arm_speed_deg_s'])


class Excavate(Primitive):
    NAME = 'excavate'
    SUMMARY = 'Lower arm(s) to a dig angle, spin drums in the dig direction, crawl forward.'
    STOPS_WHEN = 'duration elapsed (m_target needs load cells)'
    FAILS_WHEN = 'not homed, stall, timeout'
    PARAMS = [
        Param('arm', str, default='front', choices=ARM_CHOICES),
        Param('dig_angle_rad', float, default=-10 * DEG, unit='rad', help='shoulder angle while digging'),
        Param('omega_drum_rad_s', float, default=DEFAULT_DRUM_RAD_S, unit='rad/s',
              aliases={'drum_steps_s': DRUM_STEPS_TO_RAD_S, 'drum_rpm': DRUM_RPM_TO_RAD_S},
              help='drum speed at the drum; + = dig direction (also drum_rpm, drum_steps_s)'),
        Param('speed_mps', float, default=0.05, unit='m/s', min=0.0, help='crawl speed'),
        Param('duration_s', float, default=10.0, unit='s', min=0.0, help='dig time'),
        Param('d_dig_m', float, default=None, unit='m', min=0.0,
              help='cut depth below the surface; sets the dig angle (overrides dig_angle_rad). '
                   'Flat, level ground; max ~0.079 m at the -40 deg limit'),
        Param('m_target_kg', float, default=None, unit='kg', unsupported=True,
              help='needs arm load cells; use duration_s'),
    ]

    def run(self, ctx):
        self.reject_unsupported_params()
        arms = arms_of(self.params['arm'])
        omega = abs(self.params['omega_drum_rad_s'])
        check_drum_speed(self.config, omega)

        depth = self.params['d_dig_m']
        if depth is not None:
            angle = angle_for_depth_deg(depth, self.config)
            lo = self.config['arm_min_deg']
            if not angle >= lo:          # also catches NaN
                raise PrimitiveFailure(
                    Codes.INVALID_PARAMS,
                    f'd_dig_m {depth:.3f} m is deeper than the arm reaches '
                    f'({cut_depth_m(lo, self.config):.3f} m at {lo:g} deg)')
        else:
            angle = math.degrees(self.params['dig_angle_rad'])

        move_arms(ctx, self.config, arms, angle, (0.0, 0.2))
        spin_drums(ctx, self.config, arms, omega)
        ctx.sleep(self.config['drum_spinup_s'])

        duration = self.params['duration_s']
        v = min(self.params['speed_mps'], self.config['max_speed_mps'])
        ctx.set_timeout(duration + self.config['drive_timeout_margin_s'])
        elapsed = 0.0
        while elapsed < duration:
            ctx.drive(v, 0.0)
            ctx.check_stall()
            ctx.sleep(0.05)
            elapsed += 0.05
            ctx.feedback(0.2 + 0.8 * min(elapsed / duration, 1.0) if duration else 1.0,
                         f'digging {elapsed:.1f} / {duration:.1f} s')
        ctx.stop_drive()
        return {'dig_time_s': round(duration, 2), 'dig_angle_deg': round(angle, 2),
                'cut_depth_m': round(cut_depth_m(angle, self.config), 4),
                'note': 'cut depth estimated for flat ground; m_load not measured (no load cells)'}

    def cleanup(self, ctx):
        for arm in arms_of(self.params['arm']):
            ctx.drums(arm, 0.0)


class Deposit(Primitive):
    NAME = 'deposit'
    SUMMARY = 'Raise arm(s) to a dump angle and spin drums in the dump direction.'
    STOPS_WHEN = 'duration elapsed (m_deposit needs load cells)'
    FAILS_WHEN = 'not homed, stall, timeout'
    PARAMS = [
        Param('arm', str, default='front', choices=ARM_CHOICES),
        Param('dump_angle_rad', float, default=45 * DEG, unit='rad', help='shoulder angle while dumping'),
        Param('omega_drum_rad_s', float, default=DEFAULT_DRUM_RAD_S, unit='rad/s',
              aliases={'drum_steps_s': DRUM_STEPS_TO_RAD_S, 'drum_rpm': DRUM_RPM_TO_RAD_S},
              help='drum speed at the drum (magnitude; always dump direction; also drum_rpm, drum_steps_s)'),
        Param('speed_mps', float, default=0.0, unit='m/s', min=0.0, help='forward speed while dumping'),
        Param('duration_s', float, default=10.0, unit='s', min=0.0, help='dump time'),
        Param('h_drop_m', float, default=None, unit='m', unsupported=True,
              help='needs arm kinematics; use dump_angle_rad'),
        Param('m_deposit_kg', float, default=None, unit='kg', unsupported=True,
              help='needs arm load cells; use duration_s'),
    ]

    def run(self, ctx):
        self.reject_unsupported_params()
        arms = arms_of(self.params['arm'])
        omega = abs(self.params['omega_drum_rad_s'])
        check_drum_speed(self.config, omega)

        move_arms(ctx, self.config, arms, math.degrees(self.params['dump_angle_rad']), (0.0, 0.3))
        spin_drums(ctx, self.config, arms, -omega)

        duration = self.params['duration_s']
        v = min(self.params['speed_mps'], self.config['max_speed_mps'])
        ctx.set_timeout(duration + self.config['drum_spinup_s'] + self.config['drive_timeout_margin_s'])
        elapsed = 0.0
        while elapsed < duration:
            if v > 0:
                ctx.drive(v, 0.0)
                ctx.check_stall()
            ctx.sleep(0.05)
            elapsed += 0.05
            ctx.feedback(0.3 + 0.7 * min(elapsed / duration, 1.0) if duration else 1.0,
                         f'dumping {elapsed:.1f} / {duration:.1f} s')
        ctx.stop_drive()
        return {'dump_time_s': round(duration, 2), 'note': 'm_deposit not measured (no load cells)'}

    def cleanup(self, ctx):
        for arm in arms_of(self.params['arm']):
            ctx.drums(arm, 0.0)


class Idle(Primitive):
    NAME = 'idle'
    SUMMARY = 'Do nothing for t_wait; measures idle power.'
    STOPS_WHEN = 't_wait elapsed'
    FAILS_WHEN = 'all stop, override, cancel'
    PARAMS = [
        Param('t_wait_s', float, unit='s', min=0.0, help='time to wait'),
    ]

    def run(self, ctx):
        wait = self.params['t_wait_s']
        ctx.set_timeout(wait + 5.0)
        elapsed = 0.0
        while elapsed < wait:
            step = min(0.1, wait - elapsed)
            ctx.sleep(step)
            elapsed += step
            ctx.feedback(elapsed / wait if wait else 1.0, f'idle {elapsed:.1f} / {wait:.1f} s')
        return {}


class ScanTerrain(Primitive):
    NAME = 'scan_terrain'
    SUMMARY = 'Map a region (not capable yet: no perception sensor).'
    CAPABLE = False
    STOPS_WHEN = 'region mapped'
    FAILS_WHEN = 'localization lost'
    PARAMS = [Param('region', str, default='', help='region description')]


class Recharge(Primitive):
    NAME = 'recharge'
    SUMMARY = 'Return to charger and charge (not capable yet: no SoC estimate or docking).'
    CAPABLE = False
    STOPS_WHEN = 'SoC target reached'
    FAILS_WHEN = 'timeout'
    PARAMS = [Param('soc_target', float, default=1.0, min=0.0, max=1.0, help='target state of charge 0..1')]


CATALOG = {cls.NAME: cls for cls in (
    DriveStraight, PointTurn, ArcTurn, SetArm, Excavate, Deposit, ScanTerrain, Recharge, Idle)}
