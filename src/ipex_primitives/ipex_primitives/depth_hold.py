"""IMU pitch depth hold: keep the drum cut at a fixed depth while the chassis pitches.

ROS-free, shared by the excavate primitive (server.py / library.py) and teleop
excavation mode (ipex_motion/controller.py).

Model (side view, 2D; roll ignored): while one arm digs, the wheels at the
OTHER end stay on the original ground and the chassis rotates about their
contact point as the digging end sinks into the trench or rides up on spoil.
Pitch is measured from the start of the dig (d_pitch = pitch - pitch at start),
so a rover parked on a slope digs relative to that slope, not to level.

  s = +1 front arm, -1 rear arm; d = d_pitch (rad, nose down +)
  cut height = (H + L sin a) cos d - s (A + L cos a) sin d - r        (depth = -height)
  a = s d + asin((r - depth - H cos d + s A sin d) / L)               (inverse kinematics)

  H arm_pivot_height_m, L arm_length_m, r drum_cut_radius_m (as cut_depth_m),
  A arm_pivot_dist_m: far axle to this arm's pivot, horizontal (0.469 m, URDF).
At d = 0 these are cut_depth_m / angle_for_depth_deg.

Not seen by this model: uniform sinkage (no pitch change), roll, arm slip or
backlash (the arm angle is the open-loop commanded one).
"""

import math


ARM_SIGN = {'front': 1.0, 'rear': -1.0}

# Geometry (also used by cut_depth_m). Overridden by primitives.yaml / teleop.yaml.
GEOMETRY_DEFAULTS = {
    'arm_pivot_height_m': 0.3208,   # pivot above the tire tread (CAD 0.3183 + 0.0025; rover-checked 2026-10-31)
    'arm_length_m': 0.3786,         # shoulder pivot to drum axle (ipex.xacro)
    'drum_cut_radius_m': 0.149,     # drum tooth-tip radius (mesh)
    'arm_pivot_dist_m': 0.469,      # far axle to shoulder pivot, horizontal (ipex.xacro: 0.46875 both arms)
}

# Depth hold limits. Sources: see primitives.yaml (estimates unless noted).
DEPTH_HOLD_DEFAULTS = {
    'depth_hold_min_deg': -30.0,            # lowest angle the hold may command (operator limit, 2026-11-18)
    'depth_hold_max_pitch_deg': 4.0,        # |pitch change| above this: stop correcting
    'depth_hold_max_correction_deg': 10.0,  # |arm angle - dig angle| cap
    'depth_hold_max_ratio': 3.0,            # |correction| <= ratio x |pitch change| (geometry ~2.4)
    'depth_hold_deadband_deg': 0.5,         # re-command only when the target moved this much
    'depth_hold_min_interval_s': 1.0,       # and no sooner than this after the last command
    'pitch_filter_tau_s': 1.0,              # IMU pitch low-pass time constant
    'imu_stale_s': 0.5,                     # pitch older than this = no pitch (UM7 at 60 Hz)
}


def pitch_from_quaternion(x, y, z, w):
    """Pitch (rad) from a body FLU orientation quaternion; nose down = + (REP-103,
    sign checked on the rover 2026-11-15)."""
    s = 2.0 * (w * y - z * x)
    return math.asin(max(-1.0, min(1.0, s)))


def cut_depth_pitched_m(arm, angle_deg, d_pitch_deg, config):
    """Depth of the cut below the original ground (negative = above it)."""
    s = ARM_SIGN[arm]
    a, d = math.radians(angle_deg), math.radians(d_pitch_deg)
    H, L = config['arm_pivot_height_m'], config['arm_length_m']
    A, r = config['arm_pivot_dist_m'], config['drum_cut_radius_m']
    height = (H + L * math.sin(a)) * math.cos(d) - s * (A + L * math.cos(a)) * math.sin(d) - r
    return -height


def angle_for_depth_pitched_deg(arm, depth_m, d_pitch_deg, config):
    """Shoulder angle that cuts depth_m below the original ground (NaN if unreachable)."""
    s = ARM_SIGN[arm]
    d = math.radians(d_pitch_deg)
    H, L = config['arm_pivot_height_m'], config['arm_length_m']
    A, r = config['arm_pivot_dist_m'], config['drum_cut_radius_m']
    q = (r - depth_m - H * math.cos(d) + s * A * math.sin(d)) / L
    if not -1.0 <= q <= 1.0:
        return math.nan
    return math.degrees(s * d + math.asin(q))


def correction_gain(angle_deg, config):
    """Small-angle d(arm angle) / d(pitch) at this arm angle: 1 + A / (L cos a).
    ~2.4 at the dig angles (-30 to -10 deg). Same magnitude for both arms."""
    return 1.0 + config['arm_pivot_dist_m'] / (
        config['arm_length_m'] * math.cos(math.radians(angle_deg)))


class PitchFilter:
    """First-order low-pass on IMU pitch (rejects digging vibration) + staleness."""

    def __init__(self, tau_s):
        self.tau = max(0.0, float(tau_s))
        self.value = None       # filtered pitch, deg
        self.t = None           # time of the last sample (s, caller's clock)

    def add(self, pitch_deg, t):
        if not math.isfinite(pitch_deg):
            return
        if self.value is None or self.t is None or t <= self.t or self.tau == 0.0:
            self.value = pitch_deg
        else:
            k = 1.0 - math.exp(-(t - self.t) / self.tau)
            self.value += k * (pitch_deg - self.value)
        self.t = t

    def get(self, now, stale_s):
        """Filtered pitch (deg), or None if there is none or it is older than stale_s."""
        if self.value is None or self.t is None or now - self.t > stale_s:
            return None
        return self.value


class DepthHold:
    """Decides the arm angle that holds the cut depth for one arm.

    nominal_deg is the operator / primitive dig angle at d_pitch = 0; the target
    depth is the depth it cuts on the original ground. update() returns a new
    absolute arm angle to send (/ipex/<arm>_arm/goto), or None to leave the arm
    where it is. Every refusal is reported in self.reason (and self.warning
    when it is a sanity-check failure worth logging).

    Sanity checks (each one: no command, arm stays put):
      - no fresh pitch (IMU stale / not started)
      - |d_pitch| > max_pitch_deg (something is wrong, or about to tip)
      - target outside [min_deg, max_deg] -> clamped to the bound (never out of bounds)
      - |correction| > max_correction_deg (correction = target - nominal) -> clamped
      - |correction| > max_ratio x |d_pitch| + 0.05 deg  (the arm may never move
        more than a fixed ratio of the pitch change; geometry says ~2.4)
      - correction in the wrong direction for this arm (front: same sign as
        d_pitch, rear: opposite)
    Rate: only commands when the target differs from the last command by more
    than deadband_deg and min_interval_s has passed since the last command.
    """

    def __init__(self, arm, nominal_deg, config):
        self.arm = arm
        self.s = ARM_SIGN[arm]
        self.cfg = config
        self.nominal_deg = 0.0
        self.set_nominal(nominal_deg)       # within [min_deg, max_deg]
        self.last_cmd_deg = self.nominal_deg
        self.last_cmd_t = -1e9
        self.target_deg = self.nominal_deg
        self.d_pitch_deg = 0.0
        self.reason = 'starting'
        self.warning = None

    # ------------------------------------------------------------------ setup

    @property
    def min_deg(self):
        return max(self.cfg['depth_hold_min_deg'], self.cfg.get('arm_min_deg', -40.0))

    @property
    def max_deg(self):
        return self.cfg.get('arm_max_deg', 45.0)

    def target_depth_m(self):
        return cut_depth_pitched_m(self.arm, self.nominal_deg, 0.0, self.cfg)

    def set_nominal(self, nominal_deg):
        """New dig angle at d_pitch = 0 (tap, or re-baseline after a jog)."""
        self.nominal_deg = min(max(float(nominal_deg), self.min_deg), self.max_deg)
        return self.nominal_deg

    def rebaseline(self, current_deg, d_pitch_deg):
        """The operator moved the arm by hand: keep the depth it cuts now."""
        depth = cut_depth_pitched_m(self.arm, current_deg, d_pitch_deg, self.cfg)
        nominal = angle_for_depth_pitched_deg(self.arm, depth, 0.0, self.cfg)
        if math.isfinite(nominal):
            self.set_nominal(nominal)
        self.last_cmd_deg = float(current_deg)
        return self.nominal_deg

    # ----------------------------------------------------------------- update

    def compute(self, d_pitch_deg):
        """Target angle for this d_pitch with all bounds applied, or None."""
        c = self.cfg
        self.warning = None
        if d_pitch_deg is None or not math.isfinite(d_pitch_deg):
            self.reason = 'no fresh IMU pitch: holding the arm where it is'
            self.warning = self.reason
            return None
        self.d_pitch_deg = d_pitch_deg
        if abs(d_pitch_deg) > c['depth_hold_max_pitch_deg']:
            self.reason = (f'pitch change {d_pitch_deg:+.2f} deg is over the '
                           f'{c["depth_hold_max_pitch_deg"]:g} deg limit: not correcting')
            self.warning = self.reason
            return None

        raw = angle_for_depth_pitched_deg(self.arm, self.target_depth_m(), d_pitch_deg, c)
        if not math.isfinite(raw):
            self.reason = 'depth unreachable at this pitch: not correcting'
            self.warning = self.reason
            return None
        target, notes = raw, []

        max_corr = c['depth_hold_max_correction_deg']
        corr = target - self.nominal_deg
        if abs(corr) > max_corr:
            target = self.nominal_deg + math.copysign(max_corr, corr)
            notes.append(f'correction capped at {max_corr:g} deg')
        if target < self.min_deg or target > self.max_deg:
            target = min(max(target, self.min_deg), self.max_deg)
            notes.append(f'clamped to [{self.min_deg:g}, {self.max_deg:g}] deg')

        corr = target - self.nominal_deg
        ratio_limit = c['depth_hold_max_ratio'] * abs(d_pitch_deg) + 0.05
        if abs(corr) > ratio_limit:
            self.reason = (f'correction {corr:+.2f} deg is over {c["depth_hold_max_ratio"]:g} x '
                           f'pitch change {d_pitch_deg:+.2f} deg: not correcting')
            self.warning = self.reason
            return None
        if abs(corr) > 0.05 and corr * self.s * d_pitch_deg < 0.0:
            self.reason = (f'correction {corr:+.2f} deg is the wrong way for pitch '
                           f'{d_pitch_deg:+.2f} deg: not correcting')
            self.warning = self.reason
            return None

        self.target_deg = target
        self.reason = 'holding' + (f' ({"; ".join(notes)})' if notes else '')
        if notes:
            self.warning = f'{self.arm} arm depth hold: {"; ".join(notes)}'
        return target

    def update(self, d_pitch_deg, now):
        """New absolute arm angle to command, or None (within deadband, too soon,
        or a sanity check failed)."""
        target = self.compute(d_pitch_deg)
        if target is None:
            return None
        if abs(target - self.last_cmd_deg) <= self.cfg['depth_hold_deadband_deg']:
            return None
        if now - self.last_cmd_t < self.cfg['depth_hold_min_interval_s']:
            return None
        self.last_cmd_deg = target
        self.last_cmd_t = now
        return target

    def status(self):
        """[d_pitch_deg, nominal_deg, target_deg, last_cmd_deg, target_depth_m,
        predicted depth at the last command (m)] for /ipex/<arm>_arm/depth_hold."""
        return [float(self.d_pitch_deg), self.nominal_deg, float(self.target_deg),
                self.last_cmd_deg, self.target_depth_m(),
                cut_depth_pitched_m(self.arm, self.last_cmd_deg, self.d_pitch_deg, self.cfg)]
