import math
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

from ipex_primitives.depth_hold import (
    DEPTH_HOLD_DEFAULTS, GEOMETRY_DEFAULTS, DepthHold, PitchFilter, angle_for_depth_pitched_deg,
    correction_gain, cut_depth_pitched_m, pitch_from_quaternion)
from ipex_primitives.library import angle_for_depth_deg, cut_depth_m
from ipex_primitives.server import CONFIG_DEFAULTS

CFG = dict(CONFIG_DEFAULTS)
SRC = os.path.join(os.path.dirname(__file__), '..', '..')


# ------------------------------------------------------------------ geometry

def test_zero_pitch_is_todays_cut_depth():
    for arm in ('front', 'rear'):
        for angle in (-30.0, -28.7, -20.0, 0.0):
            assert abs(cut_depth_pitched_m(arm, angle, 0.0, CFG) - cut_depth_m(angle, CFG)) < 1e-12
        for depth in (0.0, 0.01, 0.0175):
            assert abs(angle_for_depth_pitched_deg(arm, depth, 0.0, CFG)
                       - angle_for_depth_deg(depth, CFG)) < 1e-9


def test_inverse_round_trip():
    for arm in ('front', 'rear'):
        for d_pitch in (-3.0, -1.0, 0.5, 2.0, 4.0):
            angle = angle_for_depth_pitched_deg(arm, 0.01, d_pitch, CFG)
            assert abs(cut_depth_pitched_m(arm, angle, d_pitch, CFG) - 0.01) < 1e-9


def test_table_from_the_derivation():
    # 1 cm target (-28.7 deg at zero pitch): +2 deg nose down -> front -23.9, rear -33.5
    assert abs(angle_for_depth_pitched_deg('front', 0.01, 2.0, CFG) - (-23.9)) < 0.1
    assert abs(angle_for_depth_pitched_deg('rear', 0.01, 2.0, CFG) - (-33.5)) < 0.1
    # Uncorrected front arm at +2 deg cuts ~3.8 cm
    assert abs(cut_depth_pitched_m('front', -28.7, 2.0, CFG) - 0.038) < 0.001


def test_gain_matches_geometry():
    for angle in (-30.0, -10.0):
        assert 2.2 < correction_gain(angle, CFG) < 2.5
        assert correction_gain(angle, CFG) < CFG['depth_hold_max_ratio']


def test_pitch_sign():
    # Rotation about +y (FLU) by +5 deg = nose down = +5
    half = math.radians(5.0) / 2.0
    assert abs(math.degrees(pitch_from_quaternion(0.0, math.sin(half), 0.0, math.cos(half))) - 5.0) < 1e-9


# ------------------------------------------------------------------ the hold

def make(arm='front', nominal=-28.7, **overrides):
    cfg = dict(CFG)
    cfg.update(overrides)
    return DepthHold(arm, nominal, cfg)


def test_corrects_the_right_way_for_each_arm():
    front, rear = make('front'), make('rear')
    assert front.update(2.0, now=10.0) == pytest.approx(-23.9, abs=0.1)   # nose down: front up
    assert rear.update(2.0, now=10.0) == pytest.approx(-30.0)             # rear down, stops at -30


def test_never_below_min_or_above_max():
    h = make('rear', nominal=-28.7)
    for d in (0.5, 1.0, 2.0, 3.0, 4.0):
        target = h.compute(d)
        assert target is None or target >= -30.0
    assert h.compute(2.0) == -30.0
    assert 'clamped' in h.reason
    h = make('front', nominal=40.0)
    assert h.compute(3.0) <= CFG['arm_max_deg']
    assert make('front', nominal=-35.0).nominal_deg == -30.0     # dig angle raised to the limit


def test_ratio_limit():
    h = make('front', depth_hold_max_ratio=2.0)       # geometry needs ~2.4
    assert h.compute(1.0) is None
    assert 'over 2' in h.reason
    assert make('front').compute(1.0) is not None     # 3.0 allows it


def test_correction_cap():
    h = make('front', nominal=-28.7, depth_hold_max_correction_deg=3.0)
    assert h.compute(2.0) == pytest.approx(-25.7)
    assert 'capped' in h.reason


def test_pitch_limit_and_stale_imu_freeze():
    h = make('front')
    assert h.update(4.5, now=10.0) is None and 'limit' in h.reason
    assert h.update(None, now=11.0) is None and 'IMU' in h.reason
    assert h.update(float('nan'), now=12.0) is None


def test_deadband_and_interval():
    h = make('front', depth_hold_deadband_deg=0.5, depth_hold_min_interval_s=1.0)
    assert h.update(0.1, now=10.0) is None             # 0.24 deg < 0.5 deadband
    first = h.update(1.0, now=10.0)
    assert first is not None
    assert h.update(2.0, now=10.5) is None             # < 1 s since the last command
    assert h.update(2.0, now=11.1) is not None
    fast = make('front')                               # brake-off defaults: 0.25 deg, 0.2 s
    assert fast.update(0.1, now=10.0) is None          # 0.24 deg < 0.25
    assert fast.update(0.2, now=10.0) is not None
    assert fast.update(0.5, now=10.1) is None
    assert fast.update(0.5, now=10.25) is not None


def test_rebaseline_keeps_the_depth_where_the_arm_stopped():
    h = make('front')
    h.rebaseline(-24.0, 1.0)
    assert abs(cut_depth_pitched_m('front', -24.0, 1.0, CFG) - h.target_depth_m()) < 1e-9


def test_pitch_filter():
    f = PitchFilter(1.0)
    f.add(0.0, 0.0)
    f.add(10.0, 1.0)
    assert 6.0 < f.get(1.0, 0.5) < 6.5                   # 1 - e^-1 = 0.63 of the step
    assert f.get(2.0, 0.5) is None                       # stale


# ------------------------------------------------------------------ excavate primitive

class PitchCtx:
    """Arms reach targets at once; pitch ramps 0 -> 2 deg nose down over the dig."""

    def __init__(self, pitch_rate_deg_s=0.4, imu=True, teensy_holds=True, drop_hold_at=None,
                 imu_drop_at=None):
        self.t = 0.0
        self.teensy_holds, self.drop_hold_at, self.imu_drop_at = teensy_holds, drop_hold_at, imu_drop_at
        self.hold_msgs = []        # (t, arm, on)
        self.rate = pitch_rate_deg_s
        self.imu = imu
        self.angle = {'front': 45.0, 'rear': 45.0}
        self.gotos = []
        self.status = []

    def check(self): pass
    def feedback(self, progress, status): pass
    def set_timeout(self, s): pass
    def drive(self, v, w): pass
    def stop_drive(self): pass
    def check_stall(self): pass
    def arm_homed(self, arm): return True
    def arm_angle_deg(self, arm): return self.angle[arm]
    def arm_speed(self, arm, deg_s): pass
    def drums(self, arm, rad_s): pass
    def warn(self, msg): pass

    def sleep(self, s):
        self.t += s

    def arm_goto(self, arm, deg):
        self.angle[arm] = deg
        self.gotos.append((self.t, arm, deg))

    def pitch_deg(self):
        if not self.imu or (self.imu_drop_at is not None and self.t >= self.imu_drop_at):
            return None
        return min(self.rate * self.t, 2.0)

    def arm_hold(self, arm, on):
        self.hold_msgs.append((self.t, arm, on))

    def arm_holding(self, arm):
        if not self.teensy_holds or (self.drop_hold_at is not None and self.t >= self.drop_hold_at):
            return False
        return any(a == arm and on for _, a, on in self.hold_msgs)

    def depth_hold_status(self, arm, values):
        self.status.append((arm, values))


def run_excavate(given, ctx):
    from ipex_primitives.library import CATALOG
    from ipex_primitives.params import resolve_params
    cls = CATALOG['excavate']
    params, _ = resolve_params(cls.PARAMS, given)
    return cls(params, CFG).run(ctx)


def test_excavate_holds_depth_as_the_nose_drops():
    ctx = PitchCtx()
    out = run_excavate({'arm': 'both', 'd_dig_m': 0.01, 'duration_s': 10.0,
                        'drum_rpm': 2.0, 'hold_depth': True}, ctx)
    d_pitch = 2.0 - ctx.rate * (CFG['drum_spinup_s'])   # pitch zero is taken after spin-up
    assert out['hold_corrections'] > 0
    # Each arm ends where the 1 cm depth needs it (within the deadband), never below -30
    for arm in ('front', 'rear'):
        want = max(angle_for_depth_pitched_deg(arm, 0.01, d_pitch, CFG), -30.0)
        assert abs(ctx.angle[arm] - want) <= CFG['depth_hold_deadband_deg'] + 1e-6
        assert all(deg >= -30.0 for _, a, deg in ctx.gotos if a == arm)
    # Corrections at least min_interval apart, per arm
    for arm in ('front', 'rear'):
        times = [t for t, a, _ in ctx.gotos if a == arm][1:]       # first = lowering
        assert all(b - a >= CFG['depth_hold_min_interval_s'] - 1e-6 for a, b in zip(times, times[1:]))
    assert ctx.status
    # Brake-off hold: keepalives during the dig, HOLD OFF from cleanup
    assert sum(1 for _, a, on in ctx.hold_msgs if a == 'front' and on) > 100


def run_excavate_with_cleanup(given, ctx):
    from ipex_primitives.library import CATALOG
    from ipex_primitives.params import resolve_params
    cls = CATALOG['excavate']
    params, _ = resolve_params(cls.PARAMS, given)
    prim = cls(params, CFG)
    try:
        return prim.run(ctx)
    finally:
        prim.cleanup(ctx)


HOLD = {'arm': 'front', 'd_dig_m': 0.01, 'duration_s': 10.0, 'drum_rpm': 2.0, 'hold_depth': True}


def test_cleanup_releases_the_hold():
    ctx = PitchCtx()
    run_excavate_with_cleanup(HOLD, ctx)
    assert ctx.hold_msgs[-1][1:] == ('front', False)


def test_brake_off_hold_fails_safe():
    from ipex_primitives.base import Codes, PrimitiveFailure
    # Teensy never enters hold mode -> fail within ~2 s of the dig start
    ctx = PitchCtx(teensy_holds=False)
    with pytest.raises(PrimitiveFailure) as e:
        run_excavate_with_cleanup(HOLD, ctx)
    assert e.value.code == Codes.HARDWARE and 'did not enter' in e.value.message
    assert ctx.hold_msgs[-1][2] is False
    # Teensy leaves hold mid-dig (slip / timeout) -> fail, HOLD OFF
    ctx = PitchCtx(drop_hold_at=6.0)
    with pytest.raises(PrimitiveFailure) as e:
        run_excavate_with_cleanup(HOLD, ctx)
    assert 'left hold mode' in e.value.message and ctx.hold_msgs[-1][2] is False
    # IMU goes quiet mid-dig -> fail after 2 s, HOLD OFF
    ctx = PitchCtx(imu_drop_at=6.0)
    with pytest.raises(PrimitiveFailure) as e:
        run_excavate_with_cleanup(HOLD, ctx)
    assert 'no IMU pitch' in e.value.message and 7.9 < ctx.t < 8.5
    assert ctx.hold_msgs[-1][2] is False


def test_excavate_hold_needs_imu_and_a_legal_angle():
    from ipex_primitives.base import PrimitiveFailure
    with pytest.raises(PrimitiveFailure):
        run_excavate({'d_dig_m': 0.01, 'drum_rpm': 2.0, 'hold_depth': True}, PitchCtx(imu=False))
    with pytest.raises(PrimitiveFailure):
        run_excavate({'dig_angle_deg': -35.0, 'drum_rpm': 2.0, 'hold_depth': True}, PitchCtx())
    # Without hold_depth nothing changes: -35 is fine, no IMU needed
    out = run_excavate({'dig_angle_deg': -35.0, 'drum_rpm': 2.0, 'duration_s': 1.0},
                       PitchCtx(imu=False))
    assert 'hold_corrections' not in out


# ------------------------------------------------------------------ config agreement

def test_pivot_distance_matches_urdf():
    path = os.path.join(SRC, 'ipex_description', 'urdf', 'ipex.xacro')
    if not os.path.exists(path):
        pytest.skip('ipex_description source not found')
    joints = {j.get('name'): j for j in ET.parse(path).getroot().iter('joint')}

    def x(name):
        return float(joints[name].find('origin').get('xyz').split()[0])

    front = x('front_shoulder_rev') - x('back_left_wheel_rev')
    rear = x('front_left_wheel_rev') - x('back_shoulder_rev')
    for a in (front, rear):
        assert abs(a - CFG['arm_pivot_dist_m']) < 0.002


def test_yaml_files_agree():
    cfg = os.path.join(SRC, 'ipex_bringup', 'config')
    if not os.path.isdir(cfg):
        pytest.skip('ipex_bringup source not found')
    with open(os.path.join(cfg, 'primitives.yaml')) as f:
        prim = yaml.safe_load(f)['primitive_server']['ros__parameters']
    with open(os.path.join(cfg, 'teleop.yaml')) as f:
        tele = yaml.safe_load(f)['controller']['ros__parameters']
    for key in list(GEOMETRY_DEFAULTS) + list(DEPTH_HOLD_DEFAULTS):
        assert prim[key] == tele[key], key
        assert prim[key] == CFG[key], key
