import math

import pytest

from ipex_primitives.base import PrimitiveFailure
from ipex_primitives.library import CATALOG, DRUM_RPM_TO_RAD_S
from ipex_primitives.params import resolve_params
from ipex_primitives.server import CONFIG_DEFAULTS


class FakeCtx:
    """Arms reach their target at once; odometry follows the drive command."""

    def __init__(self):
        self.x = 0.0
        self.v = 0.0
        self.angle = {'front': 45.0, 'rear': 45.0}
        self.drum_cmds = []

    def check(self):
        pass

    def sleep(self, s):
        self.x += self.v * s

    def feedback(self, progress, status):
        pass

    def set_timeout(self, s):
        pass

    def drive(self, v, w):
        self.v = v

    def stop_drive(self):
        self.v = 0.0

    def pose(self):
        return (self.x, 0.0, 0.0)

    def check_stall(self):
        pass

    def arm_homed(self, arm):
        return True

    def arm_angle_deg(self, arm):
        return self.angle[arm]

    def arm_speed(self, arm, deg_s):
        pass

    def arm_goto(self, arm, deg):
        self.angle[arm] = deg

    def drums(self, arm, rad_s):
        self.drum_cmds.append((arm, rad_s))


def run(given):
    cls = CATALOG['bulldoze']
    params, _ = resolve_params(cls.PARAMS, given)
    ctx = FakeCtx()
    out = cls(params, CONFIG_DEFAULTS).run(ctx)
    return out, ctx


def test_default_blade_sits_about_1cm_above_ground():
    out, ctx = run({'duration_s': 1.0})
    assert out['blade_angle_deg'] == -25.3
    assert 0.005 < out['clearance_m'] < 0.015
    assert ctx.angle['front'] == pytest.approx(-25.3)
    assert ctx.angle['rear'] == 45.0      # front arm only by default


def test_clearance_sets_the_angle():
    out, ctx = run({'arm': 'both', 'h_clear_m': 0.02, 'duration_s': 0.5})
    assert abs(out['clearance_m'] - 0.02) < 1e-4
    assert ctx.angle['front'] == ctx.angle['rear'] == pytest.approx(out['blade_angle_deg'], abs=0.01)


def test_distance_mode_stops_at_distance():
    out, ctx = run({'distance_m': 0.1, 'speed_mps': 0.05})
    assert 0.1 <= out['distance_m'] < 0.1 + 0.05 * 0.05 + 1e-9
    assert ctx.v == 0.0


def test_drum_direction_follows_sign():
    sign = CONFIG_DEFAULTS['drum_dig_sign_front']
    _, ctx = run({'drum_rpm': 2.0, 'duration_s': 0.1})
    assert ctx.drum_cmds[0] == ('front', pytest.approx(2.0 * DRUM_RPM_TO_RAD_S * sign))
    _, ctx = run({'drum_rpm': -2.0, 'duration_s': 0.1})
    assert ctx.drum_cmds[0] == ('front', pytest.approx(-2.0 * DRUM_RPM_TO_RAD_S * sign))


def test_drums_off():
    _, ctx = run({'drum_rpm': 0.0, 'duration_s': 0.1})
    assert ctx.drum_cmds == []


def test_rejects_bad_requests():
    with pytest.raises(PrimitiveFailure):
        run({'h_clear_m': 2.0})                        # out of reach
    with pytest.raises(PrimitiveFailure):
        run({'distance_m': 0.5, 'speed_mps': 0.0})     # would never get there
    with pytest.raises(PrimitiveFailure):
        run({'drum_rpm': 10.0})                        # above the drum limit
    assert math.isfinite(run({'duration_s': 0.0})[0]['clearance_m'])
