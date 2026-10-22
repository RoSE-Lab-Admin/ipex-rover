import math

from ipex_primitives.library import angle_for_depth_deg, cut_depth_m
from ipex_primitives.server import CONFIG_DEFAULTS


def test_matches_rover_measurement():
    # Measured on the rover 2026-10-21, loaded arm: -35 deg shoulder angle cuts 53 mm.
    assert abs(cut_depth_m(-35.0, CONFIG_DEFAULTS) - 0.053) < 0.001


def test_inverse():
    for depth in (0.0, 0.03, 0.06):
        angle = angle_for_depth_deg(depth, CONFIG_DEFAULTS)
        assert abs(cut_depth_m(angle, CONFIG_DEFAULTS) - depth) < 1e-9


def test_reach_at_limits():
    # Home (+45) is well clear of the ground; the -40 deg limit reaches ~7.9 cm.
    assert cut_depth_m(45.0, CONFIG_DEFAULTS) < -0.4
    assert 0.075 < cut_depth_m(CONFIG_DEFAULTS['arm_min_deg'], CONFIG_DEFAULTS) < 0.083
    # Ground contact starts around -26 deg.
    assert -27.0 < angle_for_depth_deg(0.0, CONFIG_DEFAULTS) < -24.5
    assert math.isnan(angle_for_depth_deg(1.0, CONFIG_DEFAULTS))
