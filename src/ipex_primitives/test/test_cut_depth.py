import math

from ipex_primitives.library import angle_for_depth_deg, cut_depth_m
from ipex_primitives.server import CONFIG_DEFAULTS


def test_matches_rover_check():
    # Depth below the tire tread: 0.149 - (0.3208 + 0.3786 sin(theta)).
    # On the stand 2026-10-31: -28.7 deg put the lowest tooth 1 cm below the tread.
    assert abs(cut_depth_m(-28.7, CONFIG_DEFAULTS) - 0.010) < 0.001
    assert abs(angle_for_depth_deg(0.010, CONFIG_DEFAULTS) - (-28.7)) < 0.1


def test_inverse():
    for depth in (0.0, 0.03, 0.06):
        angle = angle_for_depth_deg(depth, CONFIG_DEFAULTS)
        assert abs(cut_depth_m(angle, CONFIG_DEFAULTS) - depth) < 1e-9


def test_reach_at_limits():
    # Home (+45) is well clear of the ground; the -40 deg limit reaches ~7.2 cm.
    assert cut_depth_m(45.0, CONFIG_DEFAULTS) < -0.4
    assert 0.070 < cut_depth_m(CONFIG_DEFAULTS['arm_min_deg'], CONFIG_DEFAULTS) < 0.074
    # Contact with the tread plane at about -27.0 deg.
    assert -27.3 < angle_for_depth_deg(0.0, CONFIG_DEFAULTS) < -26.7
    assert math.isnan(angle_for_depth_deg(1.0, CONFIG_DEFAULTS))
