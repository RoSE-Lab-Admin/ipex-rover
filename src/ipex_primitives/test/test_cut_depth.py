import math

from ipex_primitives.library import angle_for_depth_deg, cut_depth_m
from ipex_primitives.server import CONFIG_DEFAULTS


def test_matches_phd_sim_point():
    # PhD sim: -45 deg shoulder angle cuts 8.8 cm below the surface.
    assert abs(cut_depth_m(-45.0, CONFIG_DEFAULTS) - 0.088) < 0.001


def test_inverse():
    for depth in (0.0, 0.02, 0.05, 0.06):
        angle = angle_for_depth_deg(depth, CONFIG_DEFAULTS)
        assert abs(cut_depth_m(angle, CONFIG_DEFAULTS) - depth) < 1e-9


def test_reach_at_limits():
    # Home (+45) is well clear of the ground; the -40 deg limit reaches ~6.4 cm.
    assert cut_depth_m(45.0, CONFIG_DEFAULTS) < -0.4
    assert 0.06 < cut_depth_m(CONFIG_DEFAULTS['arm_min_deg'], CONFIG_DEFAULTS) < 0.07
    # Ground contact starts around -28 deg.
    assert -30.0 < angle_for_depth_deg(0.0, CONFIG_DEFAULTS) < -27.0
    assert math.isnan(angle_for_depth_deg(1.0, CONFIG_DEFAULTS))
