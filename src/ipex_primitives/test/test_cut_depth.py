import math

from ipex_primitives.library import angle_for_depth_deg, cut_depth_m
from ipex_primitives.server import CONFIG_DEFAULTS


def test_cad_geometry():
    # Depth is below the CAD tire bottom: 0.149 - (0.3287 + 0.3786 sin(theta)).
    assert abs(cut_depth_m(-35.0, CONFIG_DEFAULTS) - 0.0375) < 0.001
    # (On 2026-10-21 the rover measured 53 mm at -35 deg: ~16 mm of sinkage, not modeled.)


def test_inverse():
    for depth in (0.0, 0.03, 0.06):
        angle = angle_for_depth_deg(depth, CONFIG_DEFAULTS)
        assert abs(cut_depth_m(angle, CONFIG_DEFAULTS) - depth) < 1e-9


def test_reach_at_limits():
    # Home (+45) is well clear of the ground; the -40 deg limit reaches ~6.4 cm.
    assert cut_depth_m(45.0, CONFIG_DEFAULTS) < -0.4
    assert 0.062 < cut_depth_m(CONFIG_DEFAULTS['arm_min_deg'], CONFIG_DEFAULTS) < 0.066
    # Contact with the tire-bottom plane at about -28.3 deg.
    assert -28.6 < angle_for_depth_deg(0.0, CONFIG_DEFAULTS) < -28.0
    assert math.isnan(angle_for_depth_deg(1.0, CONFIG_DEFAULTS))
