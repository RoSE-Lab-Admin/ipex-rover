import math

import pytest

from ipex_primitives.library import CATALOG
from ipex_primitives.params import (
    Param, ParamError, parse_key_values, parse_params_yaml, resolve_params)


SPECS = [
    Param('arm', str, default='front', choices=('front', 'rear', 'both')),
    Param('theta_rad', float),
    Param('speed_mps', float, default=0.2, min=0.02),
    Param('omega_drum_rad_s', float, default=10.0, aliases={'drum_steps_s': 0.5}),
    Param('m_target_kg', float, default=None, unsupported=True),
]


def test_degree_alias_converts():
    params, notes = resolve_params(SPECS, {'theta_deg': -10})
    assert math.isclose(params['theta_rad'], math.radians(-10))
    assert notes


def test_explicit_alias_converts():
    params, _ = resolve_params(SPECS, {'theta_rad': 0.0, 'drum_steps_s': 100})
    assert params['omega_drum_rad_s'] == 50.0


def test_defaults_filled():
    params, _ = resolve_params(SPECS, {'theta_rad': 0.1})
    assert params['arm'] == 'front'
    assert params['speed_mps'] == 0.2
    assert params['m_target_kg'] is None


def test_missing_required():
    with pytest.raises(ParamError, match='theta_rad'):
        resolve_params(SPECS, {})


def test_unknown_param():
    with pytest.raises(ParamError, match='unknown parameter'):
        resolve_params(SPECS, {'theta_rad': 0, 'bogus': 1})


def test_choice_and_min_enforced():
    with pytest.raises(ParamError, match='must be one of'):
        resolve_params(SPECS, {'theta_rad': 0, 'arm': 'middle'})
    with pytest.raises(ParamError, match='minimum'):
        resolve_params(SPECS, {'theta_rad': 0, 'speed_mps': 0.0})


def test_same_param_twice_rejected():
    with pytest.raises(ParamError, match='same parameter'):
        resolve_params(SPECS, {'theta_rad': 0, 'theta_deg': 0})


def test_key_values_and_yaml_parsing():
    assert parse_key_values(['arm=both', 'theta_deg=-10']) == {'arm': 'both', 'theta_deg': -10}
    assert parse_params_yaml('{arm: rear, theta_deg: 5}') == {'arm': 'rear', 'theta_deg': 5}
    assert parse_params_yaml('') == {}
    with pytest.raises(ParamError):
        parse_params_yaml('- not\n- a mapping')


def test_catalog_has_all_spec_primitives():
    expected = {'drive_straight', 'point_turn', 'arc_turn', 'set_arm', 'excavate',
                'deposit', 'scan_terrain', 'recharge', 'idle'}
    assert set(CATALOG) == expected


def test_every_primitive_spec_resolves_with_minimal_params():
    minimal = {
        'drive_straight': {'distance_m': 1.0},
        'point_turn': {'dpsi_deg': 90},
        'arc_turn': {'r_arc_m': 1.0, 'dpsi_deg': 45},
        'set_arm': {'theta_deg': 0},
        'excavate': {},
        'deposit': {},
        'scan_terrain': {},
        'recharge': {},
        'idle': {'t_wait_s': 1.0},
    }
    for name, given in minimal.items():
        resolve_params(CATALOG[name].PARAMS, given)
