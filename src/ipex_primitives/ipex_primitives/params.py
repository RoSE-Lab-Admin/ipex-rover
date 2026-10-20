"""Primitive parameter specs: parsing, unit aliases, validation (ROS-free).

Canonical parameter names carry their SI unit as a suffix (theta_rad,
speed_mps, duration_s, ...). For convenience every *_rad / *_rad_s
parameter also accepts degrees (*_deg / *_deg_s), and a spec can declare
extra aliases with a conversion factor (e.g. drum_steps_s -> omega_drum_rad_s).
"""

import math
from dataclasses import dataclass, field

import yaml


REQUIRED = object()


class ParamError(ValueError):
    """Invalid, missing, or unknown parameter."""


@dataclass
class Param:
    name: str
    type: type = float
    default: object = REQUIRED
    unit: str = ''
    help: str = ''
    choices: tuple = None
    min: float = None
    max: float = None
    aliases: dict = field(default_factory=dict)   # alias name -> factor to canonical
    unsupported: bool = False                      # accepted name, but not implemented yet

    def all_aliases(self):
        """Explicit aliases plus automatic degree aliases for radian params."""
        aliases = dict(self.aliases)
        if self.name.endswith('_rad_s'):
            aliases[self.name[:-len('_rad_s')] + '_deg_s'] = math.pi / 180.0
        elif self.name.endswith('_rad'):
            aliases[self.name[:-len('_rad')] + '_deg'] = math.pi / 180.0
        return aliases

    @property
    def required(self):
        return self.default is REQUIRED


def parse_params_yaml(text):
    """YAML text -> dict. Empty text -> {}."""
    if text is None or not str(text).strip():
        return {}
    data = yaml.safe_load(text)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ParamError('params must be a YAML mapping, e.g. {arm: front, theta_deg: -10}')
    return data


def parse_key_values(items):
    """['arm=front', 'theta_deg=-10'] -> {'arm': 'front', 'theta_deg': -10}."""
    out = {}
    for item in items:
        if '=' not in item:
            raise ParamError(f"expected key=value, got '{item}'")
        key, value = item.split('=', 1)
        key = key.strip()
        if not key:
            raise ParamError(f"empty parameter name in '{item}'")
        out[key] = yaml.safe_load(value) if value.strip() else ''
    return out


def _coerce(spec, value):
    if spec.type is bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str) and value.lower() in ('true', 'false', 'yes', 'no', '1', '0'):
            return value.lower() in ('true', 'yes', '1')
        raise ParamError(f'{spec.name}: expected true/false, got {value!r}')
    if spec.type is str:
        return str(value)
    try:
        number = spec.type(value)
    except (TypeError, ValueError):
        raise ParamError(f'{spec.name}: expected a number, got {value!r}') from None
    if isinstance(number, float) and not math.isfinite(number):
        raise ParamError(f'{spec.name}: must be finite')
    return number


def resolve_params(specs, given):
    """Validate `given` (dict) against `specs` (list of Param).

    Returns (params, notes): canonical-name dict with defaults filled in, and
    a list of notes (e.g. unit conversions applied). Raises ParamError.
    Unsupported params are returned under their canonical name so the
    primitive can reject them with failure code UNSUPPORTED.
    """
    by_alias = {}
    for spec in specs:
        by_alias[spec.name] = (spec, 1.0)
        for alias, factor in spec.all_aliases().items():
            by_alias[alias] = (spec, factor)

    params, notes, seen = {}, [], {}

    for key, value in given.items():
        if key not in by_alias:
            valid = ', '.join(sorted(by_alias))
            raise ParamError(f"unknown parameter '{key}'. Valid: {valid}")
        spec, factor = by_alias[key]
        if spec.name in seen:
            raise ParamError(
                f"'{key}' and '{seen[spec.name]}' set the same parameter ({spec.name})")
        seen[spec.name] = key

        coerced = _coerce(spec, value)
        if factor != 1.0:
            coerced = float(coerced) * factor
            notes.append(f'{key}={value} -> {spec.name}={coerced:.6g}')
        params[spec.name] = coerced

    for spec in specs:
        if spec.name not in params:
            if spec.required and not spec.unsupported:
                raise ParamError(f"missing required parameter '{spec.name}'"
                                 + (f' ({spec.unit})' if spec.unit else ''))
            if not spec.required:
                params[spec.name] = spec.default
            continue

        value = params[spec.name]
        if spec.choices is not None and value not in spec.choices:
            raise ParamError(f'{spec.name}: must be one of {list(spec.choices)}, got {value!r}')
        if spec.min is not None and value < spec.min:
            raise ParamError(f'{spec.name}: {value} is below the minimum {spec.min}')
        if spec.max is not None and value > spec.max:
            raise ParamError(f'{spec.name}: {value} is above the maximum {spec.max}')

    return params, notes
