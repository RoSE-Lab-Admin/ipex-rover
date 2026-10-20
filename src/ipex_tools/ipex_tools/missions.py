"""Mission files: a YAML list of primitives to run in order.

    name: dig_cycle                 # optional (defaults to the file name)
    description: Dig in front, turn around, dump.
    record: true                    # optional: record a bag for the whole mission
    on_failure: stop                # stop (default) | continue
    steps:
      - set_arm: {arm: front, theta_deg: 0}          # short form
      - primitive: drive_straight                    # long form (adds a tag)
        params: {distance_m: 1.0, speed_mps: 0.2}
        tag: approach

Missions are validated against the primitive catalog before anything moves.
"""

import os

import yaml

from ipex_primitives.library import CATALOG
from ipex_primitives.params import ParamError, resolve_params


class MissionError(ValueError):
    """Mission file could not be read or is invalid."""


def find_mission(arg):
    """A path, or the name of an example installed with ipex_tools."""
    if os.path.isfile(arg):
        return arg
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory('ipex_tools'), 'missions')
    except Exception:   # noqa: BLE001
        share = None
    if share:
        for candidate in (arg, arg + '.yaml'):
            path = os.path.join(share, candidate)
            if os.path.isfile(path):
                return path
    raise MissionError(f"mission '{arg}' not found (not a file, not an installed example)")


def installed_examples():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory('ipex_tools'), 'missions')
        return sorted(f[:-5] for f in os.listdir(share) if f.endswith('.yaml'))
    except Exception:   # noqa: BLE001
        return []


def load_mission(path):
    with open(path) as f:
        try:
            data = yaml.safe_load(f) or {}
        except yaml.YAMLError as e:
            raise MissionError(f'{path}: invalid YAML: {e}') from None

    if not isinstance(data, dict) or 'steps' not in data:
        raise MissionError(f"{path}: expected a mapping with a 'steps' list")

    steps = []
    for i, raw in enumerate(data['steps'] or [], start=1):
        if isinstance(raw, dict) and 'primitive' in raw:
            name, params, tag = raw['primitive'], raw.get('params') or {}, raw.get('tag', '')
        elif isinstance(raw, dict) and len(raw) == 1:
            (name, params), tag = next(iter(raw.items())), ''
            params = params or {}
        else:
            raise MissionError(f'step {i}: use "- name: {{params}}" or "- primitive: name"')
        if not isinstance(params, dict):
            raise MissionError(f'step {i} ({name}): params must be a mapping')
        steps.append({'index': i, 'name': str(name), 'params': params,
                      'tag': str(tag) if tag else f'step{i}'})

    on_failure = data.get('on_failure', 'stop')
    if on_failure not in ('stop', 'continue'):
        raise MissionError("on_failure must be 'stop' or 'continue'")

    return {
        'name': data.get('name') or os.path.splitext(os.path.basename(path))[0],
        'description': data.get('description', ''),
        'record': bool(data.get('record', False)),
        'on_failure': on_failure,
        'steps': steps,
    }


def validate_mission(mission):
    """List of problems (empty = valid). Nothing is sent to the rover."""
    problems = []
    if not mission['steps']:
        problems.append('mission has no steps')
    for step in mission['steps']:
        cls = CATALOG.get(step['name'])
        label = f"step {step['index']} ({step['name']})"
        if cls is None:
            problems.append(f"{label}: unknown primitive. Available: {', '.join(CATALOG)}")
            continue
        if not cls.CAPABLE:
            problems.append(f'{label}: not capable on this rover yet ({cls.SUMMARY})')
            continue
        try:
            resolved, _ = resolve_params(cls.PARAMS, step['params'])
        except ParamError as e:
            problems.append(f'{label}: {e}')
            continue
        for spec in cls.PARAMS:
            if spec.unsupported and resolved.get(spec.name) is not None:
                problems.append(f"{label}: parameter '{spec.name}' not supported yet ({spec.help})")
    return problems
