import glob
import os
import textwrap

import pytest

from ipex_tools.missions import MissionError, load_mission, validate_mission

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLES = sorted(glob.glob(os.path.join(HERE, '..', 'missions', '*.yaml')))


def write(tmp_path, text):
    path = tmp_path / 'm.yaml'
    path.write_text(textwrap.dedent(text))
    return str(path)


@pytest.mark.parametrize('path', EXAMPLES)
def test_installed_examples_are_valid(path):
    assert validate_mission(load_mission(path)) == []


def test_short_and_long_step_forms(tmp_path):
    m = load_mission(write(tmp_path, """
        steps:
          - idle: {t_wait_s: 1}
          - primitive: set_arm
            params: {arm: front, theta_deg: 0}
            tag: lower
    """))
    assert [s['name'] for s in m['steps']] == ['idle', 'set_arm']
    assert m['steps'][1]['tag'] == 'lower'
    assert m['name'] == 'm'
    assert validate_mission(m) == []


def test_validation_reports_problems(tmp_path):
    m = load_mission(write(tmp_path, """
        steps:
          - fly: {}
          - set_arm: {theta_deg: 0, bogus: 1}
          - scan_terrain: {}
          - excavate: {m_target_kg: 2}
    """))
    problems = validate_mission(m)
    assert len(problems) == 4


def test_bad_on_failure(tmp_path):
    with pytest.raises(MissionError):
        load_mission(write(tmp_path, """
            on_failure: maybe
            steps:
              - idle: {t_wait_s: 1}
        """))
