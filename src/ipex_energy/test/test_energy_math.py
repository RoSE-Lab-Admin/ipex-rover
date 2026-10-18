import math

from ipex_energy.energy_math import Channel, EnergyAccumulator


def make(mode='abs', sign=1.0, stale_timeout_s=0.5):
    ch = Channel(name='a', group='g', current_mode=mode, current_sign=sign)
    return EnergyAccumulator(channels=[ch], stale_timeout_s=stale_timeout_s), ch


def feed(acc, v, i, t):
    acc.set_voltage('a', v, t)
    acc.set_current('a', i, t)
    acc.step(t)


def test_constant_power_integrates_linearly():
    acc, ch = make()
    for k in range(11):          # 0.0 .. 1.0 s at 10 Hz, 24 V x 2 A = 48 W
        feed(acc, 24.0, 2.0, k * 0.1)
    assert math.isclose(ch.power_w, 48.0)
    # First step has dt = 0, so exactly 1.0 s of 48 W is integrated.
    assert math.isclose(ch.energy_j, 48.0, rel_tol=1e-9)


def test_abs_mode_ignores_current_direction():
    acc, ch = make(mode='abs')
    feed(acc, 10.0, -3.0, 0.0)
    assert math.isclose(ch.power_w, 30.0)


def test_signed_mode_applies_sensor_sign():
    acc, ch = make(mode='signed', sign=-1.0)
    feed(acc, 25.0, -0.5, 0.0)
    assert math.isclose(ch.power_w, 12.5)


def test_stale_channel_counts_zero_and_is_flagged():
    acc, ch = make(stale_timeout_s=0.5)
    feed(acc, 24.0, 1.0, 0.0)
    acc.step(2.0)                # no new data for 2 s
    assert ch.stale
    assert ch.power_w == 0.0


def test_nan_is_stale():
    acc, ch = make()
    feed(acc, 24.0, float('nan'), 0.0)
    assert ch.stale
    assert ch.power_w == 0.0


def test_reset_zeroes_energy():
    acc, ch = make()
    for k in range(5):
        feed(acc, 24.0, 1.0, k * 0.1)
    assert ch.energy_j > 0.0
    acc.reset(0.5)
    assert ch.energy_j == 0.0
    assert acc.elapsed(0.5) == 0.0


def test_group_totals_include_total():
    a = Channel(name='a', group='drive')
    b = Channel(name='b', group='arm', current_mode='signed', current_sign=1.0)
    acc = EnergyAccumulator(channels=[a, b])
    for name, v, i in (('a', 10.0, 1.0), ('b', 20.0, 2.0)):
        acc.set_voltage(name, v, 0.0)
        acc.set_current(name, i, 0.0)
    acc.step(0.0)
    totals = acc.group_totals()
    assert acc.groups() == ['drive', 'arm', 'total']
    assert math.isclose(totals['total'][0], 50.0)
