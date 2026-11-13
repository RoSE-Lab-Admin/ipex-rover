import math

from ipex_energy.energy_math import Channel, EnergyAccumulator


def make(stale_timeout_s=0.5):
    ch = Channel(name='a', group='g')
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


def test_reverse_driving_draws_power():
    # Wheel frame: driving backward = negative V and negative I.
    acc, ch = make()
    feed(acc, -10.0, -3.0, 0.0)
    assert math.isclose(ch.power_w, 30.0)


def test_braking_returns_power():
    # Driving forward (V > 0) while the motor brakes (I < 0) = regen.
    acc, ch = make()
    feed(acc, 10.0, -2.0, 0.0)
    assert math.isclose(ch.power_w, -20.0)


def test_drum_abs_current():
    # Drum channel: P = V * |I|; a negative idle reading counts as draw.
    ch = Channel(name='a', group='arm', abs_current=True)
    acc = EnergyAccumulator(channels=[ch])
    for k in range(11):
        feed(acc, 26.5, -0.08, k * 0.1)
    assert math.isclose(ch.power_w, 26.5 * 0.08)
    assert math.isclose(ch.energy_j, 26.5 * 0.08, rel_tol=1e-9)   # 1.0 s
    feed(acc, 26.5, 0.4, 1.1)    # drum spinning
    assert math.isclose(ch.power_w, 26.5 * 0.4)


def test_drive_channel_keeps_negative_current():
    # Default channel (drive): no floor, braking stays negative.
    acc, ch = make()
    feed(acc, 10.0, -0.5, 0.0)
    assert math.isclose(ch.power_w, -5.0)


def test_returned_energy_subtracts():
    acc, ch = make()
    for k in range(11):          # 1 s drawing 20 W
        feed(acc, 10.0, 2.0, k * 0.1)
    for k in range(11, 21):      # ramp to 1 s returning 20 W
        feed(acc, 10.0, -2.0, k * 0.1)
    # 20 J drawn, then a 0.1 s trapezoid through zero, then 0.9 s at -20 W.
    assert math.isclose(ch.energy_j, 20.0 + 0.0 - 18.0, abs_tol=1e-9)


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
    b = Channel(name='b', group='arm')
    acc = EnergyAccumulator(channels=[a, b])
    for name, v, i in (('a', 10.0, 1.0), ('b', 20.0, 2.0)):
        acc.set_voltage(name, v, 0.0)
        acc.set_current(name, i, 0.0)
    acc.step(0.0)
    totals = acc.group_totals()
    assert acc.groups() == ['drive', 'arm', 'total']
    assert math.isclose(totals['total'][0], 50.0)
