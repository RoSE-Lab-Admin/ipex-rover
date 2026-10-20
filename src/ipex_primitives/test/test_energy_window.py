import math

from ipex_primitives.energy_window import EnergyHistory, Sample, window_energy


def s(t, total, drive=0.0, arm=0.0, w=0.0):
    return Sample(t, total, {'drive': drive, 'front_arm': arm}, w)


def test_window_energy_and_above_idle():
    start, end = s(10.0, 100.0, 40.0, 60.0), s(20.0, 400.0, 240.0, 160.0)
    r = window_energy(start, end, duration_s=10.0, idle_power_w=10.0)
    assert r.energy_j == 300.0
    assert r.energy_above_idle_j == 200.0
    assert r.groups == {'drive': 200.0, 'front_arm': 100.0}


def test_idle_power_from_history_slope():
    h = EnergyHistory()
    for k in range(31):                    # 3 s at 10 Hz, 20 W constant
        h.add(s(k * 0.1, 20.0 * k * 0.1, w=20.0))
    assert math.isclose(h.idle_power(3.0, window_s=2.0), 20.0, rel_tol=1e-6)


def test_idle_power_falls_back_to_instantaneous():
    h = EnergyHistory()
    h.add(s(0.0, 0.0, w=12.5))
    assert h.idle_power(0.0) == 12.5


def test_counter_reset_clears_history():
    h = EnergyHistory()
    h.add(s(0.0, 500.0))
    h.add(s(0.1, 0.0))                     # meter reset
    assert len(h.samples) == 1


def test_missing_samples_give_nan():
    r = window_energy(None, s(1.0, 1.0), 1.0, 5.0)
    assert math.isnan(r.energy_j)
