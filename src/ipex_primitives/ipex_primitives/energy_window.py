"""Energy over a time window from cumulative energy samples (ROS-free).

Samples come from /ipex/energy (EnergyState): monotonic receive time plus
cumulative energy per group. Counters only increase, so the energy over a
window is a difference; idle power is the energy slope just before the
window started.
"""

import math
from collections import deque
from dataclasses import dataclass


@dataclass
class Sample:
    t: float               # monotonic seconds (receive time)
    total_j: float
    groups: dict           # {group: energy_j}, excluding 'total'
    total_w: float


class EnergyHistory:
    """Ring buffer of recent EnergyState samples."""

    def __init__(self, keep_s=30.0):
        self.keep_s = keep_s
        self.samples = deque()

    def add(self, sample):
        # A counter reset (energy went down) invalidates older history.
        if self.samples and sample.total_j + 1e-9 < self.samples[-1].total_j:
            self.samples.clear()
        self.samples.append(sample)
        while self.samples and sample.t - self.samples[0].t > self.keep_s:
            self.samples.popleft()

    def latest(self):
        return self.samples[-1] if self.samples else None

    def at_or_before(self, t):
        best = None
        for s in self.samples:
            if s.t <= t:
                best = s
            else:
                break
        return best

    def idle_power(self, now, window_s=2.0, min_span_s=0.5):
        """Average total power over [now - window_s, now]; falls back to the
        latest instantaneous power if there is not enough history."""
        latest = self.latest()
        if latest is None:
            return math.nan
        start = self.at_or_before(now - window_s) or (self.samples[0] if self.samples else None)
        if start is None or latest.t - start.t < min_span_s:
            return latest.total_w
        return (latest.total_j - start.total_j) / (latest.t - start.t)


@dataclass
class EnergyResult:
    energy_j: float
    energy_above_idle_j: float
    idle_power_w: float
    groups: dict


def window_energy(start, end, duration_s, idle_power_w):
    """Energy between two samples, plus energy above idle."""
    if start is None or end is None:
        return EnergyResult(math.nan, math.nan, idle_power_w, {})
    energy = end.total_j - start.total_j
    groups = {g: end.groups.get(g, 0.0) - start.groups.get(g, 0.0) for g in end.groups}
    above = energy - idle_power_w * duration_s if math.isfinite(idle_power_w) else math.nan
    return EnergyResult(energy, above, idle_power_w, groups)
