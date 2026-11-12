"""ROS-free power and energy accounting (unit-testable without ROS).

Power per channel: P = V * I. Positive = power drawn, negative = power
returned (drive motors braking or going downhill). The drivetrain firmware
sets every sign so this holds:
  - Drive motors: V = battery V x PWM duty and I = motor current, both
    signed in the wheel frame (+ = pushing the rover forward).
  - Arm supplies: V = supply voltage, I = supply current, + when drawing.

Energy is integrated with the trapezoid rule between steps. A channel whose
voltage or current is missing, NaN, or older than the stale timeout counts
as 0 W and is flagged stale.
"""

import math
from dataclasses import dataclass, field


GROUP_TOTAL = 'total'


@dataclass
class Channel:
    name: str
    group: str
    voltage: float = math.nan
    current: float = math.nan
    voltage_time: float = -math.inf
    current_time: float = -math.inf
    power_w: float = 0.0
    energy_j: float = 0.0
    stale: bool = True


@dataclass
class EnergyAccumulator:
    channels: list
    stale_timeout_s: float = 0.5
    start_time: float = None
    last_time: float = None
    _by_name: dict = field(default_factory=dict, init=False)

    def __post_init__(self):
        self._by_name = {c.name: c for c in self.channels}

    # ----------------------------------------------------------------- inputs

    def set_voltage(self, name, value, now):
        ch = self._by_name[name]
        ch.voltage = float(value)
        ch.voltage_time = now

    def set_current(self, name, value, now):
        ch = self._by_name[name]
        ch.current = float(value)
        ch.current_time = now

    # ------------------------------------------------------------- accounting

    def _power(self, ch, now):
        fresh = (now - ch.voltage_time <= self.stale_timeout_s and
                 now - ch.current_time <= self.stale_timeout_s)
        valid = math.isfinite(ch.voltage) and math.isfinite(ch.current)
        if not (fresh and valid):
            return 0.0, True

        return ch.voltage * ch.current, False

    def step(self, now):
        """Recompute power and integrate energy up to time `now` (seconds)."""
        if self.start_time is None:
            self.start_time = now

        dt = 0.0 if self.last_time is None else max(now - self.last_time, 0.0)
        self.last_time = now

        for ch in self.channels:
            previous = ch.power_w
            ch.power_w, ch.stale = self._power(ch, now)
            ch.energy_j += 0.5 * (previous + ch.power_w) * dt

    def reset(self, now):
        """Zero all energy counters and restart elapsed time."""
        self.start_time = now
        self.last_time = now
        for ch in self.channels:
            ch.energy_j = 0.0

    # ---------------------------------------------------------------- outputs

    def elapsed(self, now):
        return 0.0 if self.start_time is None else now - self.start_time

    def groups(self):
        """Ordered group names, followed by 'total'."""
        names = []
        for ch in self.channels:
            if ch.group not in names:
                names.append(ch.group)
        return names + [GROUP_TOTAL]

    def group_totals(self):
        """{group: (power_w, energy_j)} including 'total'."""
        totals = {g: [0.0, 0.0] for g in self.groups()}
        for ch in self.channels:
            for g in (ch.group, GROUP_TOTAL):
                totals[g][0] += ch.power_w
                totals[g][1] += ch.energy_j
        return {g: (p, e) for g, (p, e) in totals.items()}
