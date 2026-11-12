"""Seeded force-sample transport with fixed latency and independent frame loss."""

from collections import deque
import random


class BusSensors:
    """Delay ADC codes and ready events while forwarding encoder and calibration."""

    def __init__(self, sensor, delay_ms=0, drop=0, seed=7):
        # Requested defaults: direct transport and independent loss with seed 7.
        if delay_ms < 0 or not 0 <= drop <= 1:
            raise ValueError("delay_ms must be nonnegative and drop must be in [0, 1]")
        self.sensor = sensor
        self.delay_s = delay_ms / 1000
        self.drop = drop
        self.rng = random.Random(seed)
        self.pending = deque()
        self.code = sensor.code

    def __getattr__(self, name):
        return getattr(self.sensor, name)

    def encoder(self, plant):
        """Read the local encoder without bus delay or loss."""
        return self.sensor.encoder(plant)

    def step(self, plant, time_s, stop=False, full=False):
        """Queue each surviving conversion and release it at its arrival time."""
        if self.sensor.step(plant, time_s, stop, full):
            if self.rng.random() >= self.drop:
                self.pending.append((time_s + self.delay_s, self.sensor.code))
        ready = False
        # Match the sensors' 1 ns tolerance for floating-point sample boundaries.
        while self.pending and self.pending[0][0] <= time_s + 1e-9:
            _, self.code = self.pending.popleft()
            ready = True
        return ready
