"""Seeded analog bridge, moving-window conversion, signed 24-bit saturation."""

import math
from collections import deque

import numpy as np

from .params import ADC_MODELS


class Sensors:
    """Generate seeded bridge conversions and quantized screw encoder counts."""

    def __init__(self, parameters, model="ADS1220_1000"):
        """Initialize the instance."""
        self.parameters = parameters
        self.model = model
        self.rate, self.periods, self.noise, self.full_scale = ADC_MODELS[model]
        self.window = self.periods / self.rate
        self.samples = deque()
        self.total = 0.0
        self.next_ready = self.window or 1 / self.rate
        self.rng = np.random.default_rng(parameters.seed)
        self.code = 0
        self.volts_per_N = (
            parameters.cell_mv_v
            * 0.001
            * parameters.excitation
            / parameters.cell_rating
        )
        self.counts_per_N = 2**23 * self.volts_per_N / self.full_scale

    def encoder(self, plant):
        """Quantize carriage displacement to quadrature encoder counts."""
        if hasattr(plant, "counts_per_m"):
            return round(plant.carriage_position_m * plant.counts_per_m)
        return round(
            plant.carriage_position_m
            / self.parameters.lead
            * self.parameters.encoder_lines
            * self.parameters.quadrature
        )

    def step(self, plant, time_s, stop=False, full=False):
        """Advance one simulation or control tick."""
        parameters = self.parameters
        if self.model == "IDEAL_1000":
            # No structural/electrical noise or averaging; only the wire ADC quantization.
            if time_s + 1e-9 < self.next_ready:
                return False
            self.next_ready += 1 / self.rate
            if stop:
                return False
            self.code = (
                8388607
                if full
                else max(-8388608, min(8388607, round(plant.force * self.counts_per_N)))
            )
            return True
        vibration = (
            self.rng.normal(0, parameters.vibration_rms)
            + parameters.vibration_amp
            * math.sin(2 * math.pi * parameters.vibration_hz * time_s)
            if abs(plant.carriage_velocity_m_s) > 0.001
            else 0
        )
        bridge_voltage_V = (plant.force + vibration) * self.volts_per_N
        self.samples.append((time_s, bridge_voltage_V))
        self.total += bridge_voltage_V
        while self.samples and self.samples[0][0] < time_s - self.window + 1e-09:
            self.total -= self.samples.popleft()[1]
        if time_s + 1e-09 < self.next_ready:
            return False
        self.next_ready += 1 / self.rate
        if stop:
            return False
        conversion_voltage_V = self.total / len(self.samples) + self.rng.normal(
            0, self.noise
        )
        self.code = (
            8388607
            if full
            else max(
                -8388608,
                min(8388607, round(conversion_voltage_V / self.full_scale * 2**23)),
            )
        )
        return True
