"""Assumed FSR 402 and the two team-board analog channels, without bridge noise."""

from dataclasses import dataclass
import math

import numpy as np

from .sensors import Sensors

# Assumed part: Interlink FSR 402; the team has not specified a part number.
# Official FSR 400 Series datasheet, PDS-10004-C, pp. 2-3 and Model 402 p. 6:
# https://www.interlinkelectronics.com/downloads/datasheets/fsr-400-series-datasheet.pdf
# Approximate manual digitisation of the typical log-log graph, converted from
# grams with assumed standard gravity 9.81 m/s^2: (force N, resistance ohm).
# Points span 0.3-20 N within the stated 0.2-20 N range, above the knee.
CURVE_POINTS = ((0.3, 62000), (0.5, 29000), (1, 15000), (2, 8500),
                (5, 4300), (10, 2550), (20, 1500))
_slope, _intercept = np.polyfit(np.log(np.array(CURVE_POINTS)[:, 0]),
                             np.log(np.array(CURVE_POINTS)[:, 1]), 1)
FSR_A, FSR_B = float(np.exp(_intercept)), float(-_slope)
ACTUATION_N = 0.2  # Datasheet typical minimum actuation force.
OFF_OHM = 10e6  # Assumed finite open approximation; datasheet specifies >10 MOhm.
VREF = 3.3  # Assumed VDDA = VREF+ = sensor excitation, V.
FULL_SCALE = 4095  # Requested unsigned 12-bit ADC.


@dataclass(frozen=True)
class FrontEnd:
    pulldown_ohm: float
    series_ohm: float
    capacitance_F: float


# Values supplied from the team's PCB revisions. Previous Q4 is assumed on,
# with negligible on resistance and a 3.3 V source; PE14/FSR_SIG is not switched.
FRONT_ENDS = {"now": FrontEnd(1e6, 0, 10e-9),
              "prev": FrontEnd(1e6, 10e3, 100e-9),
              # Not on any board: the current circuit with R26 changed to 2.2 kOhm, near the
              # geometric mean of the FSR's resistance at 5 N and 35 N, where the limits sit.
              "fix": FrontEnd(2.2e3, 0, 10e-9)}


def resistance(force_N):
    # Hysteresis, drift and part/actuator variation are ignored. Above 20 N this
    # is an assumed power-law extrapolation, not a datasheet-qualified response.
    return OFF_OHM if force_N < ACTUATION_N else FSR_A * force_N ** -FSR_B


def static_channel(force_N, revision="now"):
    """Return DC transfer and local differential N/LSB (before quantisation)."""
    circuit = FRONT_ENDS[revision]
    r = resistance(force_N)
    pd = circuit.pulldown_ohm
    volts = VREF * pd / (pd + r)
    code = FULL_SCALE * volts / VREF
    sensitivity = (force_N * (pd+r)**2 / (FULL_SCALE*pd*FSR_B*r)
                   if force_N >= ACTUATION_N else math.inf)
    return dict(fsr_ohm=r, node_volts=volts, ideal_code=code,
                N_per_LSB=sensitivity,
                tau_ms=(circuit.series_ohm + pd*r/(pd+r))*circuit.capacitance_F*1000)


class FSRSensors:
    """Exact held-force RC update at plant rate, then one ADC sample per tick."""

    rate = 1000  # Requested one conversion per 1 ms firmware tick.
    window = 0
    counts_per_N = 1  # Unused bridge calibration, retained for the SIL interface.

    def __init__(self, parameters, revision="now"):
        self.parameters, self.revision = parameters, revision
        self.voltage = 0.0  # Assumed initially discharged capacitor at power-on.
        self.next_ready = 1 / self.rate
        self.code = 0
        self.codes = []
        self.rng = np.random.default_rng(parameters.seed)

    # Use the unchanged local encoder mapping for both arm and rack fixtures.
    encoder = Sensors.encoder

    def advance_voltage(self, force_N, dt):
        channel = static_channel(force_N, self.revision)
        decay = math.exp(-dt / (channel["tau_ms"] / 1000))
        self.voltage = channel["node_volts"] + (self.voltage-channel["node_volts"])*decay
        return self.voltage

    def step(self, plant, time_s, stop=False, full=False):
        self.advance_voltage(plant.force, self.parameters.dt)
        if time_s + 1e-9 < self.next_ready:
            return False
        self.next_ready += 1 / self.rate
        if stop:
            return False
        # Assumed 1.5 LSB RMS Gaussian ADC noise; no bridge vibration is applied.
        # Ignore sample-and-hold loading: an assumed order-5 pF sampling capacitor
        # is ~5e-4 of C32 (10 nF), ~5e-5 of C8 (100 nF), per full-scale acquisition.
        self.code = FULL_SCALE if full else max(0, min(FULL_SCALE, round(
            FULL_SCALE*self.voltage/VREF + self.rng.normal(0, 1.5))))
        self.codes.append(self.code)
        return True
