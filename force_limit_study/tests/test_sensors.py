"""Verify conversion timing, calibration and velocity tracking."""

import math

import numpy as np
import pytest

from host.axis_client.identifiers import State
from sim.params import ADC_MODELS, Params
from sim.sensors import Sensors
from sim.sil import Sim
from sim.trace import TraceColumn


@pytest.mark.parametrize("model", ADC_MODELS)
def test_conversion_window_and_saturation(model):
    """Check conversion window and saturation."""
    parameters = Params(vibration_rms=0, vibration_amp=0)
    sensor = Sensors(parameters, model)

    class Fixed:
        """Fixed."""

        force = 100.0
        carriage_velocity_m_s = 0.0
        carriage_position_m = 0.0

    ready = []
    for i in range(math.ceil((sensor.window + 3 / sensor.rate) / parameters.dt)):
        t = (i + 1) * parameters.dt
        if sensor.step(Fixed, t):
            ready.append(t)
    assert ready[0] == pytest.approx(
        sensor.window or 1 / sensor.rate, abs=parameters.dt
    )
    assert np.diff(ready) == pytest.approx(1 / sensor.rate, abs=parameters.dt)
    assert sensor.code / sensor.counts_per_N == pytest.approx(100, abs=0.2)
    Fixed.force = 1000000.0
    for i in range(math.ceil(2 * max(sensor.window, 1 / sensor.rate) / parameters.dt)):
        sensor.step(Fixed, t + (i + 1) * parameters.dt)
    assert sensor.code == 8388607


@pytest.mark.parametrize("model", ADC_MODELS)
def test_host_calibration_tare_and_stale_budget(model):
    """Check host calibration tare and stale budget."""
    simulation = Sim({"adc": model})
    simulation.prepare()
    assert simulation.firmware.debug.state == State.IDLE and (
        not simulation.firmware.debug.fault_flags
    )
    assert simulation.firmware.debug.tare_samples == 50
    assert simulation.firmware.debug.force_N == pytest.approx(0, abs=0.2)


def test_velocity_step():
    """Check velocity step."""
    simulation = Sim({"params": {"approach_speed_mm_s": 2, "accel_mm_s2": 3000}})
    simulation.prepare()
    simulation.client.start_insert(100)
    simulation.advance(6000, True)
    a = simulation.trace
    start = np.where(a[:, TraceColumn.REFERENCE_VELOCITY_MM_S] >= 1.999)[0][0]
    b = a[start:]
    assert a[start - 1, TraceColumn.REFERENCE_VELOCITY_MM_S] == 0
    steady = b[-200:, TraceColumn.MEASURED_VELOCITY_MM_S].mean()
    t10 = (
        b[np.where(b[:, TraceColumn.MEASURED_VELOCITY_MM_S] >= 2 * 0.1)[0][0], 0]
        - b[0, TraceColumn.TIME_S]
    )
    t90 = (
        b[np.where(b[:, TraceColumn.MEASURED_VELOCITY_MM_S] >= 2 * 0.9)[0][0], 0]
        - b[0, TraceColumn.TIME_S]
    )
    rise = t90 - t10
    bandwidth = 0.35 / rise
    print(
        f"VELOCITY_STEP steady={steady:.6f} mm/s, t10={t10:.3f} s, "
        f"t90={t90:.3f} s, rise={rise:.3f} s, "
        f"bandwidth_estimate={bandwidth:.3f} Hz"
    )
    assert steady == pytest.approx(2, abs=0.1) and 0 < rise < 6
