"""Fast circuit, native conversion and selected 3-D integration checks."""

import math

import numpy as np
import pytest

from host.axis_client import Client, Reader
from host.axis_client.identifiers import Command, State
from host.axis_client.parameters import PARAMETER_IDS
from sim.fsr import FSR_A, FSR_B, FRONT_ENDS, FSRSensors, static_channel
from sim.params import Params
from sim.sil import Firmware, Sim
from sim3d.cases import FSR_CASES, run_case
from sim3d.mj_plant import MjPlant
from sim3d.scene import ARM


@pytest.mark.parametrize("revision", FRONT_ENDS)
def test_static_transfer_is_monotonic(revision):
    channels = [static_channel(f, revision) for f in (0, .1, .2, .5, 1, 2, 5, 10, 20, 35, 50)]
    assert np.all(np.diff([c["fsr_ohm"] for c in channels]) <= 0)
    assert np.all(np.diff([c["node_volts"] for c in channels]) >= 0)
    assert np.all(np.diff([c["ideal_code"] for c in channels]) >= 0)
    assert all(0 <= c["ideal_code"] <= 4095 for c in channels)


@pytest.mark.parametrize("revision", FRONT_ENDS)
@pytest.mark.parametrize("force", [.5, 10])
def test_exact_rc_step(revision, force):
    sensor = FSRSensors(Params(), revision)
    channel = static_channel(force, revision)
    for _ in range(13):
        sensor.advance_voltage(force, 1e-4)
    expected = channel["node_volts"] * (1-math.exp(-.0013/(channel["tau_ms"]/1000)))
    assert sensor.voltage == pytest.approx(expected, rel=1e-12)
    pd = FRONT_ENDS[revision].pulldown_ohm
    r = channel["fsr_ohm"]
    assert channel["node_volts"] == pytest.approx(3.3*pd/(pd+r))
    assert channel["tau_ms"] == pytest.approx(
        (pd*r/(pd+r)+FRONT_ENDS[revision].series_ohm)*FRONT_ENDS[revision].capacitance_F*1000)


def set_fsr(firmware, pulldown):
    client = Client(firmware)
    for key, value in dict(force_sensor=1, fsr_pulldown_ohm=pulldown,
                           fsr_a=FSR_A, fsr_b=FSR_B).items():
        client.set_param(PARAMETER_IDS[key], value)
        assert Reader().feed(firmware.read())[0].cmd == Command.SET_PARAM | 0x80
    return client


# Both boards have 1 MOhm pull-downs. 10 kOhm is an assumed alternate calibration
# used only to exercise the parameter, not a modification to either board model.
@pytest.mark.parametrize("pulldown", [1e6, 1e4])
@pytest.mark.parametrize("force", [.5, 2, 5, 15])
def test_dll_fsr_inverse(pulldown, force):
    firmware = Firmware()
    set_fsr(firmware, pulldown)
    r = FSR_A*force**-FSR_B
    code = round(4095*pulldown/(pulldown+r))
    firmware.step(1, adc_code=code)
    # Tolerance: one local differential LSB, or 0.002 N for float arithmetic.
    per_lsb = force*(pulldown+r)**2/(4095*pulldown*FSR_B*r)
    assert firmware.debug.force_raw_N == pytest.approx(force, abs=max(.002, per_lsb))


def test_dll_zero_and_full_scale_are_finite():
    firmware = Firmware()
    set_fsr(firmware, 1e6)
    firmware.step(1, adc_code=0)
    assert firmware.debug.force_raw_N == 0
    firmware.step(2, adc_code=4095)
    maximum = (FSR_A*4094/1e6)**(1/FSR_B)
    assert firmware.debug.force_raw_N == pytest.approx(maximum, rel=.001)
    firmware.step(3, adc_code=4094)
    assert firmware.debug.force_raw_N == pytest.approx(maximum, rel=.001)


@pytest.mark.parametrize("revision", FRONT_ENDS)
def test_zero_force_tare_and_adc_tick(revision):
    from functools import partial
    sim = Sim(dict(plant_factory=partial(MjPlant, case="arm"),
                   sensor_factory=partial(FSRSensors, revision=revision),
                   params=dict(force_sensor=1, fsr_a=FSR_A, fsr_b=FSR_B,
                               counts_per_mm=ARM.counts_per_m/1000)))
    sim.prepare()
    sim.advance(300, True)  # Assumed settling observation, >3 off-state time constants.
    assert sim.firmware.debug.tare_samples == 50
    assert abs(sim.firmware.debug.force_N) < .01
    assert sim.firmware.debug.state == State.IDLE
    assert len(sim.sensors.codes) == sim.now_ms
    assert all(0 <= code <= 4095 for code in sim.sensors.codes)


@pytest.mark.parametrize("name", FSR_CASES)
def test_fsr_cases_have_terminal_or_reported_timeout(name):
    sim, summary = run_case(name)
    assert summary["outcome"] in ("seated", "tripped", "time_out")
    assert np.isfinite(sim.trace).all()
    assert summary["true_peak_force_N"] >= 0
    if summary["outcome"] == "time_out":
        assert summary["timed_out"] and summary["cycle_time_s"] == 12
        assert summary["timeout_reason"]
    else:
        assert not summary["timed_out"]
        assert summary["final_state"] in ("SEATED", "FAULT")
    if summary["outcome"] == "tripped":
        assert summary["firmware_force_at_trip_N"] == summary["detection_force_N"]
        assert sim.now_ms >= summary["fault_ms"] + 1500
        assert summary["backoff_mm"] is not None
    assert sim.plant.counts_per_m/1000 == pytest.approx(summary["counts_per_mm"])
    assert summary["velocity_filter_hz"] == (5 if "lowpass" in name else 80)
