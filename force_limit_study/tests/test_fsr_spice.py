"""Schematic, analog sampling, state carry and DLL allocation regressions."""
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from board.extract_fsr import extract, spice_value, REVISIONS
from hw.spice import simulate_released
from sim.fsr import FSRSensors, static_channel, FULL_SCALE, VREF, resistance
from sim.fsr_spice import (SpiceFSRSensors, front_end, steady, process_memory,
                           C_ADC, PERIOD)
from sim.params import Params


@pytest.fixture(scope="module")
def engine():
    try:
        simulate_released("load probe\nV1 x 0 1\nR1 x 0 1k\n.op\n.end")
    except OSError as error:
        pytest.skip(str(error))


@pytest.mark.parametrize("revision", REVISIONS)
def test_regeneration(revision):
    root = os.environ.get("BOARD_KICAD_DIR")
    if not root:
        pytest.skip("set BOARD_KICAD_DIR to the exported revisions")
    assert extract(Path(root) / f"rev_{REVISIONS[revision]}") == front_end(revision)


def test_committed_parts_and_units():
    def parts(revision):
        return {s[0]: (s[1:3], spice_value(s[3])) for line in front_end(revision).splitlines()
                if line.startswith(("R", "C")) for s in [line.split()]}
    now, prev, fix = (parts(r) for r in ("now", "prev", "fix"))
    assert set(now) == {"R26", "C32"}
    assert now["R26"] == (["adc", "gnd"], 1e6)
    assert now["C32"][1] == pytest.approx(10e-9)
    assert set(prev) == {"R29", "R36", "R28", "C8"}
    assert prev["R29"] == (["sensor_a", "gnd"], 1e6)
    assert prev["R36"] == (["adc", "sensor_a"], 1e4)
    assert prev["R28"] == (["gate", "v3v3"], 1e4)
    assert prev["C8"][1] == pytest.approx(100e-9)
    assert "SQ4 v3v3 sensor_b v3v3 gate Q4_AO3401A" in front_end("prev")
    assert "Ron=0.085" in front_end("prev")
    assert fix["R26"][1] == 2200
    assert spice_value("1M") == .001


@pytest.mark.parametrize("revision", ("now", "prev", "fix"))
@pytest.mark.parametrize("force", (5, 15, 25, 35))
def test_dc(engine, revision, force):
    sensor = SpiceFSRSensors(Params(), revision, sampling=False)
    assert sensor.dc(force) == pytest.approx(static_channel(force, revision)["node_volts"], abs=1e-4)


def test_time_constant_and_carry(engine):
    sensor = SpiceFSRSensors(Params(), sampling=False)
    force = 5
    tau = static_channel(force)["tau_ms"] / 1000
    result = simulate_released(sensor.circuit([force]))
    target = sensor.dc(force) * (1 - np.exp(-1))
    measured = np.interp(target, result["adc"], result["time"])
    assert measured == pytest.approx(tau, rel=.05)
    sensor.period([force])
    first = sensor.voltage
    sensor.period([force])
    assert sensor.voltage == pytest.approx(sensor.dc(force)*(1-np.exp(-2*PERIOD/tau)), abs=1e-4)
    assert sensor.voltage > first


@pytest.mark.parametrize("force", (0, 5, 35))
def test_charge_loading(engine, force):
    p = Params()
    for revision in ("now", "fix"):
        dc = SpiceFSRSensors(p, revision).dc(force)
        previous = steady(p, force, revision)
        ground = steady(p, force, revision, sh_start="ground")
        assert abs(dc-previous)*FULL_SCALE/VREF < 1
        if revision == "fix":
            assert abs(dc-ground)*FULL_SCALE/VREF < 1
        elif force == 0:
            # Average charge loss Rth*f*C_ADC*V; end-of-window ripple is about
            # 5% at the off-state 9 ms time constant (assumed 30% tolerance).
            rth = 1 / (1/resistance(force) + 1/1e6)
            assert dc-ground == pytest.approx(rth/PERIOD*C_ADC*dc, rel=.30)


@pytest.mark.parametrize("revision", ("now", "prev", "fix"))
@pytest.mark.parametrize("force", (0, 5, 35))
def test_switch_resistance_insensitivity(engine, revision, force):
    for mode in ("previous", "ground"):
        a = steady(Params(), force, revision, sh_start=mode, r_adc=500)
        b = steady(Params(), force, revision, sh_start=mode, r_adc=6000)
        assert abs(a-b)*FULL_SCALE/VREF < 1


def test_rng_and_tick_order(engine):
    p = Params()
    formula, spice = FSRSensors(p), SpiceFSRSensors(p)
    plant = SimpleNamespace(force=5, carriage_position_m=0, counts_per_m=1)
    for tick in range(1, 31):
        for substep in range(1, 11):
            t = (tick-1+substep/10)*PERIOD
            assert formula.step(plant, t, stop=tick == 12, full=tick == 20) == spice.step(
                plant, t, stop=tick == 12, full=tick == 20)
    assert len(spice.codes) == 29
    assert formula.rng.normal() == spice.rng.normal()
    assert spice.codes[18] == 4095


def test_open_sensor_and_leakage(engine):
    sensor = SpiceFSRSensors(Params())
    assert sensor.dc(float("inf")) == pytest.approx(0, abs=1e-8)
    loaded = SpiceFSRSensors(Params(), leakage=True)
    assert sensor.dc(0)-loaded.dc(0) == pytest.approx(100e-9/(1/1e6+1/resistance(0)), rel=1e-4)


def test_memory_2000_periods(engine):
    sensor = SpiceFSRSensors(Params())
    for _ in range(100):
        sensor.period([5]*10)
    before = process_memory()
    for _ in range(2000):
        sensor.period([5]*10)
    after = process_memory()
    # Requested few-MB bound, assumed 5 MiB allowance for allocator page granularity.
    assert before["private_bytes"] > 0
    assert after["private_bytes"] - before["private_bytes"] < 5*1024**2
    assert after["working_set"] - before["working_set"] < 5*1024**2
