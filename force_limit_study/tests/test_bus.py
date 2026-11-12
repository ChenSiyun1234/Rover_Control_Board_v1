"""Fast bus transport checks and exact direct-sensor case regressions."""

from types import SimpleNamespace

import numpy as np
import pytest

from host.axis_client.identifiers import Fault
from sim.bus import BusSensors
from sim.params import Params
from sim.sensors import Sensors
from sim.sil import Sim
from sim3d.cases import run_case


@pytest.mark.parametrize("family,base", [("arm", "arm_jam_fsr_fix"), ("rack", "rack_jam")])
def test_zero_bus_matches_unwrapped(family, base):
    direct, expected = run_case(base)
    expected_trace = direct.trace.copy()
    wrapped, actual = run_case(f"bus_{family}_0ms")
    assert actual == expected
    np.testing.assert_array_equal(wrapped.trace, expected_trace)


def test_first_nonzero_sample_is_delayed_ten_ms():
    plant = SimpleNamespace(force=1, carriage_position_m=0)
    direct = Sensors(Params(), "IDEAL_1000")
    bus = BusSensors(Sensors(Params(), "IDEAL_1000"), delay_ms=10)
    first_direct = first_bus = None
    # Assumed synthetic observation: 25 ms at the SIL's 0.1 ms plant cadence.
    for tick in range(1, 251):
        time_s = tick / 10000
        direct_ready = direct.step(plant, time_s)
        ready = bus.step(plant, time_s)
        if direct_ready and direct.code and first_direct is None:
            first_direct = time_s
        if first_bus is None and not ready:
            assert bus.code == 0
        if ready and bus.code and first_bus is None:
            first_bus = time_s
        assert bus.encoder(plant) == direct.encoder(plant)
    assert first_direct == pytest.approx(.001)
    assert first_bus - first_direct == pytest.approx(.010)
    assert bus.rate == direct.rate and bus.window == direct.window
    assert bus.counts_per_N == direct.counts_per_N


def loss_sequence(drop, seed=7):
    sensor = Sensors(Params(), "IDEAL_1000")
    bus = BusSensors(sensor, drop=drop, seed=seed)
    plant = SimpleNamespace(force=0)
    arrivals = []
    # Assumed statistical budget: 10,000 samples gives a useful loss-rate check.
    for tick in range(1, 10001):
        plant.force = tick
        previous = bus.code
        ready = bus.step(plant, tick / 1000)
        arrivals.append(ready)
        assert bus.code == (sensor.code if ready else previous)
    return arrivals


def test_loss_fraction_and_seed():
    sequence = loss_sequence(.05)
    assert sequence == loss_sequence(.05)
    assert sequence != loss_sequence(.05, seed=8)
    # Assumed tolerance: 1 percentage point, >4 standard deviations at 5% loss.
    assert 1 - np.mean(sequence) == pytest.approx(.05, abs=.01)


def test_all_dropped_samples_never_arrive():
    assert not any(loss_sequence(1))


def test_pending_sample_arrives_after_source_stops():
    plant = SimpleNamespace(force=1)
    bus = BusSensors(Sensors(Params(), "IDEAL_1000"), delay_ms=10)
    assert not bus.step(plant, .001)
    assert not bus.step(plant, .010, stop=True)
    assert bus.step(plant, .011, stop=True)
    held = bus.code
    assert held > 0
    assert not bus.step(plant, .012, stop=True)
    assert bus.code == held


def test_bus_keeps_existing_stale_timeout():
    # Assumed source failure at 200 ms, after normal tare and motion start.
    sim = Sim(dict(adc_stop_ms=200, sensor_factory=lambda p: BusSensors(
        Sensors(p), delay_ms=10)))
    sim.prepare()
    sim.client.start_insert(296)
    sim.advance(150, True)  # Assumed observation extends past source failure and drain.
    assert sim.firmware.debug.fault_flags == Fault.SENSOR_STALE
