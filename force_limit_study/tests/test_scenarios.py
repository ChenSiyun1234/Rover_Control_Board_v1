"""Verify nominal seating, jam unloading and injected safety faults."""

import subprocess
import sys

import pytest

from host.axis_client.identifiers import State
from sim.sil import Sim
from sim.trace import TraceColumn


@pytest.fixture(scope="module")
def jam_pair():
    """Jam pair."""
    runs = []
    for envelope in (1, 0):
        simulation = Sim({"jam": True, "params": {"envelope_enabled": envelope}})
        result = simulation.run_insert()
        simulation.advance(1500, True)
        result.update(
            peak_after_braking_N=simulation.trace[:, TraceColumn.TRUE_FORCE_N].max(),
            unloaded_force_N=simulation.plant.force,
            carriage_after_mm=simulation.plant.carriage_position_m * 1000,
        )
        print("JAM", envelope, result)
        runs.append(result)
    return runs


def test_nominal_insertion_seats_within_position_and_force_bounds():
    """Check nominal. Specification 15."""
    simulation = Sim()
    result = simulation.run_insert()
    print("NOMINAL", result)
    assert result["final_state"] == "SEATED" and (not result["fault_flags"])
    assert 293 <= result["seat_position_mm"] <= 297
    assert result["peak_force_N"] < 170


def test_position_envelope_jam_trips_and_unloads(jam_pair):
    """Check jam envelope. Specification 16."""
    result = jam_pair[0]
    assert result["final_state"] == "FAULT" and result["fault_flags"] & 1
    assert 60 < result["detection_force_N"] < 80
    assert result["unloaded_force_N"] < 10
    assert result["carriage_after_mm"] < 150


def test_global_threshold_jam_trips_later_with_higher_peak(jam_pair):
    """Check global threshold. Specification 17."""
    a, b = jam_pair
    assert b["fault_flags"] & 1 and b["fault_ms"] > a["fault_ms"]
    assert b["detection_force_N"] > a["detection_force_N"]
    assert b["peak_after_braking_N"] > a["peak_after_braking_N"]


def moving(config):
    """Moving."""
    simulation = Sim(config)
    simulation.prepare()
    simulation.client.start_insert(296)
    simulation.advance(500, True)
    assert simulation.firmware.debug.state == State.MOVING
    return simulation


def test_stale_sensor_fault_brakes_and_requires_fresh_data_to_clear():
    """Check stale. Specification 18."""
    simulation = moving({})
    t = simulation.now_ms
    simulation.config["adc_stop_ms"] = t
    simulation.advance(8, True)
    debug = simulation.firmware.debug
    assert (
        debug.state == State.FAULT
        and debug.fault_flags & 2
        and (0 < debug.fault_ms - t <= 7)
    )
    simulation.advance(500, True)
    assert (
        abs(simulation.plant.carriage_velocity_m_s) < 0.002
        and simulation.firmware.debug.velocity_ref == 0
    )
    simulation.client.clear_fault()
    simulation.advance(5)
    assert simulation.firmware.debug.state == State.FAULT
    simulation.config.pop("adc_stop_ms")
    simulation.advance(10)
    simulation.client.clear_fault()
    simulation.advance(5)
    assert simulation.firmware.debug.state == State.IDLE


def test_saturated_sensor_latches_range_fault():
    """Check range. Specification 19."""
    simulation = moving({})
    simulation.config["adc_full_ms"] = simulation.now_ms
    simulation.advance(3, True)
    assert (
        simulation.firmware.debug.state == State.FAULT
        and simulation.firmware.debug.fault_flags & 4
    )


def test_silent_host_latches_communication_fault():
    """Check sil watchdog. Specification 20."""
    simulation = moving({})
    t = simulation.now_ms
    simulation.config["host_silent_ms"] = t
    simulation.advance(230, True)
    assert (
        simulation.firmware.debug.state == State.FAULT
        and simulation.firmware.debug.fault_flags & 8
    )


def test_estop_removes_drive_and_requires_explicit_clear():
    """Check estop. Specification 21."""
    simulation = moving({})
    simulation.config["estop_ms"] = simulation.now_ms + 1
    simulation.step()
    assert (
        simulation.firmware.debug.state == State.ESTOP
        and (not simulation.firmware.out.drive_enable)
        and (simulation.firmware.out.current_cmd_A == 0)
    )
    simulation.read()
    simulation.client.start_insert()
    simulation.advance(5)
    replies = list(simulation.client.frames())
    assert any((f.cmd == 127 and f.payload == b"\x10\x04" for f in replies))
    simulation.client.clear_fault()
    simulation.advance(5)
    assert simulation.firmware.debug.state == State.ESTOP
    simulation.config.pop("estop_ms")
    simulation.advance(5)
    simulation.client.start_insert()
    simulation.advance(5)
    assert simulation.firmware.debug.state == State.ESTOP
    simulation.client.clear_fault()
    simulation.advance(5)
    assert simulation.firmware.debug.state == State.IDLE


def test_same_firmware_sources_compile_for_cortex_m4():
    """Check arm compile. Specification 22."""
    result = subprocess.run(
        [sys.executable, "build.py", "--arm-only"], text=True, capture_output=True
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ARM compile check passed" in result.stdout
