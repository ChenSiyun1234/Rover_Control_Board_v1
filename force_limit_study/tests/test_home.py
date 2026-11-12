"""Limit homing checks with real arm physics and deterministic native tick inputs."""

import pytest

from host.axis_client import Client, Reader
from host.axis_client.identifiers import Command, Fault, ParameterId, State
from sim.sil import Firmware
from sim3d.cases import run_case, run_home


def native_home(mode):
    firmware = Firmware()
    client = Client(firmware)
    client.set_param(ParameterId.DRIVE_MODE, mode)
    firmware.read()
    client.enable()
    firmware.read()
    for tick in range(1, 61):
        firmware.step(tick)
    client.home()
    assert Reader().feed(firmware.read())[0].cmd == Command.HOME | 0x80
    return firmware, client


@pytest.mark.parametrize("mode", [0, 1])
def test_arm_home_seats_zeroes_and_clears_switch(mode):
    sim, summary = run_home("home_5", drive_mode=mode)
    assert summary["ok"], summary
    # Assumed acceptance tolerance 0.5 mm: allows debounce and PI braking travel
    # at 5 mm/s, while remaining below the assumed 1 mm switch clearance.
    assert abs(summary["zero_error_mm"]) < 0.5, summary
    assert summary["firmware_carriage_mm"] == pytest.approx(1, abs=1/summary["counts_per_mm"])
    assert not sim.plant.limit_switch
    assert summary["time_to_switch_s"] > 0


def test_double_swap_hits_home_travel_timeout():
    # Assumed test search bound 30 mm reaches timeout before the real drawer.
    # The default 60 mm case instead hits OVERFORCE at the drawer; retain that case.
    _, summary = run_home("home_both_swapped", home_max_travel_mm=30)
    assert summary["final_state"] == "FAULT", summary
    assert summary["fault_flags"] == Fault.HOME_TIMEOUT, summary
    assert summary["time_to_switch_s"] is None
    assert summary["carriage_mm"] > 0
    assert summary["zero_error_mm"] is None


@pytest.mark.parametrize("mode", [0, 1])
def test_short_bounce_resets_consecutive_tick_count_in_both_modes(mode):
    firmware, _ = native_home(mode)
    for tick, active in enumerate([True, True, False, True, True], 61):
        firmware.step(tick, limit_switch=active)
        assert firmware.debug.state == State.HOMING
        assert not firmware.debug.fault_flags
    firmware.step(66, limit_switch=True)
    assert firmware.debug.state == State.HOME_BACKOFF
    assert firmware.debug.carriage_mm == 0
    firmware.step(67, encoder_counts=819, limit_switch=False)
    # Let the velocity filter settle before checking backoff completion.
    for tick in range(68, 100):
        firmware.step(tick, encoder_counts=819)
    assert firmware.debug.state == State.IDLE


@pytest.mark.parametrize("mode", [0, 1])
def test_timeout_suppresses_drive_and_home_locks_parameters(mode):
    firmware, client = native_home(mode)
    client.set_param(ParameterId.HOME_SPEED_MM_S, 20)
    assert Reader().feed(firmware.read())[0].payload == bytes((Command.SET_PARAM, 4))
    firmware.step(61, encoder_counts=-50000)
    assert firmware.debug.fault_flags == Fault.HOME_TIMEOUT
    assert firmware.out.current_cmd_A == firmware.out.pwm_duty == 0


@pytest.mark.parametrize("command", ["stop", "disable"])
def test_home_can_be_aborted(command):
    firmware, client = native_home(1)
    getattr(client, command)()
    assert Reader().feed(firmware.read())[0].cmd != Command.NACK
    firmware.step(61)
    assert firmware.debug.state == (State.IDLE if command == "stop" else State.DISABLED)


def test_default_double_swap_reports_obstacle_fault_instead_of_home_timeout():
    _, summary = run_case("home_both_swapped")
    assert summary["final_state"] == "FAULT", summary
    assert summary["fault_flags"] == Fault.OVERFORCE, summary
    assert summary["time_to_switch_s"] is None
    assert summary["zero_error_mm"] is None
