"""Fast checks for native PWM encoding, armature physics and wiring protection."""

import ctypes as C

import pytest

from host.axis_client import Client, Reader
from host.axis_client.identifiers import Fault, State
from host.axis_client.parameters import PARAMETER_IDS
from sim.sil import Firmware
from sim3d.cases import run_case
from sim3d.mj_plant import DCMotor


def ready(mode, **parameters):
    firmware = Firmware()
    client = Client(firmware)
    for name, value in dict(drive_mode=mode, **parameters).items():
        client.set_param(PARAMETER_IDS[name], value)
        assert Reader().feed(firmware.read())[0].cmd == 192
    client.enable()
    firmware.read()
    for tick in range(1, 61):
        firmware.step(tick)
    client.start_insert(100)
    firmware.read()
    firmware.lib.fw_probe_reference.argtypes = [C.c_float]
    return firmware


@pytest.mark.parametrize("reference", [-1000, -10, 0, 10, 1000])
def test_native_duty_dir_round_trip(reference):
    firmware = ready(1, duty_kp=.01, duty_ki=0)
    firmware.lib.fw_probe_reference(reference)
    out = firmware.step(61)
    decoded = out.pwm_duty*(1 if out.dir else -1)
    assert decoded == pytest.approx(max(-1, min(1, .01*reference)))
    assert 0 <= out.pwm_duty <= 1 and out.dir in (0, 1)
    assert out.current_cmd_A == 0
    assert out.dir == (1 if reference >= 0 else 0)


@pytest.mark.parametrize("direction", [0, 1])
def test_full_duty_stall_current(direction):
    current = 0
    for _ in range(1000):
        current = DCMotor.advance(current, 1, direction, 0, .0001)
    expected = DCMotor.bus_V/DCMotor.resistance_ohm*(1 if direction else -1)
    assert current == pytest.approx(expected, rel=1e-12)
    assert abs(current) > 13  # The MD13S continuous rating is not a current clamp.


@pytest.mark.parametrize("mode", [0, 1])
def test_wiring_persistence_reset_and_fault_in_both_modes(mode):
    firmware = ready(mode)
    firmware.lib.fw_probe_reference(20)
    # Default encoder scale: -10 mm/s, well beyond the 2 mm/s threshold.
    for tick in range(61, 101):
        firmware.step(tick, encoder_counts=round(-(tick-60)*.01*819.2))
    assert not firmware.debug.fault_flags
    # Removing the reference must reset the persistence window.
    firmware.lib.fw_probe_reference(0)
    firmware.step(101, encoder_counts=-336)
    firmware.lib.fw_probe_reference(20)
    for tick in range(102, 152):
        firmware.step(tick, encoder_counts=round(-(tick-60)*.01*819.2))
    assert not firmware.debug.fault_flags
    for tick in range(152, 156):
        firmware.step(tick, encoder_counts=round(-(tick-60)*.01*819.2))
    assert firmware.debug.fault_flags == Fault.WIRING
    assert firmware.debug.state == State.FAULT
    assert firmware.out.pwm_duty == firmware.out.current_cmd_A == 0


@pytest.mark.parametrize("name, expected", [("wiring_ok", 0),
                                             ("wiring_motor_swapped", Fault.WIRING),
                                             ("wiring_encoder_swapped", Fault.WIRING),
                                             ("wiring_both_swapped", 0)])
def test_arm_wiring_cases(name, expected):
    _, summary = run_case(name)
    assert summary["fault_flags"] == expected, summary
    if expected:
        assert summary["motion_to_trip_ms"] > 50
    if name == "wiring_ok":
        assert summary["final_state"] == "SEATED"
        assert summary["wrong_way_mm"] == 0
    if name == "wiring_both_swapped":
        assert summary["wrong_way_mm"] > 0


def test_current_mode_arm_regression():
    _, summary = run_case("arm_nominal_fsr_fix")
    # Captured with the arm pose read off the team's CAD picture (78 deg, +18 deg) and
    # the 2.2 kOhm input; the PWM drive work must not change the current-mode result.
    assert summary["peak_force_N"] == pytest.approx(25.083348003320907, abs=1e-10)
    assert summary["seat_position_mm"] == pytest.approx(40.034143786736706, abs=1e-10)
    assert summary["cycle_time_s"] == 8.252
    assert summary["fault_flags"] == 0
