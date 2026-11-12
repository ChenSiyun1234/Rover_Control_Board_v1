"""Requirement checks use fresh ngspice runs, never generated results files."""

import pytest

from hw.brake.run import run
from hw.brake.run import run_corners as brake_corners
from hw.io24.run import run_corners, transient


def test_type3_thresholds_currents_and_sensor_leakage():
    rows, voltage, envelope = run_corners()
    assert len(rows) == 128
    assert max(r["on"] for r in rows) <= 11
    assert min(r["off"] for r in rows) >= 5
    assert min(r["i11"] for r in rows) >= 0.002
    assert max(r["i30"] for r in rows) < 0.015
    assert all(r["leakage_out"] >= 2.31 for r in rows)
    region = (voltage >= 11) & (voltage <= 30)
    assert envelope[:, region].min() >= 0.002
    assert envelope[:, region].max() <= 0.015


def test_reverse_polarity_and_continuous_power():
    rows, *_ = run_corners()
    for r in rows:
        assert abs(r["reverse_led"]) < 1e-6
        assert abs(r["reverse_i"]) < 5e-6
        assert abs(r["reverse_d1"]) < 1000
        assert abs(r["reverse_led_v"]) < 5
        assert abs(r["reverse_z"]) < 0.8
        assert r["reverse_out"] < 80
        assert all(r[k] < 0.5 * 0.5 for k in ("r1_w", "r2_w", "r3_w", "r4_w"))
        assert r["z_w"] < 0.5


def test_brake_voltage_clamp_energy_and_speed():
    rows = brake_corners()
    for r in rows:
        assert r["peak_v"] < 0.8 * 100
        assert 0 < r["energy_j"] < 0.15
        assert r["rg_w"] < 0.5 * 0.5
        assert r["rpd_w"] < 0.5 * 0.5
        if r["kind"] != "a":
            # Conservative 100 W / 10 ms single-pulse envelope, not 600 W at all widths.
            assert r["clamp_peak_w"] < 100
    for supply in (21.6, 24, 26.4):
        groups = {
            k: [r["delay_s"] for r in rows if r["supply"] == supply and r["kind"] == k]
            for k in "abc"
        }
        assert max(groups["b"]) < min(groups["a"])
        assert max(groups["c"]) < min(groups["a"])


@pytest.mark.parametrize("kind", list("abc"))
def test_reset_disconnects_mcu_and_pulldown_turns_off(kind):
    d, r = run(kind, 26.4, 1, True, 1, 1.01, 1.01)
    assert r["held_a"] > 0.35
    assert r["gate_end_v"] < 0.1
    assert abs(r["current_end_a"]) < 1e-5
    assert r["delay_s"] < 0.011
    assert r["peak_v"] < 80
    # No driven-low GPIO: the disconnected pin follows the gate down.
    assert abs(d["mcu"][-1]) < 0.1


def test_input_transient_reaches_both_logic_levels():
    for ctr in (1, 4):
        d, r = transient(ctr)
        assert d["out"][d["time"] < 0.0009].min() >= 2.31
        assert d["out"][(d["time"] > 0.002) & (d["time"] < 0.0029)].max() <= 0.99
        assert d["out"][-1] >= 2.31
        assert 0 < r["on_s"] < 0.001
        assert 0 < r["off_s"] < 0.001
