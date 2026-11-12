"""Check the direction of conclusions using small subsets of E1 through E5."""

import pytest

from experiments.cases import filter_cases, jam_case, nominal_case
from experiments.velocity import velocity_check


def test_sensor_latency_increases_jam_peak():
    """E1: the slow HX711 model peaks above ADS1220 at 100 mm/s."""
    slow, _ = jam_case(adc="HX711_10")
    fast, _ = jam_case()
    assert slow["trip_delay_ms"] > 0
    assert fast["trip_delay_ms"] is None
    assert fast["true_trip_N"] < 60 < fast["measured_trip_N"]
    assert slow["peak_pusher_force_N"] > fast["peak_pusher_force_N"]
    assert not slow["current_limit_before_trip"]
    assert slow["peak_regime"] == "transient / protection"


def test_compliance_reduces_jam_peak_and_acceleration_costs_time():
    """E2: compare actual jams at the common 50 mm/s^2 acceleration."""
    stiff, _ = jam_case(acceleration=50)
    compliant, _ = jam_case(stiffness=5, acceleration=50)
    assert compliant["contact_s"] < compliant["fault_ms"] / 1000
    assert compliant["peak_pusher_force_N"] < stiff["peak_pusher_force_N"]
    nominal_stiff, _ = nominal_case()
    nominal_soft, _ = nominal_case(stiffness=5)
    assert nominal_stiff["seated"]
    assert not nominal_soft["seated"]
    assert nominal_soft["final_state"] == "IDLE"
    assert nominal_soft["fault_flags"] == 0
    for stiffness in (200, 50, 20, 5):
        fast, _ = nominal_case(stiffness=stiffness, acceleration=500)
        gentle, _ = nominal_case(stiffness=stiffness, acceleration=50)
        assert gentle["max_free_filtered_N"] < fast["max_free_filtered_N"]
        assert fast["fault_name"] == gentle["fault_name"] == "NONE"
        assert gentle["seated"] == fast["seated"] == (stiffness != 5)
        if fast["seated"]:
            assert gentle["cycle_s"] > fast["cycle_s"]


def test_envelope_reduces_trip_peak_and_residual_force():
    """E3: a lower free-zone threshold detects earlier and unloads the jam."""
    envelope, _ = jam_case()
    global_threshold, _ = jam_case(envelope=False)
    assert envelope["measured_trip_N"] < global_threshold["measured_trip_N"]
    assert envelope["peak_pusher_force_N"] < global_threshold["peak_pusher_force_N"]
    assert envelope["backoff_force_N"] < global_threshold["backoff_force_N"]


def test_filter_reduces_noise_maximum_but_delays_protection():
    """E4: two noise seeds exercise the free-force versus jam-delay trade-off."""
    bypass, bypass_runs = filter_cases(0, seeds=(42, 43))
    filtered, filtered_runs = filter_cases(20, seeds=(42, 43))
    assert bypass["max_free_filtered_N"] > filtered["max_free_filtered_N"]
    assert bypass["jam_delay_ms"] is None
    assert filtered["jam_delay_ms"] > 0
    assert bypass["jam_peak_N"] < filtered["jam_peak_N"]
    assert bypass["faulted_runs"] == filtered["faulted_runs"] == 0
    assert len(bypass_runs) == len(filtered_runs) == 2


def test_nominal_metrics_have_a_complete_cycle():
    """E5: seating and the two phase durations agree with the total cycle."""
    nominal, _ = nominal_case()
    assert nominal["seated"]
    assert 293 < nominal["final_server_mm"] < 297
    assert nominal["peak_true_N"] < 150
    assert nominal["approach_s"] > 0 and nominal["insert_s"] > 0
    assert nominal["approach_s"] + nominal["insert_s"] == pytest.approx(
        nominal["cycle_s"]
    )


def test_cruising_velocity_step_avoids_contact_and_tracks_insert_speed():
    """Phase 2: inject a true reference step on moving free rails."""
    result = velocity_check()
    assert result["cruise_speed_mm_s"] == pytest.approx(50, abs=0.01)
    assert 10 <= result["bandwidth_Hz"] <= 40
    assert result["overshoot_percent"] < 10
    assert result["true_overshoot_percent"] < 10
    assert result["max_server_position_mm"] < 280
    assert result["fault_flags"] == 0 and result["final_state"] == 2
    assert abs(result["insert_speed_error_mm_s"]) < 0.01
    assert abs(result["insert_true_speed_error_mm_s"]) < 0.01


def test_ideal_sensor_rack_peak_exceeds_pusher_peak():
    """Server momentum can produce a higher rack force than pusher force."""
    metrics, _ = jam_case(adc="IDEAL_1000", speed=100, jam_mm=150)
    assert metrics["peak_rack_force_N"] > metrics["peak_pusher_force_N"]
