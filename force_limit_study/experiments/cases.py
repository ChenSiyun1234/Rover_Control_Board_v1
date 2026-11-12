"""Deterministic experiment cases; importable by the regression tests."""

import numpy as np

from host.axis_client.identifiers import Fault
from sim.sil import Sim
from sim.trace import PlantTraceColumn, TraceColumn

ADC_ORDER = (
    "HX711_10",
    "HX711_80",
    "ADS1220_330",
    "ADS1220_1000",
    "ADS1220_2000",
    "IDEAL_1000",
)
CUTOFFS = (0, 200, 100, 50, 20)


def jam_case(
    adc="ADS1220_1000",
    speed=100,
    stiffness=200,
    jam_mm=150,
    envelope=True,
    cutoff=0,
    seed=42,
    acceleration=500,
    include_plant_trace=False,
):
    """Run a jam through braking and 1.5 s of backoff; return metrics and trace."""
    simulation = Sim(
        {
            "adc": adc,
            "jam": True,
            "physical": {"pusher_k": stiffness, "x_jam": jam_mm, "seed": seed},
            "params": {
                "approach_speed_mm_s": speed,
                "pusher_stiffness_N_per_mm": stiffness,
                "envelope_enabled": int(envelope),
                "force_filter_hz": cutoff,
                "accel_mm_s2": acceleration,
            },
        }
    )
    summary = simulation.run_insert()
    if summary["fault_flags"] != 1:
        raise RuntimeError(f"Expected overforce, got {summary}")
    trip_time_s = summary["fault_ms"] / 1000
    true_trip_N = simulation.plant.force
    current_limit_s = simulation.first_current_limit_s
    crossing_s = simulation.first_limit_crossing_s
    # The comparison threshold is 150 N when the envelope is disabled.
    if not envelope:
        crossing = simulation.trace[simulation.trace[:, TraceColumn.TRUE_FORCE_N] > 150]
        crossing_s = crossing[0, 0] if len(crossing) else None
    current_limit_before_trip = (
        current_limit_s is not None and current_limit_s <= trip_time_s
    )
    contact_s = simulation.first_contact_s
    if contact_s is None or contact_s > trip_time_s:
        raise RuntimeError(
            "Run faulted before reaching the jam; lower approach acceleration"
        )
    simulation.advance(1500, True)
    trace = simulation.trace
    contact_index = np.searchsorted(trace[:, TraceColumn.TIME_S], contact_s)
    peak_index = np.argmax(trace[:, TraceColumn.TRUE_FORCE_N])
    peak_current_A = float(trace[peak_index, TraceColumn.DRIVER_CURRENT_A])
    peak_command_A = float(trace[peak_index, TraceColumn.CURRENT_COMMAND_A])
    metrics = {
        "adc": adc,
        "speed_mm_s": speed,
        "acceleration_mm_s2": acceleration,
        "stiffness_N_mm": stiffness,
        "jam_mm": jam_mm,
        "envelope": envelope,
        "cutoff_Hz": cutoff,
        "seed": seed,
        "measured_trip_N": summary["detection_force_N"],
        "true_trip_N": true_trip_N,
        "trip_delay_ms": None
        if crossing_s is None
        else (trip_time_s - crossing_s) * 1000,
        "peak_pusher_force_N": simulation.peak_true_force_N,
        "peak_rack_force_N": float(simulation.plant_trace[:, PlantTraceColumn.RACK_FORCE_N].max()),
        "current_limit_before_trip": current_limit_before_trip,
        "current_at_peak_A": peak_current_A,
        "command_at_peak_A": peak_command_A,
        "stall_force_N": float(
            simulation.parameters.max_current
            * simulation.parameters.kt
            * 2
            * np.pi
            * simulation.parameters.efficiency
            / simulation.parameters.lead
            - simulation.parameters.coulomb
        ),
        "peak_regime": "current-limited stall"
        if abs(peak_current_A) >= 7.99 and peak_command_A >= 7.99
        else "transient / protection",
        "backoff_force_N": simulation.plant.force,
        "fault_ms": summary["fault_ms"],
        "contact_s": contact_s,
        "contact_speed_mm_s": float(
            trace[contact_index, TraceColumn.TRUE_VELOCITY_MM_S]
        ),
    }
    if include_plant_trace:
        return metrics, trace, simulation.plant_trace
    return metrics, trace


def nominal_case(stiffness=200, cutoff=0, seed=42, acceleration=500, gains=None):
    """Run one insertion, reporting both seating outcome and phase durations."""
    simulation = Sim(
        {
            "physical": {"pusher_k": stiffness, "seed": seed},
            "params": {
                "pusher_stiffness_N_per_mm": stiffness,
                "force_filter_hz": cutoff,
                **(gains or {}),
                "accel_mm_s2": acceleration,
            },
        }
    )
    summary = simulation.run_insert()
    trace = simulation.trace
    moving = (trace[:, TraceColumn.STATE] == 2) | (
        (trace[:, TraceColumn.TIME_S] >= simulation.insertion_started_ms / 1000)
        & (trace[:, TraceColumn.STATE] == 5)
    )
    free_travel = moving & (
        trace[:, TraceColumn.SERVER_MM] < simulation.parameters.mate_start
    )
    insert_rows = np.flatnonzero(moving & (trace[:, TraceColumn.SERVER_EST_MM] >= 270))
    start_s = simulation.insertion_started_ms / 1000
    approach_s = (
        (trace[insert_rows[0], 0] - start_s)
        if len(insert_rows)
        else summary["cycle_time_s"]
    )
    metrics = {
        "stiffness_N_mm": stiffness,
        "acceleration_mm_s2": acceleration,
        "resonance_Hz": float(
            np.sqrt(stiffness * 1000 / simulation.parameters.server_mass) / (2 * np.pi)
        ),
        "damping_ratio": simulation.parameters.damping_ratio,
        "server_mode_damping_ratio": float(
            simulation.plant.damping
            / (2 * np.sqrt(stiffness * 1000 * simulation.parameters.server_mass))
        ),
        "cutoff_Hz": cutoff,
        "seed": seed,
        "seated": summary["final_state"] == "SEATED",
        "final_state": summary["final_state"],
        "final_server_mm": summary["seat_position_mm"],
        "peak_true_N": simulation.peak_true_force_N,
        "peak_tick_N": summary["peak_force_N"],
        "cycle_s": summary["cycle_time_s"],
        "approach_s": approach_s,
        "insert_s": summary["cycle_time_s"] - approach_s,
        "max_free_filtered_N": float(
            trace[free_travel, TraceColumn.MEASURED_FORCE_N].max()
        ),
        "fault_flags": summary["fault_flags"],
        "fault_name": Fault(summary["fault_flags"]).name or "NONE",
    }
    return metrics, trace


def filter_cases(cutoff, seeds=range(42, 62)):
    """Aggregate the observed free-travel noise maxima, retaining individual runs."""
    runs = [nominal_case(cutoff=cutoff, seed=seed)[0] for seed in seeds]
    jam = jam_case(cutoff=cutoff)[0]
    aggregate = {
        "cutoff_Hz": cutoff,
        "runs": len(runs),
        "max_free_filtered_N": max(run["max_free_filtered_N"] for run in runs),
        "faulted_runs": sum(bool(run["fault_flags"]) for run in runs),
        "jam_delay_ms": jam["trip_delay_ms"],
        "jam_peak_N": jam["peak_pusher_force_N"],
    }
    return aggregate, runs
