"""Measure the velocity PI around a moving free-rail operating point."""

import ctypes

import numpy as np

from sim.sil import Sim
from sim.trace import TraceColumn


def velocity_check(gains=None):
    """Return 50-to-60 mm/s step metrics and the 10 mm/s steady error.

    Reference injection is available only in the SIL DLL. The real PI,
    encoder estimator, driver lag and two-mass mechanics remain in the loop.
    """
    simulation = Sim({"params": {"approach_speed_mm_s": 50, **(gains or {})}})
    simulation.prepare()
    simulation.client.start_insert(1000)
    simulation.firmware.lib.fw_probe_reference.argtypes = [ctypes.c_float]
    simulation.advance(3000, True)
    before_speed = simulation.trace[-200:, TraceColumn.MEASURED_VELOCITY_MM_S].mean()
    start_index = len(simulation.records)
    simulation.firmware.lib.fw_probe_reference(60)
    simulation.advance(500, True)
    response = simulation.trace[start_index:]
    # Thresholds use the specified 10 mm/s step, not the observed final value.
    time_10 = response[
        np.flatnonzero(response[:, TraceColumn.MEASURED_VELOCITY_MM_S] >= 51)[0], 0
    ]
    time_90 = response[
        np.flatnonzero(response[:, TraceColumn.MEASURED_VELOCITY_MM_S] >= 59)[0], 0
    ]
    rise_s = round(time_90 - time_10, 6)
    result = {
        "cruise_speed_mm_s": float(before_speed),
        "rise_time_ms": float(rise_s * 1000),
        "bandwidth_Hz": float(round(0.35 / rise_s, 9)),
        "overshoot_percent": float(
            max(0, response[:, TraceColumn.MEASURED_VELOCITY_MM_S].max() - 60)
            / 10
            * 100
        ),
        "steady_60_error_mm_s": float(
            60 - response[-200:, TraceColumn.MEASURED_VELOCITY_MM_S].mean()
        ),
        "max_server_position_mm": float(simulation.plant.server_position_m * 1000),
    }
    true_speed = response[:, TraceColumn.TRUE_VELOCITY_MM_S]
    true_time_10 = response[np.flatnonzero(true_speed >= 51)[0], TraceColumn.TIME_S]
    true_time_90 = response[np.flatnonzero(true_speed >= 59)[0], TraceColumn.TIME_S]
    result["true_rise_time_ms"] = float((true_time_90 - true_time_10) * 1000)
    result["true_bandwidth_Hz"] = float(0.35 / (true_time_90 - true_time_10))
    result["true_overshoot_percent"] = float(max(0, true_speed.max() - 60) * 10)
    simulation.firmware.lib.fw_probe_reference(10)
    simulation.advance(2000, True)
    result["insert_speed_error_mm_s"] = float(
        10 - simulation.trace[-500:, TraceColumn.MEASURED_VELOCITY_MM_S].mean()
    )
    result["insert_true_speed_error_mm_s"] = float(
        10 - simulation.trace[-500:, TraceColumn.TRUE_VELOCITY_MM_S].mean()
    )
    result["final_state"] = int(simulation.firmware.debug.state)
    result["fault_flags"] = int(simulation.firmware.debug.fault_flags)
    result["max_server_position_mm"] = float(
        simulation.trace[:, TraceColumn.SERVER_MM].max()
    )
    return result
