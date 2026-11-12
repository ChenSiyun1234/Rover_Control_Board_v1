"""Firmware parameter metadata; defaults are assumed controller tuning."""

from .identifiers import ParameterId

# name, default, minimum, maximum, unit; all firmware writes lock during motion.
PARAMS = [
    ("lc_counts_per_N", 10485.76, 1, 1000000.0, "counts/N"),
    ("pusher_stiffness_N_per_mm", 200, 1, 1000, "N/mm"),
    ("free_limit_N", 60, 1, 500, "N"),
    ("mate_limit_N", 150, 1, 500, "N"),
    ("seat_force_N", 110, 1, 500, "N"),
    ("mate_start_mm", 278, 0, 1000, "mm"),
    ("seat_lo_mm", 293, 0, 1000, "mm"),
    ("seat_hi_mm", 297, 0, 1000, "mm"),
    ("approach_speed_mm_s", 100, 1, 200, "mm/s"),
    ("insert_speed_mm_s", 10, 1, 100, "mm/s"),
    ("slow_zone_mm", 270, 0, 1000, "mm"),
    ("force_filter_hz", 0, 0, 500, "Hz"),
    ("stale_timeout_ms", 5, 5, 5000, "ms"),
    ("backoff_mm", 2, 0, 20, "mm"),
    ("comm_timeout_ms", 200, 20, 5000, "ms"),
    ("envelope_enabled", 1, 0, 1, "bool"),
    ("max_current_A", 8, 0.1, 8, "A"),
    ("accel_mm_s2", 500, 10, 3000, "mm/s^2"),
    ("velocity_kp", 0.035, 0, 1, "A/(mm/s)"),
    ("velocity_ki", 0.12, 0, 10, "A/mm"),
    ("velocity_filter_hz", 80, 1, 500, "Hz"),
    ("backoff_speed_mm_s", 10, 1, 50, "mm/s"),
    ("counts_per_mm", 819.2, 1, 100000, "counts/mm"),
    ("position_kp", 5, 0.1, 50, "1/s"),
    ("force_sensor", 0, 0, 1, "enum"),
    ("fsr_pulldown_ohm", 1000000, 1, 10000000, "ohm"),
    ("fsr_a", 17394.59495, 1, 10000000, "ohm*N^b"),
    ("fsr_b", 0.8503824633, 0.1, 3, "1"),
    ("adc_full_scale", 4095, 2, 65535, "counts"),
    ("adc_vref", 3.3, 0.1, 5, "V"),
    ("drive_mode", 0, 0, 1, "enum"),
    ("duty_kp", 0.035*0.7/12, 0, 1, "duty/(mm/s)"),
    ("duty_ki", 0.12*0.7/12, 0, 10, "duty/mm"),
    ("wiring_check_mm_s", 2, 0.1, 200, "mm/s"),
    ("wiring_check_ms", 50, 1, 5000, "ms"),
    # Assumed cautious seek, bounce rejection, clearance and bounded search; see firmware.
    ("home_speed_mm_s", 5, 1, 50, "mm/s"),
    ("home_debounce_ms", 3, 1, 100, "ms"),
    ("home_backoff_mm", 1, 0.01, 20, "mm"),
    ("home_max_travel_mm", 60, 1, 1000, "mm"),
]

PARAMETER_IDS = {entry[0]: ParameterId(index) for index, entry in enumerate(PARAMS, 1)}
