"""Named columns for the compact NumPy SIL trace format."""

from enum import IntEnum


class TraceColumn(IntEnum):
    """Indices in each millisecond trace sample."""

    TIME_S = 0
    CARRIAGE_MM = 1
    SERVER_MM = 2
    TRUE_FORCE_N = 3
    MEASURED_FORCE_N = 4
    STATE = 5
    CURRENT_COMMAND_A = 6
    DRIVE_ENABLE = 7
    MEASURED_VELOCITY_MM_S = 8
    REFERENCE_VELOCITY_MM_S = 9
    RAW_FORCE_N = 10
    LIMIT_N = 11
    SERVER_EST_MM = 12
    TRUE_VELOCITY_MM_S = 13
    DRIVER_CURRENT_A = 14
    RACK_FORCE_N = 15
    PWM_DUTY = 16
    DIR = 17
    FIRMWARE_CARRIAGE_MM = 18
    LIMIT_SWITCH = 19


class PlantTraceColumn(IntEnum):
    """Indices in each 10 kHz plant trace sample."""

    TIME_S = 0
    RACK_FORCE_N = 1
