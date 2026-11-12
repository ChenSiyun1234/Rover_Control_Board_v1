"""Stable UART identifiers shared by the Python host, SIL and tests."""

from enum import IntEnum, IntFlag


class State(IntEnum):
    """Controller states reported by firmware telemetry."""

    DISABLED = 0
    IDLE = 1
    MOVING = 2
    SEATED = 3
    RETRACTING = 4
    FAULT = 5
    ESTOP = 6
    HOMING = 7
    HOME_BACKOFF = 8


class Fault(IntFlag):
    """Latched controller fault bits."""

    OVERFORCE = 1
    SENSOR_STALE = 2
    SENSOR_RANGE = 4
    COMM_TIMEOUT = 8
    ESTOP = 16
    WIRING = 32
    HOME_TIMEOUT = 64


class Command(IntEnum):
    """Request command bytes; successful replies set bit 7."""

    PING = 1
    ENABLE = 2
    DISABLE = 3
    START_INSERT = 16
    STOP = 17
    RETRACT = 18
    TARE = 19
    HOME = 20
    CLEAR_FAULT = 32
    GET_STATUS = 48
    SET_PARAM = 64
    GET_PARAM = 65
    STREAM = 80
    TELEMETRY = 0x7E
    NACK = 0x7F


class ParameterId(IntEnum):
    """Parameter addresses; names match the firmware metadata table."""

    LC_COUNTS_PER_N = 1
    PUSHER_STIFFNESS_N_PER_MM = 2
    FREE_LIMIT_N = 3
    MATE_LIMIT_N = 4
    SEAT_FORCE_N = 5
    MATE_START_MM = 6
    SEAT_LO_MM = 7
    SEAT_HI_MM = 8
    APPROACH_SPEED_MM_S = 9
    INSERT_SPEED_MM_S = 10
    SLOW_ZONE_MM = 11
    FORCE_FILTER_HZ = 12
    STALE_TIMEOUT_MS = 13
    BACKOFF_MM = 14
    COMM_TIMEOUT_MS = 15
    ENVELOPE_ENABLED = 16
    MAX_CURRENT_A = 17
    ACCEL_MM_S2 = 18
    VELOCITY_KP = 19
    VELOCITY_KI = 20
    VELOCITY_FILTER_HZ = 21
    BACKOFF_SPEED_MM_S = 22
    COUNTS_PER_MM = 23
    POSITION_KP = 24
    FORCE_SENSOR = 25
    FSR_PULLDOWN_OHM = 26
    FSR_A = 27
    FSR_B = 28
    ADC_FULL_SCALE = 29
    ADC_VREF = 30
    DRIVE_MODE = 31
    DUTY_KP = 32
    DUTY_KI = 33
    WIRING_CHECK_MM_S = 34
    WIRING_CHECK_MS = 35
    HOME_SPEED_MM_S = 36
    HOME_DEBOUNCE_MS = 37
    HOME_BACKOFF_MM = 38
    HOME_MAX_TRAVEL_MM = 39
