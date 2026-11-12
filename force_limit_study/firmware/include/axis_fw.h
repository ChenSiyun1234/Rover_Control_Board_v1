/* Public, register-independent firmware ABI and stable wire identifiers. */
#ifndef AXIS_FW_H
#define AXIS_FW_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif
enum axis_state {
    AXIS_DISABLED = 0,
    AXIS_IDLE = 1,
    AXIS_MOVING = 2,
    AXIS_SEATED = 3,
    AXIS_RETRACTING = 4,
    AXIS_FAULT = 5,
    AXIS_ESTOP = 6,
    AXIS_HOMING = 7,
    AXIS_HOME_BACKOFF = 8,
};

enum axis_fault {
    FAULT_OVERFORCE = 1,
    FAULT_SENSOR_STALE = 2,
    FAULT_SENSOR_RANGE = 4,
    FAULT_COMM_TIMEOUT = 8,
    FAULT_ESTOP = 16,
    FAULT_WIRING = 32,
    FAULT_HOME_TIMEOUT = 64,
};

enum nack_code {
    NACK_NONE = 0,
    NACK_UNKNOWN_CMD = 1,
    NACK_BAD_LENGTH = 2,
    NACK_OUT_OF_RANGE = 3,
    NACK_WRONG_STATE = 4,
    NACK_NOT_ENABLED = 5,
    NACK_BAD_PARAM_ID = 6,
};

enum command_id {
    CMD_PING = 1,
    CMD_ENABLE = 2,
    CMD_DISABLE = 3,
    CMD_START_INSERT = 16,
    CMD_STOP = 17,
    CMD_RETRACT = 18,
    CMD_TARE = 19,
    CMD_HOME = 20,
    CMD_CLEAR_FAULT = 32,
    CMD_GET_STATUS = 48,
    CMD_SET_PARAM = 64,
    CMD_GET_PARAM = 65,
    CMD_STREAM = 80,
};

enum parameter_id {
    PARAM_LC_COUNTS_PER_N = 1,
    PARAM_PUSHER_STIFFNESS_N_PER_MM = 2,
    PARAM_FREE_LIMIT_N = 3,
    PARAM_MATE_LIMIT_N = 4,
    PARAM_SEAT_FORCE_N = 5,
    PARAM_MATE_START_MM = 6,
    PARAM_SEAT_LO_MM = 7,
    PARAM_SEAT_HI_MM = 8,
    PARAM_APPROACH_SPEED_MM_S = 9,
    PARAM_INSERT_SPEED_MM_S = 10,
    PARAM_SLOW_ZONE_MM = 11,
    PARAM_FORCE_FILTER_HZ = 12,
    PARAM_STALE_TIMEOUT_MS = 13,
    PARAM_BACKOFF_MM = 14,
    PARAM_COMM_TIMEOUT_MS = 15,
    PARAM_ENVELOPE_ENABLED = 16,
    PARAM_MAX_CURRENT_A = 17,
    PARAM_ACCEL_MM_S2 = 18,
    PARAM_VELOCITY_KP = 19,
    PARAM_VELOCITY_KI = 20,
    PARAM_VELOCITY_FILTER_HZ = 21,
    PARAM_BACKOFF_SPEED_MM_S = 22,
    PARAM_COUNTS_PER_MM = 23,
    PARAM_POSITION_KP = 24,
    PARAM_FORCE_SENSOR = 25,
    PARAM_FSR_PULLDOWN_OHM = 26,
    PARAM_FSR_A = 27,
    PARAM_FSR_B = 28,
    PARAM_ADC_FULL_SCALE = 29,
    PARAM_ADC_VREF = 30,
    PARAM_DRIVE_MODE = 31,
    PARAM_DUTY_KP = 32,
    PARAM_DUTY_KI = 33,
    PARAM_WIRING_CHECK_MM_S = 34,
    PARAM_WIRING_CHECK_MS = 35,
    PARAM_HOME_SPEED_MM_S = 36,
    PARAM_HOME_DEBOUNCE_MS = 37,
    PARAM_HOME_BACKOFF_MM = 38,
    PARAM_HOME_MAX_TRAVEL_MM = 39,
};

typedef struct {
    uint32_t now_ms;
    int32_t encoder_counts;
    int32_t adc_code;
    bool adc_new;
    bool estop_ok;
    bool limit_switch; /* Logical active; MCU adapter inverts the active-low pin. */
} fw_inputs_t;
typedef struct {
    float current_cmd_A;
    bool drive_enable;
    float pwm_duty;
    uint8_t dir; /* 1 = positive, 0 = negative; zero duty uses 1. */
} fw_outputs_t;
typedef struct {
    uint32_t state;
    uint32_t fault_flags;
    float force_N;
    float force_raw_N;
    float carriage_mm;
    float server_est_mm;
    float vel_mm_s;
    float limit_N;
    float current_A;
    uint32_t now_ms;
    uint32_t crc_error_count;
    uint32_t frame_drop_count;
    uint32_t tare_samples;
    float velocity_ref;
    uint32_t fault_ms;
    float detection_force_N;
} fw_debug_t;
#ifdef AXIS_SIL
/* DLL-only experiment hook; safety checks still run, normal motion is unchanged. */
API void fw_probe_reference(float reference_mm_s);
#endif
API void fw_init(void);
API void fw_step(const fw_inputs_t *inputs, fw_outputs_t *outputs);
API void fw_uart_rx(const uint8_t *data, size_t length);
API size_t fw_uart_tx(uint8_t *output, size_t capacity);
API void fw_get_debug(fw_debug_t *debug);
API uint16_t fw_crc16(const uint8_t *data, size_t length);
#endif
