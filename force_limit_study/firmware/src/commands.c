/* Command eligibility and execution; wire parsing lives in protocol.c. */
#include "internal.h"

static float read_float(const uint8_t *bytes) {
    union {
        uint32_t bits;
        float value;
    } decoded;
    decoded.bits = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << WIRE_BYTE_BITS) |
                   ((uint32_t)bytes[2] << (2 * WIRE_BYTE_BITS)) | ((uint32_t)bytes[3] << (3 * WIRE_BYTE_BITS));
    return decoded.value;
}

static void write_float(uint8_t *bytes, float value) {
    union {
        uint32_t bits;
        float value;
    } encoded;
    unsigned index;
    encoded.value = value;
    for (index = 0; index < WIRE_FLOAT_BYTES; index++) {
        bytes[index] = (uint8_t)(encoded.bits >> (index * WIRE_BYTE_BITS));
    }
}

static void start_tare(void) {
    axis.tare_left = TARE_SAMPLES;
    axis.tare_sum_N = 0;
    axis.debug.tare_samples = 0;
}

void command(uint8_t seq, uint8_t cmd, const uint8_t *payload, uint8_t length) {
    enum nack_code error = NACK_NONE;
    uint8_t reply[WIRE_PARAMETER_BYTES] = {0};
    uint8_t reply_length = 0;
    uint8_t expected_length = 0;
    uint8_t parameter_id = 0;
    float value = 0;
    axis.last_host_ms = axis.debug.now_ms;
    switch (cmd) {
        case CMD_PING:
        case CMD_ENABLE:
        case CMD_DISABLE:
        case CMD_STOP:
        case CMD_HOME:
        case CMD_TARE:
        case CMD_CLEAR_FAULT:
        case CMD_GET_STATUS:
            break;
        case CMD_START_INSERT:
        case CMD_RETRACT:
            expected_length = WIRE_FLOAT_BYTES;
            break;
        case CMD_SET_PARAM:
            expected_length = WIRE_PARAMETER_BYTES;
            break;
        case CMD_GET_PARAM:
            expected_length = 1;
            break;
        case CMD_STREAM:
            expected_length = WIRE_U16_BYTES;
            break;
        default:
            error = NACK_UNKNOWN_CMD;
            break;
    }
    if (!error && length != expected_length) {
        error = NACK_BAD_LENGTH;
    }
    if (!error && (cmd == CMD_START_INSERT || cmd == CMD_RETRACT)) {
        value = read_float(payload);
        if (!(value > 0 && value <= MAX_COMMAND_DISTANCE_MM)) {
            error = NACK_OUT_OF_RANGE;
        }
    }
    if (!error && (cmd == CMD_SET_PARAM || cmd == CMD_GET_PARAM)) {
        parameter_id = payload[0];
        error = parameter_get(parameter_id, &value);
        if (!error && cmd == CMD_SET_PARAM) {
            value = read_float(payload + 1);
            error = parameter_set(parameter_id, value);
        }
    }
    if (!error) {
        switch (cmd) {
            case CMD_PING:
                reply[0] = VERSION_MAJOR;
                reply[1] = VERSION_MINOR;
                reply[2] = PROTOCOL_VERSION;
                reply_length = PING_PAYLOAD_BYTES;
                break;
            case CMD_ENABLE:
                if (axis.debug.state == AXIS_DISABLED && axis.safety_ok &&
                    !axis.debug.fault_flags) {
                    axis.enabled = true;
                    axis.debug.state = AXIS_IDLE;
                    axis.hold_mm = axis.debug.carriage_mm;
                    start_tare();
                } else {
                    error = NACK_WRONG_STATE;
                }
                break;
            case CMD_DISABLE:
                axis.enabled = false;
                axis.debug.state = AXIS_DISABLED;
                axis.debug.velocity_ref = 0;
                axis.integral_A = 0;
                axis.integral_duty = 0;
                axis.wiring_pending = false;
                axis.backoff_phase = BACKOFF_NONE;
                break;
            case CMD_HOME:
            case CMD_START_INSERT:
            case CMD_RETRACT:
                if (axis.debug.state == AXIS_DISABLED) {
                    error = NACK_NOT_ENABLED;
                } else if ((axis.debug.state != AXIS_IDLE && axis.debug.state != AXIS_SEATED) ||
                           axis.tare_left) {
                    error = NACK_WRONG_STATE;
                } else {
                    axis.debug.state = cmd == CMD_START_INSERT ? AXIS_MOVING : AXIS_RETRACTING;
                    axis.target_mm = cmd == CMD_START_INSERT
                        ? value : axis.debug.carriage_mm - value;
                    axis.integral_A = 0;
                    axis.integral_duty = 0;
                    axis.wiring_pending = false;
                    if (cmd == CMD_HOME) {
                        axis.debug.state = AXIS_HOMING;
                        axis.home_travel_mm = 0;
                        axis.home_ticks = 0;
                        axis.home_braking = false;
                    }
                }
                break;
            case CMD_STOP:
                if (axis.debug.state == AXIS_MOVING || axis.debug.state == AXIS_RETRACTING ||
                    axis.debug.state == AXIS_HOMING || axis.debug.state == AXIS_HOME_BACKOFF) {
                    axis.debug.state = AXIS_IDLE;
                    axis.hold_mm = axis.debug.carriage_mm;
                } else if (axis.debug.state != AXIS_IDLE) {
                    error = NACK_WRONG_STATE;
                }
                break;
            case CMD_TARE:
                if (axis.debug.state != AXIS_IDLE ||
                    absolute(axis.debug.vel_mm_s) > STATIONARY_SPEED_MM_S) {
                    error = NACK_WRONG_STATE;
                } else {
                    start_tare();
                }
                break;
            case CMD_CLEAR_FAULT:
                if (!axis.safety_ok || axis.latest_code >= ADC_SIGNED_MAX ||
                    axis.latest_code <= ADC_SIGNED_MIN || axis.debug.force_N > axis.debug.limit_N ||
                    axis.debug.now_ms - axis.last_adc_ms > axis.params.stale_timeout_ms ||
                    axis.backoff_phase == BACKOFF_BRAKING ||
                    axis.backoff_phase == BACKOFF_REVERSING) {
                    error = NACK_WRONG_STATE;
                } else if (axis.debug.state == AXIS_FAULT || axis.debug.state == AXIS_ESTOP ||
                           (axis.debug.state == AXIS_DISABLED && axis.debug.fault_flags)) {
                    axis.enabled = true;
                    axis.debug.state = AXIS_IDLE;
                    axis.debug.fault_flags = 0;
                    axis.backoff_phase = BACKOFF_NONE;
                    axis.hold_mm = axis.debug.carriage_mm;
                    axis.integral_A = 0;
                    axis.integral_duty = 0;
                    axis.wiring_pending = false;
                } else {
                    error = NACK_WRONG_STATE;
                }
                break;
            case CMD_GET_STATUS:
                send_status(seq, cmd | UART_REPLY_BIT);
                return;
            case CMD_GET_PARAM:
                reply[0] = parameter_id;
                write_float(reply + 1, value);
                reply_length = WIRE_PARAMETER_BYTES;
                break;
            case CMD_STREAM:
                protocol_stream((uint16_t)(payload[0] | ((uint16_t)payload[1] << WIRE_BYTE_BITS)));
                break;
            default:
                break;
        }
    }
    if (error) {
        reply[0] = cmd;
        reply[1] = (uint8_t)error;
        protocol_emit(seq, UART_NACK_COMMAND, reply, WIRE_U16_BYTES);
    } else {
        protocol_emit(seq, cmd | UART_REPLY_BIT, reply, reply_length);
    }
}
