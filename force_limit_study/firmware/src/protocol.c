/* Bounded UART framing, resynchronization, telemetry encoding and TX buffering. */
#include "internal.h"

static uint8_t rx[UART_MAX_FRAME];
static uint8_t tx[UART_TX_CAPACITY];
static unsigned used;
static unsigned expected;
static unsigned head;
static unsigned tail;
static uint32_t byte_ms;
static uint32_t stream_ms;
static uint32_t last_stream_ms;

static uint16_t read_u16(const uint8_t *bytes) {
    return (uint16_t)(bytes[0] | ((uint16_t)bytes[1] << WIRE_BYTE_BITS));
}

static void write_u16(uint8_t *bytes, uint16_t value) {
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> WIRE_BYTE_BITS);
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

void protocol_emit(uint8_t seq, uint8_t cmd, const uint8_t *payload, unsigned length) {
    uint8_t frame[UART_MAX_FRAME];
    unsigned index;
    unsigned free_bytes = (tail + UART_TX_CAPACITY - head - 1) % UART_TX_CAPACITY;
    uint16_t crc;
    if (free_bytes < length + UART_FRAME_OVERHEAD) {
        axis.debug.frame_drop_count++;
        return;
    }
    frame[0] = UART_SYNC_FIRST;
    frame[1] = UART_SYNC_SECOND;
    frame[UART_LENGTH_OFFSET] = (uint8_t)length;
    frame[UART_SEQUENCE_OFFSET] = seq;
    frame[UART_COMMAND_OFFSET] = cmd;
    for (index = 0; index < length; index++) {
        frame[UART_PAYLOAD_OFFSET + index] = payload[index];
    }
    crc = fw_crc16(frame + UART_LENGTH_OFFSET, length + UART_CRC_HEADER_BYTES);
    write_u16(frame + UART_PAYLOAD_OFFSET + length, crc);
    for (index = 0; index < length + UART_FRAME_OVERHEAD; index++) {
        tx[head] = frame[index];
        head = (head + 1) % UART_TX_CAPACITY;
    }
}

void send_status(uint8_t seq, uint8_t cmd) {
    uint8_t payload[STATUS_PAYLOAD_BYTES];
    unsigned index;
    payload[0] = (uint8_t)axis.debug.state;
    write_u16(payload + STATUS_FAULT_OFFSET, (uint16_t)axis.debug.fault_flags);
    write_float(payload + STATUS_CARRIAGE_OFFSET, axis.debug.carriage_mm);
    write_float(payload + STATUS_SERVER_OFFSET, axis.debug.server_est_mm);
    write_float(payload + STATUS_VELOCITY_OFFSET, axis.debug.vel_mm_s);
    write_float(payload + STATUS_FORCE_OFFSET, axis.debug.force_N);
    write_float(payload + STATUS_CURRENT_OFFSET, axis.debug.current_A);
    for (index = 0; index < WIRE_FLOAT_BYTES; index++) {
        payload[STATUS_TIME_OFFSET + index] = (uint8_t)(axis.debug.now_ms >> (WIRE_BYTE_BITS * index));
    }
    write_u16(payload + STATUS_CRC_ERRORS_OFFSET, (uint16_t)axis.debug.crc_error_count);
    write_u16(payload + STATUS_DROPS_OFFSET, (uint16_t)axis.debug.frame_drop_count);
    protocol_emit(seq, cmd, payload, STATUS_PAYLOAD_BYTES);
}

void protocol_stream(uint16_t period_ms) {
    stream_ms = period_ms;
    last_stream_ms = axis.debug.now_ms;
}

void protocol_reset(void) {
    used = 0;
    expected = 0;
    head = 0;
    tail = 0;
    byte_ms = 0;
    stream_ms = 0;
    last_stream_ms = 0;
}

void protocol_tick(void) {
    if (used && axis.debug.now_ms - byte_ms > UART_INTERBYTE_TIMEOUT_MS) {
        used = 0;
        axis.debug.frame_drop_count++;
    }
    if (stream_ms && axis.debug.now_ms - last_stream_ms >= stream_ms) {
        last_stream_ms = axis.debug.now_ms;
        send_status(0, UART_STREAM_COMMAND);
    }
}

static void discard_prefix(unsigned length) {
    unsigned index;
    for (index = length; index < used; index++) {
        rx[index - length] = rx[index];
    }
    used -= length;
}

void fw_uart_rx(const uint8_t *data, size_t length) {
    size_t index;
    for (index = 0; index < length; index++) {
        if (used && axis.debug.now_ms - byte_ms > UART_INTERBYTE_TIMEOUT_MS) {
            used = 0;
            axis.debug.frame_drop_count++;
        }
        byte_ms = axis.debug.now_ms;
        rx[used++] = data[index];
        while (used) {
            if (rx[0] != UART_SYNC_FIRST) {
                discard_prefix(1);
                continue;
            }
            if (used < WIRE_U16_BYTES) {
                break;
            }
            if (rx[1] != UART_SYNC_SECOND) {
                discard_prefix(1);
                continue;
            }
            if (used <= UART_LENGTH_OFFSET) {
                break;
            }
            if (rx[UART_LENGTH_OFFSET] > UART_MAX_PAYLOAD) {
                axis.debug.frame_drop_count++;
                discard_prefix(1);
                continue;
            }
            expected = rx[UART_LENGTH_OFFSET] + UART_FRAME_OVERHEAD;
            if (used < expected) {
                break;
            }
            if (fw_crc16(rx + UART_LENGTH_OFFSET,
                         rx[UART_LENGTH_OFFSET] + UART_CRC_HEADER_BYTES) ==
                read_u16(rx + expected - WIRE_U16_BYTES)) {
                command(rx[UART_SEQUENCE_OFFSET], rx[UART_COMMAND_OFFSET], rx + UART_PAYLOAD_OFFSET, rx[UART_LENGTH_OFFSET]);
                discard_prefix(expected);
            } else {
                /* Corrupt frames have no trustworthy sequence or command to NACK. */
                axis.debug.crc_error_count++;
                discard_prefix(1);
            }
        }
    }
}

size_t fw_uart_tx(uint8_t *output, size_t capacity) {
    size_t length = 0;
    while (tail != head && length < capacity) {
        output[length++] = tx[tail];
        tail = (tail + 1) % UART_TX_CAPACITY;
    }
    return length;
}
