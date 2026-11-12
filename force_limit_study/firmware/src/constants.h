/* Fixed controller and wire constants; tunable defaults live in parameters.c. */
#ifndef AXIS_CONSTANTS_H
#define AXIS_CONSTANTS_H
#define CONTROL_FREQUENCY_HZ 1000.0f /* Encoder difference samples per second. */
#define CONTROL_PERIOD_S 0.001f /* One 1 kHz control tick in seconds. */
#define SECONDS_PER_MS 0.001f /* Timestamp conversion for ADC filter intervals. */
#define TWO_PI 6.2831853f /* Radians per cycle for first-order filters. */
#define TARE_SAMPLES 50u /* Stationary ADC conversions per tare. */
#define ADC_SIGNED_MAX 8388607 /* Positive signed 24-bit rail. */
#define ADC_SIGNED_MIN (-8388608) /* Negative signed 24-bit rail. */
#define SEAT_DEBOUNCE_TICKS 5u /* Consecutive valid seating ticks. */
#define STATIONARY_SPEED_MM_S 0.5f /* Maximum measured speed during tare. */
#define STATIONARY_REFERENCE_MM_S 0.01f /* Maximum reference during tare. */
#define BACKOFF_BRAKED_SPEED_MM_S 2.0f /* Speed below which reversal starts. */
#define MAX_COMMAND_DISTANCE_MM 1000.0f /* Largest accepted travel command. */
#define BOOLEAN_THRESHOLD 0.5f /* Threshold for stored boolean parameters. */
#define UART_INTERBYTE_TIMEOUT_MS 20u /* Incomplete frame expiry. */
#define UART_MAX_PAYLOAD 64u /* Maximum wire payload bytes. */
#define UART_FRAME_OVERHEAD 7u /* Sync, header and CRC bytes. */
#define UART_MAX_FRAME (UART_MAX_PAYLOAD + UART_FRAME_OVERHEAD) /* RX frame capacity. */
#define UART_TX_CAPACITY 8192u /* Bounded outgoing ring capacity. */
#define UART_SYNC_FIRST 0xA5u /* First sync byte. */
#define UART_SYNC_SECOND 0x5Au /* Second sync byte. */
#define UART_REPLY_BIT 0x80u /* Successful response command bit. */
#define UART_STREAM_COMMAND 0x7Eu /* Unsolicited status command. */
#define UART_NACK_COMMAND 0x7Fu /* Negative response command. */
#define STATUS_PAYLOAD_BYTES 31u /* Packed status record size. */
#define WIRE_FLOAT_BYTES 4u /* IEEE-754 binary32 payload size. */
#define WIRE_U16_BYTES 2u /* Packed unsigned 16-bit payload size. */
#define WIRE_PARAMETER_BYTES 5u /* Parameter ID plus binary32 value. */
#define WIRE_BYTE_BITS 8u /* Bits per serialized byte. */
#define CRC_INITIAL 0xFFFFu /* CRC-16/CCITT-FALSE initial register. */
#define CRC_TOP_BIT 0x8000u /* CRC feedback bit. */
#define CRC_POLYNOMIAL 0x1021u /* CRC-16/CCITT-FALSE polynomial. */
#define UART_LENGTH_OFFSET 2u /* Payload length byte in a frame. */
#define UART_SEQUENCE_OFFSET 3u /* Request sequence byte in a frame. */
#define UART_COMMAND_OFFSET 4u /* Command byte in a frame. */
#define UART_PAYLOAD_OFFSET 5u /* First payload byte in a frame. */
#define UART_CRC_HEADER_BYTES 3u /* Length, sequence and command covered by CRC. */
#define STATUS_FAULT_OFFSET 1u /* Fault mask after the one-byte state. */
#define STATUS_CARRIAGE_OFFSET 3u /* Carriage position binary32. */
#define STATUS_SERVER_OFFSET 7u /* Estimated server position binary32. */
#define STATUS_VELOCITY_OFFSET 11u /* Measured velocity binary32. */
#define STATUS_FORCE_OFFSET 15u /* Filtered force binary32. */
#define STATUS_CURRENT_OFFSET 19u /* Current command binary32. */
#define STATUS_TIME_OFFSET 23u /* Millisecond timestamp uint32. */
#define STATUS_CRC_ERRORS_OFFSET 27u /* CRC error counter uint16. */
#define STATUS_DROPS_OFFSET 29u /* Frame drop counter uint16. */
#define VERSION_MAJOR 1u /* Firmware semantic version major. */
#define VERSION_MINOR 0u /* Firmware semantic version minor. */
#define PROTOCOL_VERSION 1u /* Wire protocol revision in PING. */
#define PING_PAYLOAD_BYTES 3u /* Major, minor and protocol revision. */
#endif
