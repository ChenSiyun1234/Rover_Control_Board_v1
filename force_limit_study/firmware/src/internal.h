/* Shared controller data and deliberate cross-module entry points. */
#ifndef AXIS_INTERNAL_H
#define AXIS_INTERNAL_H
#include "axis_fw.h"
#include "constants.h"

struct axis_params {
    float lc_counts_per_N;
    float pusher_stiffness_N_per_mm;
    float free_limit_N;
    float mate_limit_N;
    float seat_force_N;
    float mate_start_mm;
    float seat_lo_mm;
    float seat_hi_mm;
    float approach_speed_mm_s;
    float insert_speed_mm_s;
    float slow_zone_mm;
    float force_filter_hz;
    float stale_timeout_ms;
    float backoff_mm;
    float comm_timeout_ms;
    float envelope_enabled;
    float max_current_A;
    float accel_mm_s2;
    float velocity_kp_A_per_mm_s;
    float velocity_ki_A_per_mm;
    float velocity_filter_hz;
    float backoff_speed_mm_s;
    float counts_per_mm;
    float position_kp_per_s;
    float force_sensor;
    float fsr_pulldown_ohm;
    float fsr_a;
    float fsr_b;
    float adc_full_scale;
    float adc_vref;
    float drive_mode;
    float duty_kp;
    float duty_ki;
    float wiring_check_mm_s;
    float wiring_check_ms;
    float home_speed_mm_s;
    float home_debounce_ms;
    float home_backoff_mm;
    float home_max_travel_mm;
};

enum fault_phase {
    BACKOFF_NONE,
    BACKOFF_BRAKING,
    BACKOFF_REVERSING,
    BACKOFF_HOLDING
};

struct axis_context {
    struct axis_params params;
    fw_debug_t debug;
    uint32_t last_host_ms;
    uint32_t last_adc_ms;
    bool safety_ok;
    bool enabled;
    int32_t latest_code;
    unsigned tare_left;
    float tare_sum_N;
    float tare_offset_N;
    float encoder_zero_mm;
    float home_travel_mm;
    unsigned home_ticks;
    bool home_braking;
    float target_mm;
    float hold_mm;
    float integral_A;
    float integral_duty;
    uint32_t wiring_since_ms;
    bool wiring_pending;
    enum fault_phase backoff_phase;
};

/* The only controller instance is owned by axis.c; modules serialize access. */
struct axis_context *axis_context(void);
#define axis (*axis_context())
float absolute(float value);
float clamp(float value, float minimum, float maximum);
void trip(uint32_t flags);
void command(uint8_t seq, uint8_t cmd, const uint8_t *payload, uint8_t length);
void protocol_reset(void);
void protocol_tick(void);
void send_status(uint8_t seq, uint8_t cmd);
void protocol_emit(uint8_t seq, uint8_t cmd, const uint8_t *payload, unsigned length);
void protocol_stream(uint16_t period_ms);
void force_guard_reset(void);
void force_guard_step(const fw_inputs_t *inputs);
void parameters_reset(void);
enum nack_code parameter_set(uint8_t id, float value);
enum nack_code parameter_get(uint8_t id, float *value);
#endif
