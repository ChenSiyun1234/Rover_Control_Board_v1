/* Parameter defaults, units, bounds and motion locks; no wire framing here. */
#include "internal.h"

struct parameter_entry {
    enum parameter_id id;
    const char *name;
    size_t offset;
    float default_value;
    float minimum;
    float maximum;
    const char *unit;
    bool locked_while_moving;

};

static const struct parameter_entry parameter_table[] = {
    {PARAM_LC_COUNTS_PER_N, "lc_counts_per_N",
     offsetof(struct axis_params, lc_counts_per_N),
     10485.76f, 1.0f, 1000000.0f, "counts/N", true},
    {PARAM_PUSHER_STIFFNESS_N_PER_MM, "pusher_stiffness_N_per_mm",
     offsetof(struct axis_params, pusher_stiffness_N_per_mm),
     200.0f, 1.0f, 1000.0f, "N/mm", true},
    {PARAM_FREE_LIMIT_N, "free_limit_N",
     offsetof(struct axis_params, free_limit_N),
     60.0f, 1.0f, 500.0f, "N", true},
    {PARAM_MATE_LIMIT_N, "mate_limit_N",
     offsetof(struct axis_params, mate_limit_N),
     150.0f, 1.0f, 500.0f, "N", true},
    {PARAM_SEAT_FORCE_N, "seat_force_N",
     offsetof(struct axis_params, seat_force_N),
     110.0f, 1.0f, 500.0f, "N", true},
    {PARAM_MATE_START_MM, "mate_start_mm",
     offsetof(struct axis_params, mate_start_mm),
     278.0f, 0.0f, 1000.0f, "mm", true},
    {PARAM_SEAT_LO_MM, "seat_lo_mm",
     offsetof(struct axis_params, seat_lo_mm),
     293.0f, 0.0f, 1000.0f, "mm", true},
    {PARAM_SEAT_HI_MM, "seat_hi_mm",
     offsetof(struct axis_params, seat_hi_mm),
     297.0f, 0.0f, 1000.0f, "mm", true},
    {PARAM_APPROACH_SPEED_MM_S, "approach_speed_mm_s",
     offsetof(struct axis_params, approach_speed_mm_s),
     100.0f, 1.0f, 200.0f, "mm/s", true},
    {PARAM_INSERT_SPEED_MM_S, "insert_speed_mm_s",
     offsetof(struct axis_params, insert_speed_mm_s),
     10.0f, 1.0f, 100.0f, "mm/s", true},
    {PARAM_SLOW_ZONE_MM, "slow_zone_mm",
     offsetof(struct axis_params, slow_zone_mm),
     270.0f, 0.0f, 1000.0f, "mm", true},
    {PARAM_FORCE_FILTER_HZ, "force_filter_hz",
     offsetof(struct axis_params, force_filter_hz),
     0.0f, 0.0f, 500.0f, "Hz", true},
    {PARAM_STALE_TIMEOUT_MS, "stale_timeout_ms",
     offsetof(struct axis_params, stale_timeout_ms),
     5.0f, 5.0f, 5000.0f, "ms", true},
    {PARAM_BACKOFF_MM, "backoff_mm",
     offsetof(struct axis_params, backoff_mm),
     2.0f, 0.0f, 20.0f, "mm", true},
    {PARAM_COMM_TIMEOUT_MS, "comm_timeout_ms",
     offsetof(struct axis_params, comm_timeout_ms),
     200.0f, 20.0f, 5000.0f, "ms", true},
    {PARAM_ENVELOPE_ENABLED, "envelope_enabled",
     offsetof(struct axis_params, envelope_enabled),
     1.0f, 0.0f, 1.0f, "bool", true},
    {PARAM_MAX_CURRENT_A, "max_current_A",
     offsetof(struct axis_params, max_current_A),
     8.0f, 0.1f, 8.0f, "A", true},
    {PARAM_ACCEL_MM_S2, "accel_mm_s2",
     offsetof(struct axis_params, accel_mm_s2),
     500.0f, 10.0f, 3000.0f, "mm/s^2", true},
    {PARAM_VELOCITY_KP, "velocity_kp",
     offsetof(struct axis_params, velocity_kp_A_per_mm_s),
     0.035f, 0.0f, 1.0f, "A/(mm/s)", true},
    {PARAM_VELOCITY_KI, "velocity_ki",
     offsetof(struct axis_params, velocity_ki_A_per_mm),
     0.12f, 0.0f, 10.0f, "A/mm", true},
    {PARAM_VELOCITY_FILTER_HZ, "velocity_filter_hz",
     offsetof(struct axis_params, velocity_filter_hz),
     80.0f, 1.0f, 500.0f, "Hz", true},
    {PARAM_BACKOFF_SPEED_MM_S, "backoff_speed_mm_s",
     offsetof(struct axis_params, backoff_speed_mm_s),
     10.0f, 1.0f, 50.0f, "mm/s", true},
    {PARAM_COUNTS_PER_MM, "counts_per_mm",
     offsetof(struct axis_params, counts_per_mm),
     819.2f, 1.0f, 100000.0f, "counts/mm", true},
    {PARAM_POSITION_KP, "position_kp",
     offsetof(struct axis_params, position_kp_per_s),
     5.0f, 0.1f, 50.0f, "1/s", true},
    /* Assumed FSR 402 fit and ADC settings; see sim/fsr.py for source/points. */
    {PARAM_FORCE_SENSOR, "force_sensor",
     offsetof(struct axis_params, force_sensor),
     0.0f, 0.0f, 1.0f, "enum", true},
    {PARAM_FSR_PULLDOWN_OHM, "fsr_pulldown_ohm",
     offsetof(struct axis_params, fsr_pulldown_ohm),
     1000000.0f, 1.0f, 10000000.0f, "ohm", true},
    {PARAM_FSR_A, "fsr_a",
     offsetof(struct axis_params, fsr_a),
     17394.5949506454f, 1.0f, 10000000.0f, "ohm*N^b", true},
    {PARAM_FSR_B, "fsr_b",
     offsetof(struct axis_params, fsr_b),
     0.8503824633f, 0.1f, 3.0f, "1", true},
    {PARAM_ADC_FULL_SCALE, "adc_full_scale",
     offsetof(struct axis_params, adc_full_scale),
     4095.0f, 2.0f, 65535.0f, "counts", true},
    {PARAM_ADC_VREF, "adc_vref",
     offsetof(struct axis_params, adc_vref),
     3.3f, 0.1f, 5.0f, "V", true},
    {PARAM_DRIVE_MODE, "drive_mode", offsetof(struct axis_params, drive_mode),
     0.0f, 0.0f, 1.0f, "enum", true},
    /* Assumed arm tuning: current PI gains scaled by R/Vbus = 0.7/12.
       Back-EMF supplies extra physical damping; see sim3d/cases.py. */
    {PARAM_DUTY_KP, "duty_kp", offsetof(struct axis_params, duty_kp),
     0.002041666667f, 0.0f, 1.0f, "duty/(mm/s)", true},
    {PARAM_DUTY_KI, "duty_ki", offsetof(struct axis_params, duty_ki),
     0.007f, 0.0f, 10.0f, "duty/mm", true},
    /* Assumed 2 mm/s rejects stationary encoder ripple; 50 ms persistence
       exceeds the 80 Hz filter's ~2 ms lag and brief mechanical reversals.
       These are commissioning thresholds, not a hardware safety guarantee. */
    {PARAM_WIRING_CHECK_MM_S, "wiring_check_mm_s",
     offsetof(struct axis_params, wiring_check_mm_s), 2.0f, 0.1f, 200.0f, "mm/s", true},
    {PARAM_WIRING_CHECK_MS, "wiring_check_ms",
     offsetof(struct axis_params, wiring_check_ms), 50.0f, 1.0f, 5000.0f, "ms", true},
    /* Assumed 5 mm/s is cautious near a switch; 3 ticks reject the assumed
       2 ms contact bounce. 1 mm clears the contact; 60 mm is twice the assumed
       30 mm starting offset, bounding a missing-switch or double-swap run. */
    {PARAM_HOME_SPEED_MM_S, "home_speed_mm_s", offsetof(struct axis_params, home_speed_mm_s),
     5.0f, 1.0f, 50.0f, "mm/s", true},
    {PARAM_HOME_DEBOUNCE_MS, "home_debounce_ms", offsetof(struct axis_params, home_debounce_ms),
     3.0f, 1.0f, 100.0f, "ms", true},
    {PARAM_HOME_BACKOFF_MM, "home_backoff_mm", offsetof(struct axis_params, home_backoff_mm),
     1.0f, 0.01f, 20.0f, "mm", true},
    {PARAM_HOME_MAX_TRAVEL_MM, "home_max_travel_mm", offsetof(struct axis_params, home_max_travel_mm),
     60.0f, 1.0f, 1000.0f, "mm", true},

};

static const struct parameter_entry *find_parameter(uint8_t id) {
    unsigned index;
    for (index = 0; index < sizeof(parameter_table) / sizeof(parameter_table[0]); index++) {
        if (parameter_table[index].id == id) {
            return &parameter_table[index];
        }
    }
    return NULL;
}

static float *parameter_field(const struct parameter_entry *entry) {
    return (float *)((uint8_t *)&axis.params + entry->offset);
}

void parameters_reset(void) {
    unsigned index;
    for (index = 0; index < sizeof(parameter_table) / sizeof(parameter_table[0]); index++) {
        *parameter_field(&parameter_table[index]) = parameter_table[index].default_value;
    }
}

enum nack_code parameter_get(uint8_t id, float *value) {
    const struct parameter_entry *entry = find_parameter(id);
    if (entry == NULL) {
        return NACK_BAD_PARAM_ID;
    }
    *value = *parameter_field(entry);
    return NACK_NONE;
}

enum nack_code parameter_set(uint8_t id, float value) {
    const struct parameter_entry *entry = find_parameter(id);
    if (entry == NULL) {
        return NACK_BAD_PARAM_ID;
    }
    /* Ordered comparisons reject NaN as well as infinities. */
    if (!(value >= entry->minimum && value <= entry->maximum)) {
        return NACK_OUT_OF_RANGE;
    }
    if (entry->locked_while_moving &&
        (axis.debug.state == AXIS_MOVING || axis.debug.state == AXIS_RETRACTING ||
                    axis.debug.state == AXIS_HOMING || axis.debug.state == AXIS_HOME_BACKOFF)) {
        return NACK_WRONG_STATE;
    }
    if ((entry->id == PARAM_ENVELOPE_ENABLED || entry->id == PARAM_FORCE_SENSOR ||
         entry->id == PARAM_DRIVE_MODE) &&
        value != 0 && value != 1) {
        return NACK_OUT_OF_RANGE;
    }
    *parameter_field(entry) = value;
    return NACK_NONE;
}
