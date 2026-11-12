/* ADC calibration, stationary tare, force envelope and independent safety checks. */
#include "internal.h"

/* The MCU platform must link its single-precision math library. */
extern float powf(float base, float exponent);

static float converted_force(int32_t code) {
    float voltage;
    float resistance;
    if (axis.params.force_sensor == 0) {
        return (float)code / axis.params.lc_counts_per_N;
    }
    if (code <= 0) {
        return 0;
    }
    /* Full scale has no finite inverse. Cap it at the highest finite code,
       full_scale - 1, rather than introducing an infinite force into tare. */
    voltage = clamp((float)code, 0, axis.params.adc_full_scale - 1) /
              axis.params.adc_full_scale * axis.params.adc_vref;
    resistance = axis.params.fsr_pulldown_ohm * (axis.params.adc_vref / voltage - 1);
    return powf(axis.params.fsr_a / resistance, 1 / axis.params.fsr_b);
}

static uint32_t previous_adc_ms;

void force_guard_reset(void) {
    axis.last_adc_ms = 0;
    previous_adc_ms = 0;
}

void force_guard_step(const fw_inputs_t *inputs) {
    float alpha;
    if (inputs->adc_new) {
        axis.last_adc_ms = inputs->now_ms;
        float force = converted_force(inputs->adc_code);
        axis.debug.force_raw_N = force - axis.tare_offset_N;
        if (axis.tare_left && absolute(axis.debug.vel_mm_s) < STATIONARY_SPEED_MM_S &&
            absolute(axis.debug.velocity_ref) < STATIONARY_REFERENCE_MM_S) {
            axis.tare_sum_N += force;
            axis.tare_left--;
            axis.debug.tare_samples = TARE_SAMPLES - axis.tare_left;
            if (!axis.tare_left) {
                axis.tare_offset_N = axis.tare_sum_N / TARE_SAMPLES;
                axis.debug.force_N = 0;
            }
        }
        alpha = TWO_PI * axis.params.force_filter_hz *
                (float)(inputs->now_ms - previous_adc_ms) * SECONDS_PER_MS;
        axis.debug.force_N = axis.params.force_filter_hz == 0 ? axis.debug.force_raw_N :
            axis.debug.force_N + alpha / (1 + alpha) *
            (axis.debug.force_raw_N - axis.debug.force_N);
        previous_adc_ms = inputs->now_ms;
    }
    axis.debug.server_est_mm = axis.debug.carriage_mm -
        axis.debug.force_N / axis.params.pusher_stiffness_N_per_mm;
    /* Connector engagement needs more force than free rail travel. */
    axis.debug.limit_N = axis.params.envelope_enabled >= BOOLEAN_THRESHOLD &&
        axis.debug.server_est_mm < axis.params.mate_start_mm
        ? axis.params.free_limit_N : axis.params.mate_limit_N;
    if (!axis.safety_ok) {
        axis.debug.state = AXIS_ESTOP;
        axis.debug.fault_flags |= FAULT_ESTOP;
        axis.integral_A = 0;
        axis.backoff_phase = BACKOFF_NONE;
    } else if (axis.debug.state != AXIS_DISABLED && axis.debug.state != AXIS_ESTOP) {
        if (inputs->now_ms - axis.last_adc_ms > axis.params.stale_timeout_ms) {
            trip(FAULT_SENSOR_STALE);
        }
        if (inputs->adc_code >= ADC_SIGNED_MAX || inputs->adc_code <= ADC_SIGNED_MIN) {
            trip(FAULT_SENSOR_RANGE);
        }
        /* Idle requires no host traffic; loss of supervision matters during motion. */
        if ((axis.debug.state == AXIS_MOVING || axis.debug.state == AXIS_RETRACTING ||
                    axis.debug.state == AXIS_HOMING || axis.debug.state == AXIS_HOME_BACKOFF) &&
            inputs->now_ms - axis.last_host_ms >= axis.params.comm_timeout_ms) {
            trip(FAULT_COMM_TIMEOUT);
        }
        /* Only sampled, optionally filtered force is available on the real controller. */
        if (axis.debug.force_N > axis.debug.limit_N && !axis.tare_left) {
            trip(FAULT_OVERFORCE);
        }
    }
}
