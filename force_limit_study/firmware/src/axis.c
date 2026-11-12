/* Motion state machine, velocity PI and the single controller instance. */
#include "internal.h"

static struct axis_context context;
static float previous_position_mm;
static unsigned seat_ticks;
#ifdef AXIS_SIL
static bool probe_active;
static float probe_reference_mm_s;

void fw_probe_reference(float reference_mm_s) {
    probe_active = true;
    probe_reference_mm_s = reference_mm_s;
}
#endif

struct axis_context *axis_context(void) {
    return &context;
}

float absolute(float value) {
    return value < 0 ? -value : value;
}

float clamp(float value, float minimum, float maximum) {
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

void trip(uint32_t flags) {
    if (axis.debug.state != AXIS_FAULT && axis.debug.state != AXIS_ESTOP) {
        axis.debug.fault_ms = axis.debug.now_ms;
        axis.debug.detection_force_N = axis.debug.force_N;
        /* Unload stored spring energy after braking instead of holding the jam compressed. */
        axis.hold_mm = axis.debug.carriage_mm - axis.params.backoff_mm;
        axis.backoff_phase = flags == FAULT_OVERFORCE ? BACKOFF_BRAKING : BACKOFF_NONE;
        axis.integral_A = 0;
        axis.integral_duty = 0;
    }
    axis.debug.fault_flags |= flags;
    axis.debug.state = AXIS_FAULT;
}

void fw_init(void) {
    struct axis_context zero = {0};
    context = zero;
#ifdef AXIS_SIL
    probe_active = false;
#endif
    parameters_reset();
    force_guard_reset();
    previous_position_mm = 0;
    axis.safety_ok = true;
    seat_ticks = 0;
    protocol_reset();
}

void fw_get_debug(fw_debug_t *debug) {
    *debug = axis.debug;
}

void fw_step(const fw_inputs_t *inputs, fw_outputs_t *outputs) {
    float position_mm = (float)inputs->encoder_counts / axis.params.counts_per_mm;
    float raw_position_mm = position_mm;
    float velocity_mm_s;
    float desired_mm_s = 0;
    float error_mm_s;
    float trial_A;
    float current_A = 0;
    float alpha;
    float duty = 0;

    axis.debug.now_ms = inputs->now_ms;
    axis.safety_ok = inputs->estop_ok;
    axis.latest_code = inputs->adc_code;
    velocity_mm_s = (position_mm - previous_position_mm) * CONTROL_FREQUENCY_HZ;
    previous_position_mm = raw_position_mm;
    position_mm -= axis.encoder_zero_mm;
    alpha = TWO_PI * axis.params.velocity_filter_hz * CONTROL_PERIOD_S;
    alpha = alpha / (1 + alpha);
    axis.debug.vel_mm_s += alpha * (velocity_mm_s - axis.debug.vel_mm_s);
    axis.debug.carriage_mm = position_mm;
    force_guard_step(inputs);
    if (axis.debug.state != AXIS_MOVING) {
        seat_ticks = 0;
    }
    if (axis.debug.state == AXIS_MOVING) {
        if (!axis.tare_left) {
            desired_mm_s = axis.debug.server_est_mm < axis.params.slow_zone_mm
                ? axis.params.approach_speed_mm_s : axis.params.insert_speed_mm_s;
            if (axis.target_mm - axis.debug.server_est_mm <
                axis.debug.velocity_ref * axis.debug.velocity_ref /
                (2 * axis.params.accel_mm_s2)) {
                desired_mm_s = 0;
            }
            if (axis.debug.server_est_mm >= axis.target_mm) {
                axis.hold_mm = position_mm;
                axis.debug.state = AXIS_IDLE;
                desired_mm_s = 0;
            }
            if (axis.debug.server_est_mm >= axis.params.seat_lo_mm &&
                axis.debug.server_est_mm <= axis.params.seat_hi_mm &&
                axis.debug.force_N >= axis.params.seat_force_N) {
                seat_ticks++;
            } else {
                seat_ticks = 0;
            }
            if (seat_ticks >= SEAT_DEBOUNCE_TICKS) {
                axis.debug.state = AXIS_SEATED;
                desired_mm_s = 0;
                axis.integral_A = 0;
            }
        }
    } else if (axis.debug.state == AXIS_HOMING) {
        axis.home_travel_mm += absolute(velocity_mm_s) * CONTROL_PERIOD_S;
        if (!axis.home_braking) {
            axis.home_ticks = inputs->limit_switch ? axis.home_ticks + 1 : 0;
            if ((float)axis.home_ticks >= axis.params.home_debounce_ms) {
                axis.home_braking = true;
                axis.debug.velocity_ref = 0;
                axis.integral_A = 0;
                axis.integral_duty = 0;
            } else if (axis.home_travel_mm > axis.params.home_max_travel_mm) {
                trip(FAULT_HOME_TIMEOUT);
            } else {
                desired_mm_s = -axis.params.home_speed_mm_s;
            }
        }
        if (axis.home_braking && absolute(axis.debug.vel_mm_s) < STATIONARY_SPEED_MM_S) {
            /* Set zero at the stopped encoder coordinate, not at the first bouncing edge. */
            axis.encoder_zero_mm = raw_position_mm;
            position_mm = 0;
            axis.debug.carriage_mm = 0;
            axis.debug.server_est_mm = -axis.debug.force_N / axis.params.pusher_stiffness_N_per_mm;
            axis.target_mm = axis.params.home_backoff_mm;
            axis.debug.state = AXIS_HOME_BACKOFF;
            axis.integral_A = 0;
            axis.integral_duty = 0;
        }
    } else if (axis.debug.state == AXIS_HOME_BACKOFF) {
        desired_mm_s = clamp((axis.target_mm - position_mm) * axis.params.position_kp_per_s,
                             -axis.params.home_speed_mm_s, axis.params.home_speed_mm_s);
        /* Assumed completion tolerance: one encoder count and stationary tare speed. */
        if (absolute(axis.target_mm - position_mm) <= 1 / axis.params.counts_per_mm &&
            absolute(axis.debug.vel_mm_s) < STATIONARY_SPEED_MM_S && !inputs->limit_switch) {
            axis.hold_mm = axis.target_mm;
            axis.debug.state = AXIS_IDLE;
            desired_mm_s = 0;
        }
    } else if (axis.debug.state == AXIS_RETRACTING) {
        desired_mm_s = -axis.params.backoff_speed_mm_s;
        if (position_mm <= axis.target_mm) {
            axis.hold_mm = position_mm;
            axis.debug.state = AXIS_IDLE;
            desired_mm_s = 0;
        }
    } else if (axis.debug.state == AXIS_IDLE) {
        desired_mm_s = clamp((axis.hold_mm - position_mm) * axis.params.position_kp_per_s,
                             -axis.params.backoff_speed_mm_s, axis.params.backoff_speed_mm_s);
    } else if (axis.debug.state == AXIS_FAULT && axis.backoff_phase) {
        if (axis.backoff_phase == BACKOFF_BRAKING) {
            desired_mm_s = 0;
            if (absolute(axis.debug.vel_mm_s) < BACKOFF_BRAKED_SPEED_MM_S) {
                axis.backoff_phase = BACKOFF_REVERSING;
            }
        }
        if (axis.backoff_phase == BACKOFF_REVERSING) {
            desired_mm_s = -axis.params.backoff_speed_mm_s;
            if (position_mm <= axis.hold_mm) {
                axis.backoff_phase = BACKOFF_HOLDING;
                desired_mm_s = 0;
            }
        }
        if (axis.backoff_phase == BACKOFF_HOLDING) {
            desired_mm_s = clamp((axis.hold_mm - position_mm) * axis.params.position_kp_per_s,
                                 -axis.params.backoff_speed_mm_s, axis.params.backoff_speed_mm_s);
        }
    }
    if (axis.debug.state == AXIS_FAULT) {
        axis.debug.velocity_ref = desired_mm_s;
    } else {
        axis.debug.velocity_ref += clamp(desired_mm_s - axis.debug.velocity_ref,
                                        -axis.params.accel_mm_s2 * CONTROL_PERIOD_S,
                                        axis.params.accel_mm_s2 * CONTROL_PERIOD_S);
    }
#ifdef AXIS_SIL
    /* Test-only direct reference injection isolates the loop from profile slew. */
    if (probe_active && axis.debug.state == AXIS_MOVING) {
        axis.debug.velocity_ref = probe_reference_mm_s;
    }
#endif
    /* Compare feedback with the actual slewed reference in either drive mode.
       Reset persistence whenever motion, sign or either speed test disappears. */
    if ((axis.debug.state == AXIS_MOVING || axis.debug.state == AXIS_RETRACTING ||
         axis.debug.state == AXIS_HOMING || axis.debug.state == AXIS_HOME_BACKOFF) &&
        absolute(axis.debug.velocity_ref) > axis.params.wiring_check_mm_s &&
        absolute(axis.debug.vel_mm_s) > axis.params.wiring_check_mm_s &&
        axis.debug.velocity_ref * axis.debug.vel_mm_s < 0) {
        if (!axis.wiring_pending) {
            axis.wiring_since_ms = inputs->now_ms;
            axis.wiring_pending = true;
        } else if ((float)(inputs->now_ms - axis.wiring_since_ms) > axis.params.wiring_check_ms) {
            trip(FAULT_WIRING);
            axis.debug.velocity_ref = 0;
        }
    } else {
        axis.wiring_pending = false;
    }
    outputs->drive_enable = axis.enabled && axis.debug.state != AXIS_DISABLED &&
                            axis.debug.state != AXIS_ESTOP;
    if (outputs->drive_enable && axis.debug.state != AXIS_SEATED &&
        !(axis.debug.fault_flags & (FAULT_WIRING | FAULT_HOME_TIMEOUT)) && axis.params.drive_mode == 0) {
        error_mm_s = axis.debug.velocity_ref - axis.debug.vel_mm_s;
        trial_A = axis.integral_A + axis.params.velocity_ki_A_per_mm * error_mm_s * CONTROL_PERIOD_S;
        current_A = axis.params.velocity_kp_A_per_mm_s * error_mm_s + trial_A;
        /* Only integrate when it helps leave saturation or remains inside the clamp. */
        if (absolute(current_A) <= axis.params.max_current_A || current_A * error_mm_s < 0) {
            axis.integral_A = trial_A;
        }
        current_A = clamp(axis.params.velocity_kp_A_per_mm_s * error_mm_s + axis.integral_A,
                          -axis.params.max_current_A, axis.params.max_current_A);
    }
    if (outputs->drive_enable && axis.debug.state != AXIS_SEATED &&
        axis.params.drive_mode == 1 && !(axis.debug.fault_flags & (FAULT_WIRING | FAULT_HOME_TIMEOUT))) {
        /* Zero applied voltage shorts the modelled armature for passive dynamic
           braking. Never use reversed feedback to actively brake a wiring fault. */
        if (axis.debug.state == AXIS_FAULT &&
            (axis.backoff_phase == BACKOFF_NONE || axis.backoff_phase == BACKOFF_BRAKING)) {
            axis.integral_duty = 0;
        } else {
            float trial_duty;
            error_mm_s = axis.debug.velocity_ref - axis.debug.vel_mm_s;
            trial_duty = axis.integral_duty + axis.params.duty_ki * error_mm_s * CONTROL_PERIOD_S;
            duty = axis.params.duty_kp * error_mm_s + trial_duty;
            if (absolute(duty) <= 1 || duty * error_mm_s < 0) {
                axis.integral_duty = trial_duty;
            }
            duty = clamp(axis.params.duty_kp * error_mm_s + axis.integral_duty, -1, 1);
        }
    }
    if (axis.debug.state == AXIS_DISABLED || axis.debug.state == AXIS_ESTOP ||
        axis.debug.state == AXIS_SEATED) {
        axis.debug.velocity_ref = 0;
        axis.integral_A = 0;
        axis.integral_duty = 0;
    }
    outputs->current_cmd_A = current_A;
    outputs->pwm_duty = absolute(duty);
    outputs->dir = duty >= 0 ? 1 : 0;
    axis.debug.current_A = current_A;
    protocol_tick();
}
