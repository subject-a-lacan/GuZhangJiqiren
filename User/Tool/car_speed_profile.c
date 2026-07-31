#include "car_speed_profile.h"
#include <stddef.h>

/* ── 5th-order S-curve: q(0)=0, q(1)=1, zero velocity & accel at both ends ── */
/* q(tau)   = 10·τ³ − 15·τ⁴ + 6·τ⁵ */
/* dq(tau)  = 30·τ² − 60·τ³ + 30·τ⁴  (derivative w.r.t tau) */

static float scurve_q(float tau) {
    float t2 = tau * tau;
    float t3 = t2 * tau;
    float t4 = t3 * tau;
    float t5 = t4 * tau;
    return 10.0f * t3 - 15.0f * t4 + 6.0f * t5;
}

static float scurve_dq(float tau) {
    float t2 = tau * tau;
    float t3 = t2 * tau;
    float t4 = t3 * tau;
    return 30.0f * t2 - 60.0f * t3 + 30.0f * t4;
}

void car_speed_profile_init(CAR_SPEED_PROFILE *profile,
                            float sample_time_s, float mm_per_count) {
    if (profile == NULL) return;
    profile->enabled           = 0;
    profile->phase             = CAR_SPEED_IDLE;
    profile->start_time_ms     = 0;
    profile->sample_time_s     = sample_time_s;
    profile->mm_per_count      = mm_per_count;
    profile->cruise_speed_unit = 0.0f;
    profile->cruise_speed_mm_s = 0.0f;
    profile->ramp_distance_mm  = 0.0f;
    profile->ramp_time_s       = 0.0f;
    profile->progress          = 0.0f;
    profile->target_speed_unit = 0.0f;
    profile->target_speed_mm_s = 0.0f;
    profile->accel_mm_s2       = 0.0f;
    profile->measured_speed_unit = 0.0f;
    profile->measured_speed_mm_s = 0.0f;
    profile->sum_wheel_counts  = 0;
    profile->mileage_mm        = 0.0f;
    profile->ideal_ab_time_s   = 0.0f;
    profile->peak_accel_mm_s2  = 0.0f;
}

void car_speed_profile_reset(CAR_SPEED_PROFILE *profile) {
    if (profile == NULL) return;
    float saved_ts  = profile->sample_time_s;
    float saved_mmpc = profile->mm_per_count;
    car_speed_profile_init(profile, saved_ts, saved_mmpc);
}

uint8_t car_speed_profile_start(CAR_SPEED_PROFILE *profile,
                                uint32_t now_ms,
                                float cruise_speed_unit,
                                float ramp_distance_mm) {
    if (profile == NULL)           return 0;
    if (cruise_speed_unit <= 0.0f) return 0;
    if (ramp_distance_mm  <= 0.0f) return 0;

    /* V = U * mm_per_count / Ts  (mm/s) */
    float cruise_speed_mm_s =
        cruise_speed_unit * profile->mm_per_count / profile->sample_time_s;

    /* Tr = 2 * Sr / V  (since average speed of the S-curve is exactly V/2) */
    float ramp_time_s = 2.0f * ramp_distance_mm / cruise_speed_mm_s;

    profile->enabled           = 1;
    profile->phase             = CAR_SPEED_RAMP;
    profile->start_time_ms     = now_ms;
    profile->cruise_speed_unit = cruise_speed_unit;
    profile->cruise_speed_mm_s = cruise_speed_mm_s;
    profile->ramp_distance_mm  = ramp_distance_mm;
    profile->ramp_time_s       = ramp_time_s;
    profile->progress          = 0.0f;
    profile->target_speed_unit = 0.0f;
    profile->target_speed_mm_s = 0.0f;
    profile->accel_mm_s2       = 0.0f;
    profile->sum_wheel_counts  = 0;
    profile->mileage_mm        = 0.0f;

    /* theroetical AB time at cruise */
    /* 1500 mm / V — just a reference number, not used for control */
    profile->ideal_ab_time_s   = 1500.0f / cruise_speed_mm_s + ramp_time_s;

    /* peak dq = 1.875 at tau=0.5 */
    profile->peak_accel_mm_s2  = cruise_speed_mm_s / ramp_time_s * 1.875f;

    return 1;
}

void car_speed_profile_update_measurement(CAR_SPEED_PROFILE *profile,
    int16_t wheel0_count, int16_t wheel1_count,
    int16_t wheel2_count, int16_t wheel3_count) {
    if (profile == NULL) return;

    /* average count per wheel in this 5 ms interval */
    float average_count = ((float)wheel0_count + (float)wheel1_count +
                           (float)wheel2_count + (float)wheel3_count) / 4.0f;

    profile->measured_speed_unit = average_count;
    profile->measured_speed_mm_s =
        average_count * profile->mm_per_count / profile->sample_time_s;

    if (!profile->enabled) return;

    /* accumulate odometry directly from encoder counts (no trapezoid integration) */
    int64_t sum =
        (int64_t)wheel0_count + (int64_t)wheel1_count +
        (int64_t)wheel2_count + (int64_t)wheel3_count;
    profile->sum_wheel_counts += sum;
    profile->mileage_mm =
        (float)profile->sum_wheel_counts * profile->mm_per_count / 4.0f;
}

void car_speed_profile_step(CAR_SPEED_PROFILE *profile, uint32_t now_ms) {
    if (profile == NULL || !profile->enabled) return;

    if (profile->phase == CAR_SPEED_CRUISE) {
        /* hold steady */
        profile->progress          = 1.0f;
        profile->target_speed_unit = profile->cruise_speed_unit;
        profile->target_speed_mm_s = profile->cruise_speed_mm_s;
        profile->accel_mm_s2       = 0.0f;
        return;
    }

    /* ── RAMP phase ── */
    float elapsed_s = (float)(now_ms - profile->start_time_ms) * 0.001f;

    if (elapsed_s >= profile->ramp_time_s) {
        profile->phase             = CAR_SPEED_CRUISE;
        profile->progress          = 1.0f;
        profile->target_speed_unit = profile->cruise_speed_unit;
        profile->target_speed_mm_s = profile->cruise_speed_mm_s;
        profile->accel_mm_s2       = 0.0f;
        return;
    }

    float tau = elapsed_s / profile->ramp_time_s;
    if (tau < 0.0f) tau = 0.0f;
    if (tau > 1.0f) tau = 1.0f;

    float q_val  = scurve_q(tau);
    float dq_val = scurve_dq(tau);

    profile->progress          = tau;
    profile->target_speed_unit = profile->cruise_speed_unit * q_val;
    profile->target_speed_mm_s = profile->cruise_speed_mm_s * q_val;
    profile->accel_mm_s2 =
        profile->cruise_speed_mm_s / profile->ramp_time_s * dq_val;
}
