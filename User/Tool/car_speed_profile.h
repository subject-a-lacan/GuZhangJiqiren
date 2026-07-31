#ifndef __CAR_SPEED_PROFILE_H
#define __CAR_SPEED_PROFILE_H

#include <stdint.h>

/* ── Wheel / encoder constants (single source of truth) ── */
#define ENCODER_PPR         (13)
#define ENCODER_QUADRATURE  (4)
#define GEAR_RATIO          (28.0f)
#define WHEEL_DIAMETER_MM   (66.0f)    /* tune to actual wheel */
#define WHEEL_CIRCUMFERENCE_MM (3.1415926f * WHEEL_DIAMETER_MM)
#define COUNTS_PER_WHEEL_REV ((float)(ENCODER_PPR * ENCODER_QUADRATURE) * GEAR_RATIO)
#define MM_PER_COUNT        (WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_WHEEL_REV)

typedef enum {
    CAR_SPEED_IDLE = 0,
    CAR_SPEED_RAMP,
    CAR_SPEED_CRUISE
} CAR_SPEED_PHASE;

typedef struct {
    uint8_t enabled;
    CAR_SPEED_PHASE phase;

    uint32_t start_time_ms;

    float sample_time_s;   /* = 0.005, separate from PID.T */
    float mm_per_count;

    float cruise_speed_unit;   /* count / 5 ms */
    float cruise_speed_mm_s;   /* mm / s */

    float ramp_distance_mm;
    float ramp_time_s;         /* Tr = 2*Sr/V */

    float progress;            /* 0..1 */
    float target_speed_unit;   /* current S-curve target, count/5ms */
    float target_speed_mm_s;   /* current S-curve target, mm/s   */
    float accel_mm_s2;         /* analytical acceleration, mm/s²   */

    float measured_speed_unit; /* 4-wheel average, count/5ms */
    float measured_speed_mm_s; /* 4-wheel average, mm/s    */

    int64_t sum_wheel_counts;  /* accumulated raw counts    */
    float mileage_mm;          /* accumulated distance, mm  */

    float ideal_ab_time_s;
    float peak_accel_mm_s2;
} CAR_SPEED_PROFILE;

/* ── API ── */

void car_speed_profile_init(CAR_SPEED_PROFILE *profile,
                            float sample_time_s, float mm_per_count);

void car_speed_profile_reset(CAR_SPEED_PROFILE *profile);

/* Returns 0 if params invalid (no blocking, no storing to statics). */
uint8_t car_speed_profile_start(CAR_SPEED_PROFILE *profile,
                                uint32_t now_ms,
                                float cruise_speed_unit,
                                float ramp_distance_mm);

/* Call after all four get_wheel_speed() in the 5 ms ISR. */
void car_speed_profile_update_measurement(CAR_SPEED_PROFILE *profile,
    int16_t wheel0_count, int16_t wheel1_count,
    int16_t wheel2_count, int16_t wheel3_count);

/* Call once per 5 ms from the task that owns this profile. */
void car_speed_profile_step(CAR_SPEED_PROFILE *profile, uint32_t now_ms);

#endif
