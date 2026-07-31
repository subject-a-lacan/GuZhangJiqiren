#ifndef __BALL_CONTROL_H
#define __BALL_CONTROL_H

#include <stdint.h>

struct STATUS;

/* Mechanical/tuning parameters. Keep zero separate from the relative LUT. */
#define BALL_STEPPER_ZERO_PULSE (3399)  /* 47.8 deg */
/* Full generated LUT range, one entry per pulse. */
#define BALL_CONTROL_REL_PULSE_MIN (-2500)
#define BALL_CONTROL_REL_PULSE_MAX (6908)
#define BALL_CONTROL_CAR_ACCEL_LIMIT_MM_S2 (9810.0f)
#define BALL_STEPPER_VELOCITY (50u)
#define BALL_STEPPER_ACCEL_PARAM (0u)

#define BALL_CONTROL_KP (4.0f)
#define BALL_CONTROL_KD (4.0f)
#define BALL_ESTIMATOR_ALPHA (0.6f)
#define BALL_ESTIMATOR_BETA (0.1f)
#define BALL_ESTIMATOR_MAX_DT_MS (200u)

void ball_control_init(struct STATUS *status);
void ball_control_request(struct STATUS *status, float target_mm,
                          float car_accel_mm_s2);
void ball_control_disable(struct STATUS *status);
void ball_control_service(struct STATUS *status);

#endif
