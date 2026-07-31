#ifndef BALL_MECHANISM_LUT_H
#define BALL_MECHANISM_LUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BALL_LUT_REL_PULSE_MIN (-2500)
#define BALL_LUT_REL_PULSE_MAX (6908)
#define BALL_LUT_ENTRY_COUNT   (9409u)
#define BALL_LUT_A0_SCALE      (10)
#define BALL_LUT_COS_Q15_SCALE (32768u)

typedef struct
{
    /* gravity_accel_x10 / 10 = a0 in mm/s^2 */
    int16_t gravity_accel_x10;

    /* cos_theta_q15 / 32768 = cos(theta) */
    uint16_t cos_theta_q15;
} BallMechanismLutEntry;

extern const BallMechanismLutEntry
    g_ball_mechanism_lut[BALL_LUT_ENTRY_COUNT];

/*
 * No absolute zero is stored in this table.
 * Supply the calibrated horizontal coordinate at runtime:
 *
 *     absolute_target = zero_pulse + relative_pulse;
 */
static inline uint32_t BallMechanismLut_IndexFromRelative(
    int32_t relative_pulse)
{
    return (uint32_t)(relative_pulse - BALL_LUT_REL_PULSE_MIN);
}

static inline int32_t BallMechanismLut_RelativeFromIndex(uint32_t index)
{
    return (int32_t)index + BALL_LUT_REL_PULSE_MIN;
}

static inline int32_t BallMechanismLut_ClampRelative(int32_t relative_pulse)
{
    if (relative_pulse < BALL_LUT_REL_PULSE_MIN)
    {
        return BALL_LUT_REL_PULSE_MIN;
    }
    if (relative_pulse > BALL_LUT_REL_PULSE_MAX)
    {
        return BALL_LUT_REL_PULSE_MAX;
    }
    return relative_pulse;
}

static inline int32_t BallMechanismLut_ToAbsolute(
    int32_t zero_pulse, int32_t relative_pulse)
{
    return zero_pulse + BallMechanismLut_ClampRelative(relative_pulse);
}

#ifdef __cplusplus
}
#endif

#endif /* BALL_MECHANISM_LUT_H */
