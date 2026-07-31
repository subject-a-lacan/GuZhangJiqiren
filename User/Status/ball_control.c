#include "ball_control.h"
#include "ball_mechanism_lut.h"
#include "status.h"
#include "uart_it.h"
#include <math.h>

#if BALL_CONTROL_REL_PULSE_MIN < BALL_LUT_REL_PULSE_MIN || \
    BALL_CONTROL_REL_PULSE_MAX > BALL_LUT_REL_PULSE_MAX || \
    BALL_CONTROL_REL_PULSE_MIN > BALL_CONTROL_REL_PULSE_MAX
#error "Ball control pulse limits must stay inside the generated LUT range"
#endif

#define BALL_ROLLING_FACTOR (5.0f / 7.0f)
#define BALL_GRAVITY_MM_S2 (9810.0f)
#define BALL_ROLLING_GRAVITY_MM_S2 \
  (BALL_ROLLING_FACTOR * BALL_GRAVITY_MM_S2)

typedef struct {
  uint8_t enabled;
  float target_mm;
  float car_accel_mm_s2;
  uint32_t publish_seq;
  uint32_t session_seq;
} BALL_REQUEST_SNAPSHOT;

static BALL_REQUEST_SNAPSHOT ball_request_snapshot(STATUS *status) {
  BALL_REQUEST_SNAPSHOT snapshot;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  snapshot.enabled = status->control.ball.request.enabled;
  snapshot.target_mm = status->control.ball.request.target_mm;
  snapshot.car_accel_mm_s2 = status->control.ball.request.car_accel_mm_s2;
  snapshot.publish_seq = status->control.ball.request.publish_seq;
  snapshot.session_seq = status->control.ball.request.session_seq;
  if (!primask) __enable_irq();
  return snapshot;
}

static float ball_model_accel(uint32_t index, float car_accel_mm_s2) {
  float gravity_accel =
      (float)g_ball_mechanism_lut[index].gravity_accel_x10 /
      BALL_LUT_A0_SCALE;

  if (car_accel_mm_s2 == 0.0f) return gravity_accel;

  /* Reconstruct cos(theta) from the same quantized gravity term.  Using the
     separately quantized Q15 column can introduce tiny local reversals in the
     otherwise descending full-range search key. */
  {
    float sin_theta = -gravity_accel / BALL_ROLLING_GRAVITY_MM_S2;
    float cos_squared = 1.0f - sin_theta * sin_theta;
    float cos_theta = sqrtf(cos_squared > 0.0f ? cos_squared : 0.0f);
    return gravity_accel -
           BALL_ROLLING_FACTOR * car_accel_mm_s2 * cos_theta;
  }
}

static int32_t ball_find_relative_pulse(float requested_accel_mm_s2,
                                        float car_accel_mm_s2) {
  uint32_t first_index = BallMechanismLut_IndexFromRelative(
      BALL_CONTROL_REL_PULSE_MIN);
  uint32_t end_index = BallMechanismLut_IndexFromRelative(
                           BALL_CONTROL_REL_PULSE_MAX) +
                       1u;
  uint32_t lo = first_index;
  uint32_t hi = end_index;

  if (car_accel_mm_s2 > BALL_CONTROL_CAR_ACCEL_LIMIT_MM_S2) {
    car_accel_mm_s2 = BALL_CONTROL_CAR_ACCEL_LIMIT_MM_S2;
  } else if (car_accel_mm_s2 < -BALL_CONTROL_CAR_ACCEL_LIMIT_MM_S2) {
    car_accel_mm_s2 = -BALL_CONTROL_CAR_ACCEL_LIMIT_MM_S2;
  }

  /* The model is descending with pulse. Find the first entry whose
     acceleration is no greater than the requested acceleration. */
  while (lo < hi) {
    uint32_t mid = lo + (hi - lo) / 2u;
    if (ball_model_accel(mid, car_accel_mm_s2) > requested_accel_mm_s2) {
      lo = mid + 1u;
    } else {
      hi = mid;
    }
  }

  if (lo == first_index) return BALL_CONTROL_REL_PULSE_MIN;
  if (lo == end_index) return BALL_CONTROL_REL_PULSE_MAX;

  {
    float previous_error =
        ball_model_accel(lo - 1u, car_accel_mm_s2) - requested_accel_mm_s2;
    float current_error =
        ball_model_accel(lo, car_accel_mm_s2) - requested_accel_mm_s2;
    if (previous_error < 0.0f) previous_error = -previous_error;
    if (current_error < 0.0f) current_error = -current_error;
    if (previous_error <= current_error) lo--;
  }

  return BallMechanismLut_RelativeFromIndex(lo);
}

/* value <= full → 1, value >= off → 0, linear in between */
static float ball_fall_ratio(float value, float full, float off) {
  if (value <= full) return 1.0f;
  if (value >= off)  return 0.0f;
  return (off - value) / (off - full);
}

static float ball_clampf(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

static uint8_t ball_stepper_accel_to_param(float accel_rpm_s) {
  if (accel_rpm_s < 79.0f) accel_rpm_s = 79.0f;
  float param = 256.0f - 20000.0f / accel_rpm_s;
  param = ball_clampf(param, 1.0f, 255.0f);
  return (uint8_t)(param + 0.5f);
}

typedef struct {
  uint8_t valid;
  int32_t target_pulse;
  uint16_t velocity_rpm;
  uint8_t accel_param;
  int8_t direction;
  uint8_t reverse_guard_after;
  float move_ratio;
} BALL_STEPPER_COMMAND;

static BALL_STEPPER_COMMAND ball_stepper_prepare_command(
    const BALL_CONTROL *ball, int32_t raw_target_pulse, uint32_t now_ms) {
  BALL_STEPPER_COMMAND cmd = {0};
  const BALL_STEPPER_PROFILE *profile = &ball->stepper_profile;

  int64_t delta64 = (int64_t)raw_target_pulse - (int64_t)profile->last_target_pulse;
  uint32_t distance = delta64 < 0 ? (uint32_t)(-delta64) : (uint32_t)delta64;
  int8_t direction = delta64 > 0 ? 1 : -1;
  uint8_t reversing = profile->last_direction != 0 &&
                      direction != profile->last_direction &&
                      distance >= BALL_STEPPER_REVERSE_MIN_PULSE;
  uint32_t elapsed_ms = now_ms - profile->last_publish_ms;

  if (distance <= BALL_STEPPER_DEADBAND_PULSE) return cmd;
  if (elapsed_ms < BALL_STEPPER_MIN_PUBLISH_MS &&
      distance < BALL_STEPPER_URGENT_PULSE && !reversing)
    return cmd;

  float ratio = ((float)distance - (float)BALL_STEPPER_DEADBAND_PULSE) /
                ((float)BALL_STEPPER_FULL_SPEED_PULSE - (float)BALL_STEPPER_DEADBAND_PULSE);
  ratio = ball_clampf(ratio, 0.0f, 1.0f);
  float smooth_ratio = ratio * ratio * (3.0f - 2.0f * ratio);

  float velocity_rpm = BALL_STEPPER_VEL_MIN_RPM +
                       (BALL_STEPPER_VEL_MAX_RPM - BALL_STEPPER_VEL_MIN_RPM) * smooth_ratio;
  float accel_rpm_s = BALL_STEPPER_ACCEL_MIN_RPM_S +
                      (BALL_STEPPER_ACCEL_MAX_RPM_S - BALL_STEPPER_ACCEL_MIN_RPM_S) * smooth_ratio;

  uint8_t guard = profile->reverse_guard;
  if (reversing) guard = BALL_STEPPER_REVERSE_GUARD_COUNT;
  if (guard > 0u) {
    if (velocity_rpm > BALL_STEPPER_REVERSE_MAX_RPM) velocity_rpm = BALL_STEPPER_REVERSE_MAX_RPM;
    if (accel_rpm_s > BALL_STEPPER_REVERSE_ACCEL_RPM_S) accel_rpm_s = BALL_STEPPER_REVERSE_ACCEL_RPM_S;
    guard--;
  }

  cmd.valid = 1;
  cmd.target_pulse = raw_target_pulse;
  cmd.velocity_rpm = (uint16_t)(velocity_rpm + 0.5f);
  cmd.accel_param = ball_stepper_accel_to_param(accel_rpm_s);
  cmd.direction = direction;
  cmd.reverse_guard_after = guard;
  cmd.move_ratio = smooth_ratio;
  return cmd;
}

static void ball_stepper_commit_command(BALL_CONTROL *ball,
    const BALL_STEPPER_COMMAND *cmd, uint32_t now_ms) {
  BALL_STEPPER_PROFILE *profile = &ball->stepper_profile;
  profile->last_target_pulse = cmd->target_pulse;
  profile->last_publish_ms = now_ms;
  profile->velocity_rpm = cmd->velocity_rpm;
  profile->accel_param = cmd->accel_param;
  profile->last_direction = cmd->direction;
  profile->reverse_guard = cmd->reverse_guard_after;
  profile->move_ratio = cmd->move_ratio;
}

void ball_control_init(STATUS *status) {
  status->sensor.vision.ball.x10 = 0;
  status->sensor.vision.ball.timestamp_ms = 0;
  status->sensor.vision.ball.sample_seq = 0;
  status->sensor.vision.ball.valid = 0;

  status->control.ball.request.enabled = 0;
  status->control.ball.request.target_mm = 0.0f;
  status->control.ball.request.car_accel_mm_s2 = 0.0f;
  status->control.ball.request.publish_seq = 0;
  status->control.ball.request.session_seq = 0;
  status->control.ball.estimator.ready = 0;
  status->control.ball.estimator.control_ready = 0;
  status->control.ball.estimator.consumed_sample_seq = 0;
  status->control.ball.estimator.last_timestamp_ms = 0;
  status->control.ball.estimator.position_mm = 0.0f;
  status->control.ball.estimator.velocity_mm_s = 0.0f;
  status->control.ball.estimator.alpha = BALL_ESTIMATOR_ALPHA;
  status->control.ball.estimator.beta = BALL_ESTIMATOR_BETA;
  status->control.ball.consumed_request_seq = 0;
  status->control.ball.consumed_session_seq = 0;
  status->control.ball.kp = BALL_CONTROL_KP;
  status->control.ball.kd = BALL_CONTROL_KD;
  status->control.ball.ki = BALL_CONTROL_KI;
  status->control.ball.position_error_mm = 0.0f;
  status->control.ball.requested_accel_mm_s2 = 0.0f;
  status->control.ball.relative_target_pulse = 0;
  status->control.ball.absolute_target_pulse = BALL_STEPPER_ZERO_PULSE;
  status->control.ball.stuck_timer_s = 0.0f;
  status->control.ball.integral_accel_mm_s2 = 0.0f;
  status->control.ball.hold_timer_s = 0.0f;
  status->control.ball.hold_active = 0;

  status->control.ball.consumed_target_mm = 0.0f;
  status->control.ball.stepper_profile.last_target_pulse = BALL_STEPPER_ZERO_PULSE;
  status->control.ball.stepper_profile.last_publish_ms = 0u;
  status->control.ball.stepper_profile.velocity_rpm = 0u;
  status->control.ball.stepper_profile.accel_param =
      ball_stepper_accel_to_param(BALL_STEPPER_ACCEL_MIN_RPM_S);
  status->control.ball.stepper_profile.last_direction = 0;
  status->control.ball.stepper_profile.reverse_guard = 0u;
  status->control.ball.stepper_profile.move_ratio = 0.0f;
}

void ball_control_request(STATUS *status, float target_mm,
                          float car_accel_mm_s2) {
  BALL_CONTROL_REQUEST *request = &status->control.ball.request;
  uint32_t primask = __get_PRIMASK();
  uint8_t changed;

  __disable_irq();
  changed = !request->enabled || request->target_mm != target_mm ||
            request->car_accel_mm_s2 != car_accel_mm_s2;
  if (changed) {
    if (!request->enabled) request->session_seq++;
    request->target_mm = target_mm;
    request->car_accel_mm_s2 = car_accel_mm_s2;
    request->enabled = 1;
    __DMB();
    request->publish_seq++;
  }
  if (!primask) __enable_irq();
}

void ball_control_disable(STATUS *status) {
  BALL_CONTROL_REQUEST *request = &status->control.ball.request;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (request->enabled) {
    request->enabled = 0;
    __DMB();
    request->publish_seq++;
  }
  if (!primask) __enable_irq();
  stepper_target_disable();
}

void ball_control_service(STATUS *status) {
  BALL_CONTROL *ball = &status->control.ball;
  VISION_BALL *vision = &status->sensor.vision.ball;
  BALL_REQUEST_SNAPSHOT request = ball_request_snapshot(status);
  uint8_t request_changed = request.publish_seq != ball->consumed_request_seq;
  uint8_t vision_changed = vision->sample_seq != ball->estimator.consumed_sample_seq;
  float dt_s = 0.0f;

  if (!request.enabled) {
    ball->estimator.ready = 0;
    ball->estimator.control_ready = 0;
    ball->consumed_request_seq = request.publish_seq;
    ball->consumed_session_seq = request.session_seq;
    ball->stuck_timer_s = 0.0f;
    ball->integral_accel_mm_s2 = 0.0f;
    ball->hold_timer_s = 0.0f;
    ball->hold_active = 0;
    stepper_target_disable();
    return;
  }

  if (request.session_seq != ball->consumed_session_seq) {
    ball->estimator.ready = 0;
    ball->estimator.control_ready = 0;
    ball->stuck_timer_s = 0.0f;
    ball->integral_accel_mm_s2 = 0.0f;
    ball->hold_timer_s = 0.0f;
    ball->hold_active = 0;
    ball->consumed_target_mm = request.target_mm;
    ball->stepper_profile.last_publish_ms =
        (uint32_t)status->state.time - BALL_STEPPER_MIN_PUBLISH_MS;
    ball->stepper_profile.last_direction = 0;
    ball->stepper_profile.reverse_guard = 0u;
    ball->consumed_session_seq = request.session_seq;
  }

  if (request.target_mm != ball->consumed_target_mm) {
    ball->stuck_timer_s = 0.0f;
    ball->integral_accel_mm_s2 = 0.0f;
    ball->hold_timer_s = 0.0f;
    ball->hold_active = 0;
    ball->consumed_target_mm = request.target_mm;
  }

  if (vision_changed && vision->valid) {
    float measured_position_mm = (float)vision->x10 * 0.1f;
    uint32_t timestamp_ms = vision->timestamp_ms;

    ball->estimator.consumed_sample_seq = vision->sample_seq;
    if (!ball->estimator.ready) {
      ball->estimator.ready = 1;
      ball->estimator.last_timestamp_ms = timestamp_ms;
      ball->estimator.position_mm = measured_position_mm;
      ball->estimator.velocity_mm_s = 0.0f;
      ball->estimator.control_ready = 0;
      ball->consumed_request_seq = request.publish_seq;
      return;
    } else {
      uint32_t dt_ms = timestamp_ms - ball->estimator.last_timestamp_ms;
      if (dt_ms == 0u || dt_ms > BALL_ESTIMATOR_MAX_DT_MS) {
        ball->estimator.last_timestamp_ms = timestamp_ms;
        ball->estimator.position_mm = measured_position_mm;
        ball->estimator.velocity_mm_s = 0.0f;
        ball->estimator.control_ready = 0;
        ball->consumed_request_seq = request.publish_seq;
        return;
      } else {
        dt_s = (float)dt_ms * 0.001f;
        float predicted_position = ball->estimator.position_mm +
                                   ball->estimator.velocity_mm_s * dt_s;
        float residual = measured_position_mm - predicted_position;
        ball->estimator.position_mm = predicted_position +
                                      ball->estimator.alpha * residual;
        ball->estimator.velocity_mm_s +=
            (ball->estimator.beta / dt_s) * residual;
        ball->estimator.last_timestamp_ms = timestamp_ms;
        ball->estimator.control_ready = 1;
      }
    }
  } else if (!request_changed || !ball->estimator.control_ready) {
    return;
  }

  ball->position_error_mm = request.target_mm - ball->estimator.position_mm;

  {
    float e_abs = fabsf(ball->position_error_mm);
    float v_abs = fabsf(ball->estimator.velocity_mm_s);

    /* ── Deadband with hysteresis ── */
    if (ball->hold_active) {
      if (e_abs >= BALL_DEAD_EXIT_ERROR_MM ||
          v_abs >= BALL_DEAD_EXIT_SPEED_MM_S) {
        ball->hold_active = 0;
        ball->hold_timer_s = 0.0f;
      }
    } else {
      if (e_abs <= BALL_DEAD_ENTER_ERROR_MM &&
          v_abs <= BALL_DEAD_ENTER_SPEED_MM_S) {
        ball->hold_timer_s += dt_s;
      } else {
        /* Slowly decrease timer instead of resetting to zero */
        ball->hold_timer_s -= 2.0f * dt_s;
        if (ball->hold_timer_s < 0.0f)
          ball->hold_timer_s = 0.0f;
      }
      if (ball->hold_timer_s >= BALL_DEAD_CONFIRM_S) {
        ball->hold_active = 1;
        ball->integral_accel_mm_s2 = 0.0f;
      }
    }

    if (ball->hold_active) {
      /* In deadband: zero ball accel, clear integral */
      ball->stuck_timer_s = 0.0f;
      ball->requested_accel_mm_s2 = 0.0f;
    } else {
      float pd_accel = ball->kp * ball->position_error_mm -
                       ball->kd * ball->estimator.velocity_mm_s;

      uint8_t moving_toward =
          (ball->position_error_mm * ball->estimator.velocity_mm_s) > 0.0f;

      if (e_abs <= BALL_DEAD_ENTER_ERROR_MM) {
        /* Already inside allowed range, release integral */
        ball->stuck_timer_s = 0.0f;
        ball->integral_accel_mm_s2 = 0.0f;
      } else if (moving_toward &&
                 v_abs >= BALL_I_RELEASE_SPEED_MM_S &&
                 dt_s > 0.0f) {
        /* Ball moving toward target: fast decay, integral done its job */
        float keep = 1.0f - BALL_I_DECAY_PER_S * dt_s;
        if (keep < 0.0f) keep = 0.0f;
        ball->stuck_timer_s = 0.0f;
        ball->integral_accel_mm_s2 *= keep;
      } else {
        /* Ball slow or stuck: conditional integral */
        float error_scale = ball_fall_ratio(e_abs, BALL_I_FULL_ERROR_MM,
                                            BALL_I_OFF_ERROR_MM);
        float speed_scale = ball_fall_ratio(v_abs, BALL_I_FULL_SPEED_MM_S,
                                            BALL_I_OFF_SPEED_MM_S);
        float integral_scale = error_scale * speed_scale;

        if (integral_scale > 0.0f && dt_s > 0.0f) {
          ball->stuck_timer_s += dt_s;
          if (ball->stuck_timer_s >= BALL_I_CONFIRM_S) {
            ball->integral_accel_mm_s2 += ball->ki * integral_scale *
                                          ball->position_error_mm * dt_s;
            if (ball->integral_accel_mm_s2 > BALL_I_MAX_MM_S2)
              ball->integral_accel_mm_s2 = BALL_I_MAX_MM_S2;
            else if (ball->integral_accel_mm_s2 < -BALL_I_MAX_MM_S2)
              ball->integral_accel_mm_s2 = -BALL_I_MAX_MM_S2;
          }
        } else if (dt_s > 0.0f) {
          /* Not integrating: slowly decay old integral */
          float keep = 1.0f - BALL_I_DECAY_PER_S * dt_s;
          if (keep < 0.0f) keep = 0.0f;
          ball->stuck_timer_s = 0.0f;
          ball->integral_accel_mm_s2 *= keep;
          if (fabsf(ball->integral_accel_mm_s2) < 0.05f)
            ball->integral_accel_mm_s2 = 0.0f;
        }
      }

      ball->requested_accel_mm_s2 = pd_accel + ball->integral_accel_mm_s2;
    }
  }

  ball->relative_target_pulse =
      ball_find_relative_pulse(ball->requested_accel_mm_s2,
                               request.car_accel_mm_s2);
  ball->absolute_target_pulse =
      BALL_STEPPER_ZERO_PULSE + ball->relative_target_pulse;
  ball->consumed_request_seq = request.publish_seq;

  {
    uint32_t now_ms = (uint32_t)status->state.time;
    BALL_STEPPER_COMMAND cmd = ball_stepper_prepare_command(
        ball, ball->absolute_target_pulse, now_ms);

    if (cmd.valid) {
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      if (status->control.ball.request.enabled &&
          status->control.ball.request.publish_seq == request.publish_seq &&
          status->control.ball.request.session_seq == request.session_seq) {
        stepper_publish_absolute(cmd.target_pulse, cmd.velocity_rpm,
                                 cmd.accel_param);
        ball_stepper_commit_command(ball, &cmd, now_ms);
      }
      if (!primask) __enable_irq();
    }
  }
}

uint8_t ball_stepper_shaped_request(STATUS *status,
    int32_t raw_absolute_pulse, uint32_t now_ms) {
  BALL_CONTROL *ball = &status->control.ball;
  BALL_STEPPER_COMMAND cmd = ball_stepper_prepare_command(
      ball, raw_absolute_pulse, now_ms);
  if (!cmd.valid) return 0;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  stepper_publish_absolute(cmd.target_pulse, cmd.velocity_rpm,
                           cmd.accel_param);
  ball_stepper_commit_command(ball, &cmd, now_ms);
  if (!primask) __enable_irq();
  return 1;
}

int32_t ball_accel_to_absolute_pulse(float accel_mm_s2) {
  int32_t relative = ball_find_relative_pulse(accel_mm_s2, 0.0f);
  return BALL_STEPPER_ZERO_PULSE + relative;
}
