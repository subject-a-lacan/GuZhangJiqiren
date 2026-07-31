#include "ball_control.h"
#include "ball_mechanism_lut.h"
#include "status.h"
#include "uart_it.h"

#if BALL_CONTROL_REL_PULSE_MIN < BALL_LUT_REL_PULSE_MIN || \
    BALL_CONTROL_REL_PULSE_MAX > BALL_LUT_REL_PULSE_MAX || \
    BALL_CONTROL_REL_PULSE_MIN > BALL_CONTROL_REL_PULSE_MAX
#error "Ball control pulse limits must stay inside the generated LUT range"
#endif

#define BALL_ROLLING_FACTOR (5.0f / 7.0f)

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

static int32_t ball_find_relative_pulse(float requested_accel_mm_s2,
                                        float car_accel_mm_s2) {
  float best_error = 3.402823466e+38f;
  int32_t first_pulse = BALL_CONTROL_REL_PULSE_MIN;
  int32_t last_pulse = BALL_CONTROL_REL_PULSE_MAX;
  int32_t best_pulse = first_pulse;

  for (int32_t relative_pulse = first_pulse;
       relative_pulse <= last_pulse; relative_pulse++) {
    const BallMechanismLutEntry *entry =
        &g_ball_mechanism_lut[BallMechanismLut_IndexFromRelative(relative_pulse)];
    float model_accel;
    float error;

    model_accel = (float)entry->gravity_accel_x10 / BALL_LUT_A0_SCALE -
                  BALL_ROLLING_FACTOR * car_accel_mm_s2 *
                      ((float)entry->cos_theta_q15 / BALL_LUT_COS_Q15_SCALE);
    error = model_accel - requested_accel_mm_s2;
    if (error < 0.0f) error = -error;
    if (error < best_error) {
      best_error = error;
      best_pulse = relative_pulse;
    }
  }

  return best_pulse;
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
  status->control.ball.position_error_mm = 0.0f;
  status->control.ball.requested_accel_mm_s2 = 0.0f;
  status->control.ball.relative_target_pulse = 0;
  status->control.ball.absolute_target_pulse = BALL_STEPPER_ZERO_PULSE;
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

  if (!request.enabled) {
    ball->estimator.ready = 0;
    ball->estimator.control_ready = 0;
    ball->consumed_request_seq = request.publish_seq;
    ball->consumed_session_seq = request.session_seq;
    stepper_target_disable();
    return;
  }

  if (request.session_seq != ball->consumed_session_seq) {
    ball->estimator.ready = 0;
    ball->estimator.control_ready = 0;
    ball->consumed_session_seq = request.session_seq;
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
        float dt_s = (float)dt_ms * 0.001f;
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
  ball->requested_accel_mm_s2 = ball->kp * ball->position_error_mm -
                                ball->kd * ball->estimator.velocity_mm_s;
  ball->relative_target_pulse =
      ball_find_relative_pulse(ball->requested_accel_mm_s2,
                               request.car_accel_mm_s2);
  ball->absolute_target_pulse =
      BALL_STEPPER_ZERO_PULSE + ball->relative_target_pulse;
  ball->consumed_request_seq = request.publish_seq;

  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (status->control.ball.request.enabled &&
        status->control.ball.request.publish_seq == request.publish_seq &&
        status->control.ball.request.session_seq == request.session_seq) {
      stepper_publish_absolute(ball->absolute_target_pulse,
                               BALL_STEPPER_VELOCITY,
                               BALL_STEPPER_ACCEL_PARAM);
    }
    if (!primask) __enable_irq();
  }
}
