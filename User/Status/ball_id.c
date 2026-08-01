#include "ball_id.h"
#include "ball_control.h"
#include "ball_mechanism_lut.h"
#include "status.h"
#include "uart_it.h"
#include "maixcam.h"
#include "log.h"
#include <math.h>

BALL_ID_STATE ball_id;

static const float scan_ratios[BALL_ID_SCAN_RATIOS] = {
    0.35f, 0.45f, 0.55f, 0.65f, 0.75f, 0.85f, 0.95f
};

static void id_abort(struct STATUS *status, uint8_t code);

static float id_absf(float x) { return x < 0.0f ? -x : x; }

static int32_t id_clamp_pulse(int32_t pulse) {
    if (pulse < BALL_ID_LOW_PULSE_LIMIT)  return BALL_ID_LOW_PULSE_LIMIT;
    if (pulse > BALL_ID_HIGH_PULSE_LIMIT) return BALL_ID_HIGH_PULSE_LIMIT;
    return pulse;
}

static uint8_t id_try_publish(int32_t absolute_pulse, uint32_t now_ms) {
    if (absolute_pulse == status.control.ball.stepper_profile.last_target_pulse)
        return 1;
    uint32_t seq_before = status.stepper.target.publish_seq;
    ball_stepper_shaped_request(&status, absolute_pulse, now_ms);
    return status.stepper.target.publish_seq != seq_before;
}

static void id_compute_run_params(uint8_t run_id, int8_t *side, float *ratio) {
    if (run_id < BALL_ID_SCAN_RATIOS) {
        *side = -1; *ratio = scan_ratios[run_id]; return;
    }
    if (run_id < BALL_ID_SCAN_RATIOS * 2u) {
        *side = 1; *ratio = scan_ratios[run_id - BALL_ID_SCAN_RATIOS]; return;
    }
    uint8_t fid = run_id - BALL_ID_SCAN_RATIOS * 2u;
    float lr = ball_id.low_move_ratio;
    float hr = ball_id.high_move_ratio;
    if (lr <= 0.0f || lr >= 0.99f) lr = 0.55f;
    if (hr <= 0.0f || hr >= 0.99f) hr = 0.55f;
    float low[3] = {lr, 0.5f*(lr+0.95f), 0.95f};
    float high[3] = {hr, 0.5f*(hr+0.95f), 0.95f};
    struct { int8_t s; uint8_t idx; } plan[12] = {
        {-1,0},{-1,1},{-1,2}, {1,0},{1,1},{1,2},
        {1,2},{1,1},{1,0}, {-1,2},{-1,1},{-1,0}
    };
    *side = plan[fid].s;
    *ratio = (*side < 0) ? low[plan[fid].idx] : high[plan[fid].idx];
}

static void id_buzzer_beep(STATUS *status, uint32_t dur) {
    status->device.buzzer.on = 1;
    status->device.buzzer.off_time = status->state.time + dur;
}

static void id_abort(STATUS *status, uint8_t code) {
    if (ball_id.phase_id == BALL_ID_PHASE_DONE) return; /* already aborting */
    ball_id.abort_code = code;
    id_try_publish(BALL_STEPPER_ZERO_PULSE, (uint32_t)status->state.time);
    ball_id.phase_id = BALL_ID_PHASE_DONE;
    ball_id.phase_start_ms = (uint32_t)status->state.time;
    ball_id.command_sent = 0;
    id_buzzer_beep(status, 600);
}

void ball_id_init(STATUS *status) {
    (void)status;
    ball_id.active = 0;
    ball_id.run_id = 0;
    ball_id.phase_id = 0;
    ball_id.abort_code = 0;
    ball_id.command_sent = 0;
    ball_id.side = 0;
    ball_id.test_ratio = 0.0f;
    ball_id.test_accel = 0.0f;
    ball_id.target_pulse = BALL_STEPPER_ZERO_PULSE;
    ball_id.run_x0_mm = 0.0f;
    ball_id.run_start_ms = 0;
    ball_id.phase_start_ms = 0;
    ball_id.vision_last_ms = 0;
    ball_id.baseline_frames = 0;
    ball_id.baseline_sum = 0.0f;
    ball_id.low_move_ratio = 0.0f;
    ball_id.high_move_ratio = 0.0f;
}

void ball_id_try_next(STATUS *status) {
    if (ball_id.active) return;

    uint8_t is_scan = (ball_id.run_id < BALL_ID_SCAN_RATIOS * 2u);
    uint8_t max_runs = is_scan ? (BALL_ID_SCAN_RATIOS * 2u)
                               : (BALL_ID_SCAN_RATIOS * 2u + BALL_ID_RUN_COUNT);
    if (ball_id.run_id >= max_runs) {
        /* All runs complete — do nothing, beep 3 short bursts */
        id_buzzer_beep(status, 60);
        return;
    }

    id_compute_run_params(ball_id.run_id, &ball_id.side, &ball_id.test_ratio);

    ball_id.active = 1;
    ball_id.phase_id = BALL_ID_PHASE_BASELINE;
    ball_id.abort_code = 0;
    ball_id.command_sent = 0;
    ball_id.run_start_ms = (uint32_t)status->state.time;
    ball_id.vision_last_ms = (uint32_t)status->state.time;
    ball_id.baseline_frames = 0;
    ball_id.baseline_sum = 0.0f;
    ball_id.run_x0_mm = 0.0f;

    maixcam_cmd_D(1);
    status->control.ball.stuck_timer_s = 0.0f;
    status->control.ball.integral_accel_mm_s2 = 0.0f;
    status->control.ball.hold_timer_s = 0.0f;
    status->control.ball.hold_active = 0;

    /* Force zero first, even if already there — ensure pipe is level */
    id_try_publish(BALL_STEPPER_ZERO_PULSE, (uint32_t)status->state.time);

    id_buzzer_beep(status, 80);
}

void ball_id_service(STATUS *status) {
    uint32_t now_ms = (uint32_t)status->state.time;
    if (!ball_id.active) return;

    /* Safety (skip if already in DONE / returning to zero) */
    float raw_mm = (float)status->sensor.vision.ball.x10 * 0.1f;
    if (ball_id.phase_id != BALL_ID_PHASE_DONE) {
        if (id_absf(raw_mm) >= BALL_ID_SAFETY_MM) {
            id_abort(status, 1);
        } else if (now_ms - ball_id.vision_last_ms > BALL_ID_VISION_TIMEOUT) {
            id_abort(status, 2);
        } else if (now_ms - ball_id.run_start_ms > BALL_ID_MAX_RUN_MS) {
            id_abort(status, 2);
        } else if (status->task.stop_request) {
            id_abort(status, 3);
        }
    }
    if (status->sensor.vision.ball.valid)
        ball_id.vision_last_ms = now_ms;

    /* ── 基于视觉帧差分计算球速 (ball_control_service 在 ID 期间不运行) ── */
    static uint32_t last_vel_seq;
    static float    last_vel_x_mm;
    static uint32_t last_vel_ts_ms;
    float vision_vel_mm_s = 0.0f;
    if (status->sensor.vision.ball.valid &&
        status->sensor.vision.ball.sample_seq != last_vel_seq) {
        uint32_t dt = status->sensor.vision.ball.timestamp_ms - last_vel_ts_ms;
        if (dt > 0u && dt <= 1000u && last_vel_ts_ms > 0u) {
            vision_vel_mm_s = (raw_mm - last_vel_x_mm) / ((float)dt * 0.001f);
        }
        last_vel_x_mm  = raw_mm;
        last_vel_ts_ms = status->sensor.vision.ball.timestamp_ms;
        last_vel_seq   = status->sensor.vision.ball.sample_seq;
    }

    int32_t zero_pulse = BALL_STEPPER_ZERO_PULSE;

    switch (ball_id.phase_id) {

    case BALL_ID_PHASE_BASELINE: {
        /* Keep publishing zero to ensure motor is actually there */
        id_try_publish(zero_pulse, now_ms);

        /* Collect 8 new vision frames while at zero */
        static uint32_t last_seq_bl;
        if (status->sensor.vision.ball.sample_seq != last_seq_bl) {
            last_seq_bl = status->sensor.vision.ball.sample_seq;
            ball_id.baseline_sum += raw_mm;
            ball_id.baseline_frames++;
        }
        if (ball_id.baseline_frames >= BALL_ID_BASELINE_FRAMES) {
            ball_id.run_x0_mm = ball_id.baseline_sum / (float)ball_id.baseline_frames;
            ball_id.phase_id = BALL_ID_PHASE_EXCITE;
            ball_id.phase_start_ms = now_ms;
            ball_id.command_sent = 0;

            float limit = (ball_id.side > 0) ? BALL_ACCEL_MAX : BALL_ACCEL_MIN;
            ball_id.test_accel = ball_id.test_ratio * limit;
            ball_id.target_pulse = id_clamp_pulse(ball_accel_to_absolute_pulse(ball_id.test_accel));
        }
        /* BASELINE 超时: 等不到足够视觉帧 → abort */
        if (now_ms - ball_id.run_start_ms > BALL_ID_BASELINE_TIMEOUT_MS) {
            id_abort(status, 2);
        }
        break;
    }

    case BALL_ID_PHASE_EXCITE: {
        if (!ball_id.command_sent) {
            if (id_try_publish(ball_id.target_pulse, now_ms))
                ball_id.command_sent = 1;
            break;
        }
        float dx_mm = id_absf(raw_mm - ball_id.run_x0_mm);
        if (dx_mm >= BALL_ID_EXCITE_MM ||
            id_absf(vision_vel_mm_s) >= BALL_ID_MOVE_SPEED_MM_S ||
            (status->stepper.reached && now_ms - ball_id.phase_start_ms >= 300u) ||
            now_ms - ball_id.phase_start_ms >= BALL_ID_EXCITE_TIMEOUT_MS) {

            uint8_t is_scan = (ball_id.run_id < BALL_ID_SCAN_RATIOS * 2u);
            if (is_scan && (dx_mm >= BALL_ID_EXCITE_MM ||
                            id_absf(vision_vel_mm_s) >= BALL_ID_MOVE_SPEED_MM_S)) {
                if (ball_id.side < 0 && ball_id.low_move_ratio <= 0.0f)
                    ball_id.low_move_ratio = ball_id.test_ratio;
                if (ball_id.side > 0 && ball_id.high_move_ratio <= 0.0f)
                    ball_id.high_move_ratio = ball_id.test_ratio;
            }
            ball_id.phase_id = BALL_ID_PHASE_RETURN_ZERO;
            ball_id.phase_start_ms = now_ms;
            ball_id.command_sent = 0;
        }
        break;
    }

    case BALL_ID_PHASE_RETURN_ZERO: {
        if (!ball_id.command_sent) {
            if (id_try_publish(zero_pulse, now_ms))
                ball_id.command_sent = 1;
        }
        if (status->stepper.reached || now_ms - ball_id.phase_start_ms >= BALL_ID_RETURN_ZERO_TIMEOUT_MS) {
            ball_id.phase_id = BALL_ID_PHASE_POST_RECORD;
            ball_id.phase_start_ms = now_ms;
        }
        break;
    }

    case BALL_ID_PHASE_POST_RECORD: {
        if (now_ms - ball_id.phase_start_ms >= BALL_ID_POST_RECORD_MS) {
            ball_id.phase_id = BALL_ID_PHASE_DONE;
            ball_id.phase_start_ms = now_ms;
            ball_id.command_sent = 0;
        }
        break;
    }

    case BALL_ID_PHASE_DONE: {
        id_try_publish(zero_pulse, now_ms);
        if (status->stepper.reached || now_ms - ball_id.phase_start_ms >= BALL_ID_DONE_TIMEOUT_MS) {
            ball_id.active = 0;
            if (ball_id.abort_code == 0) {
                ball_id.run_id++;  /* only advance on clean completion */
            }
            id_buzzer_beep(status, ball_id.abort_code ? 600 : 200);
        }
        break;
    }

    }
}

/* ── Log ── */
static volatile uint8_t  id_log_ready;
static volatile float    id_log_buf[20];

void ball_id_log_service(STATUS *status) {
    if (!ball_id.active) return;
    static uint32_t last_seq;
    if (status->sensor.vision.ball.sample_seq == last_seq) return;
    last_seq = status->sensor.vision.ball.sample_seq;

    float rmm = (float)status->sensor.vision.ball.x10 * 0.1f;
    BALL_CONTROL *ball = &status->control.ball;
    static uint32_t cam_t0;
    static uint32_t cam_t0_run_start;
    /* ball_id_service 先于 log_service 递增 baseline_frames, 因此用 run_start_ms 判断新 run */
    if (ball_id.phase_id == BALL_ID_PHASE_BASELINE &&
        ball_id.run_start_ms != cam_t0_run_start) {
        cam_t0 = status->sensor.vision.ball.timestamp_ms;
        cam_t0_run_start = ball_id.run_start_ms;
    }

    id_log_buf[0]  = (float)ball_id.run_id;
    id_log_buf[1]  = (float)ball_id.phase_id;
    id_log_buf[2]  = (float)(status->state.time - ball_id.run_start_ms);
    id_log_buf[3]  = (float)((int32_t)(status->sensor.vision.ball.timestamp_ms - cam_t0));
    id_log_buf[4]  = rmm;
    id_log_buf[5]  = ball_id.run_x0_mm;
    id_log_buf[6]  = rmm - ball_id.run_x0_mm;
    id_log_buf[7]  = (float)ball_id.side;
    id_log_buf[8]  = ball_id.test_accel;
    id_log_buf[9]  = ball_id.test_ratio;
    id_log_buf[10] = (float)(ball_id.target_pulse - (int32_t)BALL_STEPPER_ZERO_PULSE);
    id_log_buf[11] = (float)(ball->stepper_profile.last_target_pulse - (int32_t)BALL_STEPPER_ZERO_PULSE);
    id_log_buf[12] = (float)(status->stepper.tx.last_started_pulse - (int32_t)BALL_STEPPER_ZERO_PULSE);
    id_log_buf[13] = (float)status->stepper.tx.last_started_velocity;
    id_log_buf[14] = (float)status->stepper.tx.last_started_accel;
    id_log_buf[15] = (float)status->stepper.target.publish_seq;
    id_log_buf[16] = (float)status->stepper.tx.last_started_seq;
    id_log_buf[17] = (float)status->stepper.reached;
    id_log_buf[18] = (float)ball->stepper_profile.reverse_guard;
    id_log_buf[19] = (float)status->sensor.vision.ball.valid;
    id_log_ready = 1;
}

void ball_id_log_flush(void) {
    if (!id_log_ready) return;
    float local[20];
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0; i < 20; i++) local[i] = id_log_buf[i];
    id_log_ready = 0;
    if (!primask) __enable_irq();
    UART_send_justfloat(&huart1, 20,
        local[0], local[1], local[2], local[3],
        local[4], local[5], local[6], local[7],
        local[8], local[9], local[10], local[11],
        local[12], local[13], local[14], local[15],
        local[16], local[17], local[18], local[19]);
}
