#ifndef __BALL_ID_H
#define __BALL_ID_H

#include <stdint.h>

struct STATUS;

/* ── Experiment limits ── */
#define BALL_ID_LOW_PULSE_LIMIT   (-2500)
#define BALL_ID_HIGH_PULSE_LIMIT  (+2200)

/* Accel range for LUT coverage */
#define BALL_ACCEL_MIN  (-500.0f)
#define BALL_ACCEL_MAX  (+700.0f)

#define BALL_ID_BASELINE_FRAMES  8u
#define BALL_ID_EXCITE_MM        20.0f
#define BALL_ID_MOVE_SPEED_MM_S  15.0f
#define BALL_ID_POST_RECORD_MS   600u
#define BALL_ID_SAFETY_MM        65.0f
/* ── 超时参数 (放宽以适应 MaixCAM WiFi 掉帧) ── */
#define BALL_ID_VISION_TIMEOUT       8000u   /* 视觉帧最大间隔 ms (原200→8000)   */
#define BALL_ID_MAX_RUN_MS           60000u  /* 单次实验总超时 ms (原12000→60000) */
#define BALL_ID_BASELINE_TIMEOUT_MS  15000u  /* BASELINE 阶段超时 ms (原3000→15000) */
#define BALL_ID_EXCITE_TIMEOUT_MS    5000u   /* EXCITE 阶段最大等待 ms (原1500→5000)   */
#define BALL_ID_RETURN_ZERO_TIMEOUT_MS 5000u /* RETURN_ZERO 超时 ms (原2000→5000)    */
#define BALL_ID_DONE_TIMEOUT_MS      8000u   /* DONE 阶段超时 ms (原3000→8000)         */

/* Scan: 7 ratios for threshold detection */
#define BALL_ID_SCAN_RATIOS 7u
/* Formal: 3 levels × 2 sides × 2 passes = 12 */
#define BALL_ID_RUN_COUNT 12u

#define BALL_ID_PHASE_BASELINE     0
#define BALL_ID_PHASE_EXCITE       1
#define BALL_ID_PHASE_RETURN_ZERO  2
#define BALL_ID_PHASE_POST_RECORD  3
#define BALL_ID_PHASE_DONE         4

/* ── State ── */
typedef struct {
    uint8_t  active;
    uint8_t  run_id;        /* 0..25 (14 scan + 12 formal) */
    uint8_t  phase_id;      /* BASELINE/EXCITE/RETURN/POST/DONE */
    uint8_t  abort_code;
    uint8_t  command_sent;

    int8_t   side;          /* +1 or -1 */
    float    test_ratio;    /* 0.35 ~ 0.95 */
    float    test_accel;    /* side * ratio * accel_limit */
    int32_t  target_pulse;  /* LUT result */

    float    run_x0_mm;
    uint32_t run_start_ms;
    uint32_t phase_start_ms;
    uint32_t vision_last_ms;
    uint32_t baseline_frames;
    float    baseline_sum;

    /* Threshold results from scan */
    float    low_move_ratio;
    float    high_move_ratio;
} BALL_ID_STATE;

extern BALL_ID_STATE ball_id;

void ball_id_init(struct STATUS *status);
void ball_id_service(struct STATUS *status);
void ball_id_log_service(struct STATUS *status);
void ball_id_log_flush(void);
void ball_id_try_next(struct STATUS *status);

#endif
