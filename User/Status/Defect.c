#include "Defect.h"
#include "status.h"

#include <stdbool.h>

#include "log.h"
#include "pid.h"
#include "math_tool.h"

// ── TASK_BASIC_1 状态机静态变量（与 xiao timer_it.c 完全一致）──
static uint64_t junction_ignore_until = 0;
static int     lost_line_cnt      = 0;
static int     line_seen_cnt      = 0;
static float   turn_dir           = 0.0f;
static int     corner_count       = 0;
static uint64_t stop_enter_time   = 0;
static bool    startup_turn_pending = true;
static bool    initial_turn_done  = false;
static bool    initial_corner_counted = false;
static int     initial_turn_ticks = 0;

// ── TASK_BASIC_2 AB 发车 状态机静态变量 ──
static uint8_t  q2_phase_init_done   = 0;
static float    q2_phase_dist0_mm    = 0.0f;
static uint64_t q2_phase_t0          = 0;
static int      q2_lost_cnt          = 0;
static int      q2_line_cnt          = 0;
static int      q2_inplace_ticks     = 0;
static uint64_t q2_ignore_until      = 0;
static uint64_t q2_stop_t0           = 0;
static bool     q2_startup_pending   = true;

// ── 差速输出（与 xiao 完全一致）──

static void diff_drive(STATUS *s, float base, float diff, float limit) {
  diff = CONFINE(diff, -limit, limit);
  int16_t ds = (int16_t)diff;
  s->motor.wheel[0].tar_speed = base + ds;
  s->motor.wheel[1].tar_speed = base - ds;
}

static void find_line(STATUS *s) {
  float d = s->sensor.gw_analogue.diff;
  float out = compute_pid(&s->state.status_pid.follow_line_pid, d);
  diff_drive(s, s->state.base_speed, out, LINE_DIFF_LIMIT);
}

static float junction_yaw_offset(Road r) {
  switch (r) {
  case LeftRoad:   return -90.0f;
  case RightRoad:  return  90.0f;
  default:         return   0.0f;
  }
}

static void reset_cross_state(STATUS *s) {
  s->sensor.gw_analogue.cross.cross    = Straight;
  s->sensor.gw_analogue.cross.maybe    = 0;
  s->sensor.gw_analogue.cross.integral = 0;
}

static void enter_keep_angle(STATUS *s, float target_yaw) {
  s->state.turn.target_yaw = target_yaw;
  s->state.motion = KEEP_ANGLE;
  s->state.status_pid.keep_angle_pid.integral   = 0;
  s->state.status_pid.keep_angle_pid.last_error = 0;
  s->state.status_pid.keep_angle_pid.is_first   = 1;
  line_seen_cnt = 0;
}

static void back_to_find_line(STATUS *s) {
  s->state.motion = FIND_LINE;
  junction_ignore_until = s->state.time + COOLDOWN_MS;
  lost_line_cnt   = 0;
  line_seen_cnt   = 0;
  turn_dir        = 0.0f;
  s->state.status_pid.follow_line_pid.integral   = 0;
  s->state.status_pid.follow_line_pid.last_error = 0;
  s->state.status_pid.follow_line_pid.is_first   = 1;
  stop_enter_time = 0;
  reset_cross_state(s);
}

// ── Q2 辅助函数 ──

static void q2_set_phase(STATUS *s, uint8_t next_phase) {
  s->task.race_phase = next_phase;
  q2_phase_init_done = 0;
}

static void q2_phase_init_once(STATUS *s) {
  if (q2_phase_init_done) return;
  q2_phase_init_done = 1;
  q2_phase_dist0_mm  = s->motor.wheel[0].distance_mm;
  q2_phase_t0        = s->state.time;
  q2_lost_cnt        = 0;
  q2_line_cnt        = 0;
  q2_inplace_ticks   = 0;
  q2_stop_t0         = 0;
  reset_cross_state(s);
  s->state.status_pid.follow_line_pid.integral   = 0;
  s->state.status_pid.follow_line_pid.last_error = 0;
  s->state.status_pid.follow_line_pid.is_first   = 1;
  s->state.status_pid.keep_angle_pid.integral    = 0;
  s->state.status_pid.keep_angle_pid.last_error  = 0;
  s->state.status_pid.keep_angle_pid.is_first    = 1;
}

static float q2_phase_mm(STATUS *s) {
  return s->motor.wheel[0].distance_mm - q2_phase_dist0_mm;
}

// 是否在 BC/CB 干扰段（按里程窗口）
static bool q2_in_guard_window(STATUS *s) {
  uint8_t p = s->task.race_phase;
  if (p != Q2_AB_SIDE_CB_GUARD && p != Q2_AB_SIDE_BC_GUARD_RET &&
      p != Q2_AD_SIDE_CB_GUARD && p != Q2_AD_SIDE_BC_GUARD_RET) return false;
  float mm = q2_phase_mm(s);
  return (mm >= Q2_BC_GUARD_START_MM && mm <= Q2_BC_GUARD_END_MM);
}

// 路口字节模式更新（仅由 q2_run_inplace_turn 调用，只管 q2_line_cnt）
// q2_lost_cnt 由 task_basic_2_ab_update 顶部统一更新（与 Q1 对齐），不在这里碰
static void q2_update_line_cnt(STATUS *s) {
  uint8_t line = s->sensor.gw_analogue.digital_8bit;
  if (line == 0x00) {
    q2_line_cnt = 0;
  } else {
    q2_line_cnt++;
  }
}

// 进入原地转弯（包括起点 -90 / 路口 -90 / 路口 +90 / B 点 +180）
static void q2_enter_inplace_turn(STATUS *s, float yaw_delta) {
  s->state.turn.target_yaw = s->state.cur_angle + yaw_delta;
  s->state.motion = KEEP_ANGLE;
  q2_inplace_ticks = 0;
  q2_line_cnt = 0;
  q2_lost_cnt = 0;
  s->state.status_pid.keep_angle_pid.integral   = 0;
  s->state.status_pid.keep_angle_pid.last_error = 0;
  s->state.status_pid.keep_angle_pid.is_first   = 1;
}

// 执行原地差速转弯一帧；返回 1 表示完成（角度+视线均已就位）
// dir_sign: -1 左转 / +1 右转
// min_ticks: 最少持续 tick（线计数在此之前不生效，避免半路压线误判）
// min_line_back: 重新看到线 N 帧才算完成
// use_pid: 1 = 闭环 PID + 前进基速（90° 弯，曲线穿过，敏捷）
//          0 = 开环原地转 INIT_TURN_DIFF（Q1 起步风格，180° 掉头和起点）
// use_pre_brake: 1 = 进入相位先 wheels=0 减速 PRE_TURN_BRAKE_TICKS 帧（仅 B 点 180° 掉头使用）
//                0 = 直接转，与 Q1 KEEP_ANGLE 对齐
#define Q2_PRE_TURN_BRAKE_TICKS 10  // 200ms @ 20ms tick
static int q2_run_inplace_turn(STATUS *s, float dir_sign, int min_ticks, int min_line_back,
                                uint8_t use_pid, uint8_t use_pre_brake) {
  q2_inplace_ticks++;

  // 转弯前减速段（仅 180° 掉头使用，避免高速冲过 B 点）
  if (use_pre_brake && q2_inplace_ticks <= Q2_PRE_TURN_BRAKE_TICKS) {
    s->motor.wheel[0].tar_speed = 0;
    s->motor.wheel[1].tar_speed = 0;
    q2_line_cnt = 0;
    return 0;
  }

  if (use_pid) {
    float err = s->state.turn.target_yaw - s->state.cur_angle;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    if (ABS(err) < ERR_THRESHOLD) {
      diff_drive(s, TURN_BASE_SPEED, dir_sign * SEARCH_DIFF, SEARCH_DIFF);
    } else {
      float out = compute_pid(&s->state.status_pid.keep_angle_pid, err);
      diff_drive(s, TURN_BASE_SPEED, out, TURN_DIFF_LIMIT);
    }
  } else {
    // Q1 起步风格：纯开环原地转，简单可靠，不会抽搐
    diff_drive(s, 0, dir_sign * INIT_TURN_DIFF, INIT_TURN_DIFF);
  }

  // 必须先转够 min_ticks 才开始检测视线（避免 180° 中途压线误判）
  int rotate_ticks = use_pre_brake ? (q2_inplace_ticks - Q2_PRE_TURN_BRAKE_TICKS) : q2_inplace_ticks;
  if (rotate_ticks < min_ticks) {
    q2_line_cnt = 0;
    return 0;
  }
  q2_update_line_cnt(s);
  return (q2_line_cnt >= min_line_back) ? 1 : 0;
}

// 进入下一段巡线：恢复 PID，设置冷却避免立即触发路口
static void q2_back_to_find_line(STATUS *s, uint32_t cooldown_ms) {
  s->state.motion = FIND_LINE;
  q2_ignore_until = s->state.time + cooldown_ms;
  q2_lost_cnt = 0;
  q2_line_cnt = 0;
  s->state.status_pid.follow_line_pid.integral   = 0;
  s->state.status_pid.follow_line_pid.last_error = 0;
  s->state.status_pid.follow_line_pid.is_first   = 1;
  reset_cross_state(s);
}

// 路口识别（Q1 风格）：line 非空 + cross.cross 是 LeftRoad/RightRoad。
// 返回 1 表示检测到路口；调用方负责切到对应的原地转弯相位。
// 关键：cross 卡在 CrossRoad/TBRoad/TLRoad/TRRoad/UnknowRoad 时主动复位重试，
// 与 Q1 的 FIND_LINE 行为对齐，避免出弯后卡死导致下一个路口漏检。
static int q2_try_detect_junction(STATUS *s) {
  uint8_t line = s->sensor.gw_analogue.digital_8bit;
  if (line == 0x00) return 0;
  Road r = s->sensor.gw_analogue.cross.cross;
  if (r == Straight) return 0;
  if (r != LeftRoad && r != RightRoad) {
    reset_cross_state(s);
    return 0;
  }
  reset_cross_state(s);
  return 1;
}

// ── 任务分发 ──

void init_task(TASK *task) {
  task->task_id = TASK_BASIC_1;
  task->start_pose = START_AB;
  task->race_phase = 0;
  task->cross_cnt = 0;
  task->armed = 0;
  task->task_running = 0;
  task->task_select_request = 0;
  task->requested_task_id = 0;
  task->pose_switch_request = 0;
  task->start_request = 0;
  task->stop_request = 0;
  task->phase_start_time = 0;
  task->phase_mileage = 0;
}

void task_start(STATUS *status) {
  status->task.start_request = 0;
  status->task.stop_request = 0;
  status->task.cross_cnt = 0;

  // 重置路口检测
  reset_cross_state(status);

  status->task.phase_mileage = 0;

  // 重置 PID
  status->state.status_pid.follow_line_pid.error = 0;
  status->state.status_pid.follow_line_pid.last_error = 0;
  status->state.status_pid.follow_line_pid.integral = 0;
  status->state.status_pid.follow_line_pid.derivative = 0;
  status->state.status_pid.follow_line_pid.out = 0;
  status->state.status_pid.follow_line_pid.is_first = 1;

  status->state.status_pid.keep_angle_pid.error = 0;
  status->state.status_pid.keep_angle_pid.last_error = 0;
  status->state.status_pid.keep_angle_pid.integral = 0;
  status->state.status_pid.keep_angle_pid.derivative = 0;
  status->state.status_pid.keep_angle_pid.out = 0;
  status->state.status_pid.keep_angle_pid.is_first = 1;

  // 刷新初始角度
  status->state.initial_angle = status->state.cur_angle;

  status->motor.wheel[0].tar_speed = 0;
  status->motor.wheel[1].tar_speed = 0;

  switch (status->task.task_id) {
    case TASK_BASIC_1:
      status->task.race_phase = 0;
      status->state.motion = FIND_LINE;
      break;
    case TASK_BASIC_2:
      // 清里程基准
      status->motor.wheel[0].total_counts = 0;
      status->motor.wheel[0].distance_mm  = 0.0f;
      status->motor.wheel[1].total_counts = 0;
      status->motor.wheel[1].distance_mm  = 0.0f;
      // Q2 静态变量复位
      q2_phase_init_done = 0;
      q2_lost_cnt        = 0;
      q2_line_cnt        = 0;
      q2_inplace_ticks   = 0;
      q2_ignore_until    = 0;
      q2_stop_t0         = 0;
      q2_startup_pending = true;
      // 起始 phase 按发车位姿区分
      if (status->task.start_pose == START_AB) {
        status->task.race_phase = Q2_AB_START_A_TURN;
      } else {
        status->task.race_phase = Q2_AD_WAIT_START;
      }
      status->state.motion = STOP;
      break;
    case TASK_ADV_1:
      status->task.race_phase = 0;
      break;
    case TASK_ADV_2:
      status->task.race_phase = 0;
      break;
  }

  status->task.phase_start_time = status->state.time;

  status->state.base_speed = 0;
}

void task_finish(STATUS *status) {
  status->task.task_running = 0;
  status->task.armed = 0;
  status->task.start_request = 0;
  status->task.stop_request = 0;
  status->task.task_select_request = 0;
  status->task.pose_switch_request = 0;
  status->state.motion = STOP;
  status->state.base_speed = 0;
  status->state.motion_bypass = 0;
  status->motor.wheel[0].tar_speed = 0;
  status->motor.wheel[1].tar_speed = 0;
  status->device.buzzer.on = 1;
  status->device.buzzer.off_time = status->state.time + 200;
}

void task_stop(STATUS *status) {
  status->task.task_running = 0;
  status->task.armed = 0;
  status->task.start_request = 0;
  status->task.stop_request = 0;
  status->state.motion = STOP;
  status->state.base_speed = 0;
  status->state.motion_bypass = 0;
  status->motor.wheel[0].tar_speed = 0;
  status->motor.wheel[1].tar_speed = 0;
}

void task_select(STATUS *status, uint8_t id) {
  if (id < TASK_BASIC_1 || id > TASK_ADV_2) return;
  status->task.task_id = id;
  if (id == TASK_BASIC_1 || id == TASK_ADV_2) {
    status->task.start_pose = START_AB;
  }
  update_task_led(status);
}

// ── 各任务 update 函数 ──

static void task_basic_1_update(STATUS *s) {
  // 首次进入时标记任务开始
  if (!s->task.task_running) {
    s->task.task_running = 1;
    junction_ignore_until = 0;
    lost_line_cnt      = 0;
    line_seen_cnt      = 0;
    turn_dir           = 0.0f;
    corner_count       = 0;
    initial_turn_done  = false;
    initial_corner_counted = false;
    initial_turn_ticks = 0;
    stop_enter_time    = 0;
    startup_turn_pending = true;
    s->state.base_speed = 50;
  }

  // 本任务直接控制轮速，跳过 motion_execute
  s->state.motion_bypass = 1;

  uint8_t line = s->sensor.gw_analogue.digital_8bit;
  float   yaw  = s->state.cur_angle;

  // ── 丢线/有线计数 ──
  if (line == 0x00) {
    lost_line_cnt++;
    line_seen_cnt = 0;
  } else {
    lost_line_cnt = 0;
    line_seen_cnt++;
  }

  // ── key2 长按释放启动门控 ──
  if (startup_turn_pending && s->task.startup_release) {
    s->task.startup_release = 0;
    startup_turn_pending = false;
    s->device.buzzer.on = 1;
    s->device.buzzer.off_time = s->state.time + 500;
  }

  // ── 状态机（与 xiao timer_it.c 完全一致）──
  switch (s->state.motion) {

  case FIND_LINE: {
    // 冷却期：继续巡线，只跳过路口检测
    if (s->state.time < junction_ignore_until) {
      find_line(s);
      break;
    }

    // 等待 key2 长按启动
    if (startup_turn_pending) {
      s->motor.wheel[0].tar_speed = 0;
      s->motor.wheel[1].tar_speed = 0;
      break;
    }

    // 长按后首次 → 起点转弯
    if (!initial_turn_done) {
      initial_turn_done = true;
      turn_dir = -90.0f;
      s->state.turn.entry_yaw = yaw;
      lost_line_cnt = 0;
      initial_turn_ticks = 0;
      corner_count = 1;
      enter_keep_angle(s, yaw + turn_dir);
      break;
    }

    // 丢线 → 启动转弯
    if (lost_line_cnt >= LOST_LINE_TRIG) {
      turn_dir = -90.0f;
      s->state.turn.entry_yaw = yaw;
      lost_line_cnt = 0;
      corner_count++;
      if (corner_count >= 5) {
        s->motor.wheel[0].tar_speed = 0;
        s->motor.wheel[1].tar_speed = 0;
        s->state.motion = STOP;
        break;
      }
      enter_keep_angle(s, yaw + turn_dir);
      break;
    }

    // 有信号 → 巡线 + 路口检测
    if (line != 0x00) {
      Road r = s->sensor.gw_analogue.cross.cross;

      if (r != Straight) {
        turn_dir = junction_yaw_offset(r);
        if (turn_dir != 0.0f) {
          s->state.turn.entry_yaw = yaw;
          corner_count++;
          if (corner_count >= 4) {
            s->motor.wheel[0].tar_speed = 0;
            s->motor.wheel[1].tar_speed = 0;
            s->state.motion = STOP;
            break;
          }
          enter_keep_angle(s, yaw + turn_dir);
          break;
        }
        reset_cross_state(s);
      }

      find_line(s);
    }
    break;
  }

  case KEEP_ANGLE: {
    float err = s->state.turn.target_yaw - yaw;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    // 初始转弯：原地转，灰度看到线就切巡线
    if (!initial_corner_counted) {
      float dir = (turn_dir < 0) ? -1.0f : 1.0f;
      diff_drive(s, 0, dir * INIT_TURN_DIFF, INIT_TURN_DIFF);
      initial_turn_ticks++;
      if (initial_turn_ticks >= 5 && line_seen_cnt >= 3) {
        initial_corner_counted = true;
        back_to_find_line(s);
      }
      break;
    }

    // 正常转弯：看到线就切回巡线
    if (line_seen_cnt >= LINE_BACK_CNT) {
      back_to_find_line(s);
      break;
    }

    // 角度到位但没线 → 继续朝搜线方向转
    if (ABS(err) < ERR_THRESHOLD && turn_dir != 0.0f) {
      float dir = (turn_dir < 0) ? -1.0f : 1.0f;
      diff_drive(s, TURN_BASE_SPEED, dir * SEARCH_DIFF, SEARCH_DIFF);
    } else {
      float out = compute_pid(&s->state.status_pid.keep_angle_pid, err);
      diff_drive(s, TURN_BASE_SPEED, out, TURN_DIFF_LIMIT);
    }
    break;
  }

  case STOP:
    if (stop_enter_time == 0) {
      stop_enter_time = s->state.time;
    }
    if (s->state.time - stop_enter_time < 300) {
      s->motor.wheel[0].tar_speed = -50;
      s->motor.wheel[1].tar_speed = -50;
    } else {
      s->motor.wheel[0].tar_speed = 0;
      s->motor.wheel[1].tar_speed = 0;
    }
    break;

  case MOTOR_TEST:
    break;
  }
}

static void task_basic_2_ab_update(STATUS *s) {
  // 首次进入：标记任务运行 + 接管轮速
  if (!s->task.task_running) {
    s->task.task_running = 1;
    s->state.motion_bypass = 1;
    s->state.base_speed = 50;
    q2_phase_init_done = 0;
    q2_startup_pending = true;
    q2_ignore_until = 0;
  }

  // ── 统一更新丢线计数（与 Q1 task_basic_1_update 顶部一致）──
  // 直道相位通过 q2_lost_cnt >= LOST_LINE_TRIG 兜底转弯，避免漏路口
  // 注：q2_line_cnt 仍由 q2_run_inplace_turn 内部维护，互不冲突
  uint8_t line = s->sensor.gw_analogue.digital_8bit;
  if (line == 0x00) {
    q2_lost_cnt++;
  } else {
    q2_lost_cnt = 0;
  }

  switch (s->task.race_phase) {

  case Q2_AB_START_A_TURN: {
    q2_phase_init_once(s);
    // 等待 key2 长按释放门控
    if (q2_startup_pending) {
      if (s->task.startup_release) {
        s->task.startup_release = 0;
        q2_startup_pending = false;
        s->device.buzzer.on = 1;
        s->device.buzzer.off_time = s->state.time + 500;
        q2_enter_inplace_turn(s, -90.0f);
      } else {
        s->motor.wheel[0].tar_speed = 0;
        s->motor.wheel[1].tar_speed = 0;
        break;
      }
    }
    if (q2_run_inplace_turn(s, -1.0f, 5, LINE_BACK_CNT, 0, 0)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      q2_set_phase(s, Q2_AB_SIDE_AD);
    }
    break;
  }

  case Q2_AB_SIDE_AD: {
    q2_phase_init_once(s);
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_enter_inplace_turn(s, -90.0f);
      q2_set_phase(s, Q2_AB_TURN_D);
      break;
    }
    if (q2_try_detect_junction(s)) {
      q2_enter_inplace_turn(s, -90.0f);
      q2_set_phase(s, Q2_AB_TURN_D);
    }
    break;
  }

  case Q2_AB_TURN_D: {
    q2_phase_init_once(s);
    if (q2_run_inplace_turn(s, -1.0f, 5, LINE_BACK_CNT, 1, 0)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      q2_set_phase(s, Q2_AB_SIDE_DC);
    }
    break;
  }

  case Q2_AB_SIDE_DC: {
    q2_phase_init_once(s);
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_enter_inplace_turn(s, -90.0f);
      q2_set_phase(s, Q2_AB_TURN_C);
      break;
    }
    if (q2_try_detect_junction(s)) {
      q2_enter_inplace_turn(s, -90.0f);
      q2_set_phase(s, Q2_AB_TURN_C);
    }
    break;
  }

  case Q2_AB_TURN_C: {
    q2_phase_init_once(s);
    if (q2_run_inplace_turn(s, -1.0f, 5, LINE_BACK_CNT, 1, 0)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      // 进入干扰段，降速
      s->state.base_speed = 35;
      q2_set_phase(s, Q2_AB_SIDE_CB_GUARD);
    }
    break;
  }

  case Q2_AB_SIDE_CB_GUARD: {
    q2_phase_init_once(s);
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    // 干扰窗口内：清掉 cross 状态 + 抑制丢线兜底（干扰纸会假性丢线）
    if (q2_in_guard_window(s)) {
      reset_cross_state(s);
      q2_lost_cnt = 0;
      break;
    }
    // 丢线兜底：到 B 点找不到路口时强制 180°
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_enter_inplace_turn(s, 180.0f);
      q2_set_phase(s, Q2_AB_TURN_180);
      break;
    }
    // 路口识别 = 到达 B 点，入弯前减速段会接管刹车，无需独立 B_REACHED
    if (q2_try_detect_junction(s)) {
      q2_enter_inplace_turn(s, 180.0f);
      q2_set_phase(s, Q2_AB_TURN_180);
    }
    break;
  }

  case Q2_AB_TURN_180: {
    q2_phase_init_once(s);
    // 180° 掉头：开环原地转 + 200ms 预减速（仅此处保留），min_ticks 加大确保过 180°
    if (q2_run_inplace_turn(s, +1.0f, 30, LINE_BACK_CNT, 0, 1)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      s->state.base_speed = 35;
      q2_set_phase(s, Q2_AB_SIDE_BC_GUARD_RET);
    }
    break;
  }

  case Q2_AB_SIDE_BC_GUARD_RET: {
    q2_phase_init_once(s);
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    // 干扰窗口内：清掉 cross 状态 + 抑制丢线兜底
    if (q2_in_guard_window(s)) {
      reset_cross_state(s);
      q2_lost_cnt = 0;
      break;
    }
    // 丢线兜底：到 C 点找不到路口时强制 +90°
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_enter_inplace_turn(s, +90.0f);
      q2_set_phase(s, Q2_AB_TURN_C_RET);
      break;
    }
    if (q2_try_detect_junction(s)) {
      q2_enter_inplace_turn(s, +90.0f);
      q2_set_phase(s, Q2_AB_TURN_C_RET);
    }
    break;
  }

  case Q2_AB_TURN_C_RET: {
    q2_phase_init_once(s);
    if (q2_run_inplace_turn(s, +1.0f, 5, LINE_BACK_CNT, 1, 0)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      s->state.base_speed = 50;
      q2_set_phase(s, Q2_AB_SIDE_CD);
    }
    break;
  }

  case Q2_AB_SIDE_CD: {
    q2_phase_init_once(s);
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_enter_inplace_turn(s, +90.0f);
      q2_set_phase(s, Q2_AB_TURN_D_RET);
      break;
    }
    if (q2_try_detect_junction(s)) {
      q2_enter_inplace_turn(s, +90.0f);
      q2_set_phase(s, Q2_AB_TURN_D_RET);
    }
    break;
  }

  case Q2_AB_TURN_D_RET: {
    q2_phase_init_once(s);
    if (q2_run_inplace_turn(s, +1.0f, 5, LINE_BACK_CNT, 1, 0)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      q2_set_phase(s, Q2_AB_SIDE_DA_FINAL);
    }
    break;
  }

  case Q2_AB_SIDE_DA_FINAL: {
    q2_phase_init_once(s);
    // 已触发刹车：反向 -50 制动 300ms 后任务完成
    if (q2_stop_t0 != 0) {
      s->motor.wheel[0].tar_speed = -50;
      s->motor.wheel[1].tar_speed = -50;
      if (s->state.time - q2_stop_t0 >= 300) {
        task_finish(s);
      }
      break;
    }
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    // 丢线兜底：到 A 点附近丢线即停
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_stop_t0 = s->state.time;
      break;
    }
    // 路口识别 = 到达 A 点 → 启动反向制动
    if (q2_try_detect_junction(s)) {
      q2_stop_t0 = s->state.time;
      break;
    }
    // 兜底：超出最大里程强制刹车
    if (q2_phase_mm(s) >= Q2_FINAL_FORCE_MM) {
      q2_stop_t0 = s->state.time;
    }
    break;
  }

  default:
    break;
  }
}

static void task_basic_2_ad_update(STATUS *s) {
  // 首次进入：标记任务运行 + 接管轮速
  if (!s->task.task_running) {
    s->task.task_running = 1;
    s->state.motion_bypass = 1;
    s->state.base_speed = 50;
    q2_phase_init_done = 0;
    q2_startup_pending = true;
    q2_ignore_until = 0;
  }

  // ── 统一更新丢线计数（与 AB 发车一致）──
  uint8_t line = s->sensor.gw_analogue.digital_8bit;
  if (line == 0x00) {
    q2_lost_cnt++;
  } else {
    q2_lost_cnt = 0;
  }

  switch (s->task.race_phase) {

  case Q2_AD_WAIT_START: {
    q2_phase_init_once(s);
    // 等待 key2 长按释放门控；车头已朝 D，无需起点 -90 转弯
    if (q2_startup_pending) {
      if (s->task.startup_release) {
        s->task.startup_release = 0;
        q2_startup_pending = false;
        s->device.buzzer.on = 1;
        s->device.buzzer.off_time = s->state.time + 500;
        // 直接切回巡线，进入 AD 边
        q2_back_to_find_line(s, COOLDOWN_MS);
        q2_set_phase(s, Q2_AD_SIDE_AD);
      } else {
        s->motor.wheel[0].tar_speed = 0;
        s->motor.wheel[1].tar_speed = 0;
      }
    }
    break;
  }

  case Q2_AD_SIDE_AD: {
    q2_phase_init_once(s);
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_enter_inplace_turn(s, -90.0f);
      q2_set_phase(s, Q2_AD_TURN_D);
      break;
    }
    if (q2_try_detect_junction(s)) {
      q2_enter_inplace_turn(s, -90.0f);
      q2_set_phase(s, Q2_AD_TURN_D);
    }
    break;
  }

  case Q2_AD_TURN_D: {
    q2_phase_init_once(s);
    if (q2_run_inplace_turn(s, -1.0f, 5, LINE_BACK_CNT, 1, 0)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      q2_set_phase(s, Q2_AD_SIDE_DC);
    }
    break;
  }

  case Q2_AD_SIDE_DC: {
    q2_phase_init_once(s);
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_enter_inplace_turn(s, -90.0f);
      q2_set_phase(s, Q2_AD_TURN_C);
      break;
    }
    if (q2_try_detect_junction(s)) {
      q2_enter_inplace_turn(s, -90.0f);
      q2_set_phase(s, Q2_AD_TURN_C);
    }
    break;
  }

  case Q2_AD_TURN_C: {
    q2_phase_init_once(s);
    if (q2_run_inplace_turn(s, -1.0f, 5, LINE_BACK_CNT, 1, 0)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      // 进入干扰段，降速
      s->state.base_speed = 35;
      q2_set_phase(s, Q2_AD_SIDE_CB_GUARD);
    }
    break;
  }

  case Q2_AD_SIDE_CB_GUARD: {
    q2_phase_init_once(s);
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    // 干扰窗口内：清掉 cross 状态 + 抑制丢线兜底
    if (q2_in_guard_window(s)) {
      reset_cross_state(s);
      q2_lost_cnt = 0;
      break;
    }
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_enter_inplace_turn(s, 180.0f);
      q2_set_phase(s, Q2_AD_TURN_180);
      break;
    }
    if (q2_try_detect_junction(s)) {
      q2_enter_inplace_turn(s, 180.0f);
      q2_set_phase(s, Q2_AD_TURN_180);
    }
    break;
  }

  case Q2_AD_TURN_180: {
    q2_phase_init_once(s);
    // 180° 掉头：开环原地转 + 200ms 预减速，min_ticks 加大确保过 180°
    if (q2_run_inplace_turn(s, +1.0f, 30, LINE_BACK_CNT, 0, 1)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      s->state.base_speed = 35;
      q2_set_phase(s, Q2_AD_SIDE_BC_GUARD_RET);
    }
    break;
  }

  case Q2_AD_SIDE_BC_GUARD_RET: {
    q2_phase_init_once(s);
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    if (q2_in_guard_window(s)) {
      reset_cross_state(s);
      q2_lost_cnt = 0;
      break;
    }
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_enter_inplace_turn(s, +90.0f);
      q2_set_phase(s, Q2_AD_TURN_C_RET);
      break;
    }
    if (q2_try_detect_junction(s)) {
      q2_enter_inplace_turn(s, +90.0f);
      q2_set_phase(s, Q2_AD_TURN_C_RET);
    }
    break;
  }

  case Q2_AD_TURN_C_RET: {
    q2_phase_init_once(s);
    if (q2_run_inplace_turn(s, +1.0f, 5, LINE_BACK_CNT, 1, 0)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      s->state.base_speed = 50;
      q2_set_phase(s, Q2_AD_SIDE_CD);
    }
    break;
  }

  case Q2_AD_SIDE_CD: {
    q2_phase_init_once(s);
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_enter_inplace_turn(s, +90.0f);
      q2_set_phase(s, Q2_AD_TURN_D_RET);
      break;
    }
    if (q2_try_detect_junction(s)) {
      q2_enter_inplace_turn(s, +90.0f);
      q2_set_phase(s, Q2_AD_TURN_D_RET);
    }
    break;
  }

  case Q2_AD_TURN_D_RET: {
    q2_phase_init_once(s);
    if (q2_run_inplace_turn(s, +1.0f, 5, LINE_BACK_CNT, 1, 0)) {
      q2_back_to_find_line(s, COOLDOWN_MS);
      q2_set_phase(s, Q2_AD_SIDE_DA_FINAL);
    }
    break;
  }

  case Q2_AD_SIDE_DA_FINAL: {
    q2_phase_init_once(s);
    // 已触发刹车：反向 -50 制动 300ms 后任务完成
    if (q2_stop_t0 != 0) {
      s->motor.wheel[0].tar_speed = -50;
      s->motor.wheel[1].tar_speed = -50;
      if (s->state.time - q2_stop_t0 >= 300) {
        task_finish(s);
      }
      break;
    }
    find_line(s);
    if (s->state.time < q2_ignore_until) {
      q2_lost_cnt = 0;
      break;
    }
    // 丢线兜底：到 A 点附近丢线即停
    if (q2_lost_cnt >= LOST_LINE_TRIG) {
      q2_stop_t0 = s->state.time;
      break;
    }
    // 路口识别 = 到达 A 点 → 启动反向制动
    if (q2_try_detect_junction(s)) {
      q2_stop_t0 = s->state.time;
      break;
    }
    // 兜底：超出最大里程强制刹车
    if (q2_phase_mm(s) >= Q2_FINAL_FORCE_MM) {
      q2_stop_t0 = s->state.time;
    }
    break;
  }

  default:
    break;
  }
}

static void task_basic_2_update(STATUS *status) {
  if (status->task.start_pose == START_AB) {
    task_basic_2_ab_update(status);
  } else {
    task_basic_2_ad_update(status);
  }
}

static void task_adv_1_update(STATUS *status) {
  status->task.task_running = 1;
  task_finish(status);
}

static void task_adv_2_update(STATUS *status) {
  status->task.task_running = 1;
  task_finish(status);
}

// ── LED 反馈 ──

void update_task_led(STATUS *status) {
  switch (status->task.task_id) {
    case TASK_BASIC_1:
      status->device.led_on_board.on = 1;
      status->device.led1.on = 0;
      status->device.led2.on = 1;
      break;
    case TASK_BASIC_2:
      if (status->task.start_pose == START_AB) {
        status->device.led_on_board.on = 1;
        status->device.led1.on = 1;
        status->device.led2.on = 1;
      } else {
        status->device.led_on_board.on = 0;
        status->device.led1.on = 1;
        status->device.led2.on = 1;
      }
      break;
    case TASK_ADV_1:
      if (status->task.start_pose == START_AB) {
        status->device.led_on_board.on = 1;
        status->device.led1.on = 0;
        status->device.led2.on = 0;
      } else {
        status->device.led_on_board.on = 0;
        status->device.led1.on = 0;
        status->device.led2.on = 0;
      }
      break;
    case TASK_ADV_2:
      status->device.led_on_board.on = 1;
      status->device.led1.on = 1;
      status->device.led2.on = 0;
      break;
  }
}

// ── 任务调度入口 ──

void update_task(STATUS *status) {
  if (status->task.stop_request) {
    task_stop(status);
    return;
  }

  if (!status->task.task_running && !status->task.armed) {
    if (status->task.task_select_request) {
      task_select(status, status->task.requested_task_id);
      status->task.task_select_request = 0;
    }

    if (status->task.pose_switch_request) {
      if (status->task.task_id == TASK_BASIC_2 || status->task.task_id == TASK_ADV_1) {
        status->task.start_pose = (status->task.start_pose == START_AB) ? START_AD : START_AB;
        update_task_led(status);
      }
      status->task.pose_switch_request = 0;
    }
  }

  if (status->task.start_request) {
    if (!status->task.task_running && !status->task.armed) {
      status->task.armed = 1;
      task_start(status);
    }
    status->task.start_request = 0;
  }

  if (!status->task.armed) {
    return;
  }

  switch (status->task.task_id) {
    case TASK_BASIC_1:
      task_basic_1_update(status);
      break;
    case TASK_BASIC_2:
      task_basic_2_update(status);
      break;
    case TASK_ADV_1:
      task_adv_1_update(status);
      break;
    case TASK_ADV_2:
      task_adv_2_update(status);
      break;
  }
}
