#include "maixcam.h"
#include "usart.h"
#include "log.h"
#include <stdio.h>
#include <stddef.h>

#define MAIXCAM_TX_QUEUE_DEPTH 8
#define MAIXCAM_TX_FRAME_MAX 128

typedef struct {
  uint8_t data[MAIXCAM_TX_FRAME_MAX];
  uint16_t len;
} MaixcamTxItem;

static MaixcamTxItem maixcam_tx_queue[MAIXCAM_TX_QUEUE_DEPTH];
static volatile uint8_t maixcam_tx_head;
static volatile uint8_t maixcam_tx_tail;
static volatile uint8_t maixcam_tx_busy;

/* 模块全局变量 */
MaixcamRxRing        maixcam_rx;
VisionDetectionData  maixcam_det;
VisionLocationData   maixcam_loc;
MaixcamCmdReq        maixcam_cmd;
uint8_t              maixcam_det_new;
uint8_t              maixcam_loc_new;
char                 maixcam_rx_raw_buf[64];
uint8_t              maixcam_rx_raw_ready;

/* ── 环形缓冲 ── */

void maixcam_init(void) {
  maixcam_rx.head     = 0;
  maixcam_rx.tail     = 0;
  maixcam_rx.overflow = 0;

  maixcam_det.found   = 0;
  maixcam_det.x10     = 0;
  maixcam_det.y10     = 0;
  maixcam_det.distance10 = -10;
  maixcam_det.update_tick = 0;
  maixcam_det.valid   = 0;

  maixcam_loc.x10     = 0;
  maixcam_loc.valid   = 0;
  maixcam_loc.update_tick = 0;
  maixcam_loc_new     = 0;

  maixcam_cmd.state         = MAIXCAM_CMD_IDLE;
  maixcam_cmd.cmd_type      = 0;
  maixcam_cmd.frame_len     = 0;
  maixcam_cmd.send_count    = 0;
  maixcam_cmd.timeout_cycles = 0;
  maixcam_tx_head = 0;
  maixcam_tx_tail = 0;
  maixcam_tx_busy = 0;
}

void maixcam_rx_feed(uint8_t byte) {
  uint16_t next = (maixcam_rx.head + 1) % MAIXCAM_RX_BUF_SIZE;
  if (next == maixcam_rx.tail) {
    maixcam_rx.overflow = 1;
    return;
  }
  maixcam_rx.buf[maixcam_rx.head] = byte;
  maixcam_rx.head = next;
}

static uint8_t maixcam_rx_read(uint8_t *out) {
  if (maixcam_rx.head == maixcam_rx.tail) return 0;
  *out = maixcam_rx.buf[maixcam_rx.tail];
  maixcam_rx.tail = (maixcam_rx.tail + 1) % MAIXCAM_RX_BUF_SIZE;
  return 1;
}

/* ── 命令发送 (DMA TX) ── */

static void maixcam_tx_start_next(void) {
  if (maixcam_tx_busy || maixcam_tx_head == maixcam_tx_tail) return;
  maixcam_tx_busy = 1;
  if (HAL_UART_Transmit_DMA(&huart4,
                            maixcam_tx_queue[maixcam_tx_tail].data,
                            maixcam_tx_queue[maixcam_tx_tail].len) != HAL_OK) {
    maixcam_tx_busy = 0;
  }
}

uint8_t maixcam_uart_tx_enqueue(const uint8_t *data, uint16_t len) {
  uint8_t next;
  uint32_t primask;
  if (data == NULL || len == 0 || len > MAIXCAM_TX_FRAME_MAX) return 0;

  primask = __get_PRIMASK();
  __disable_irq();
  next = (uint8_t)((maixcam_tx_head + 1u) % MAIXCAM_TX_QUEUE_DEPTH);
  if (next == maixcam_tx_tail) {
    if (!primask) __enable_irq();
    return 0;
  }
  for (uint16_t i = 0; i < len; i++) {
    maixcam_tx_queue[maixcam_tx_head].data[i] = data[i];
  }
  maixcam_tx_queue[maixcam_tx_head].len = len;
  maixcam_tx_head = next;
  maixcam_tx_start_next();
  if (!primask) __enable_irq();
  return 1;
}

void maixcam_uart_tx_complete(void) {
  if (maixcam_tx_busy) {
    maixcam_tx_tail = (uint8_t)((maixcam_tx_tail + 1u) % MAIXCAM_TX_QUEUE_DEPTH);
    maixcam_tx_busy = 0;
  }
  maixcam_tx_start_next();
}

static HAL_StatusTypeDef maixcam_send_frame(const char *frame, uint8_t len) {
  return maixcam_uart_tx_enqueue((const uint8_t *)frame, len) ? HAL_OK : HAL_BUSY;
}

static void maixcam_start_cmd(char cmd_type, const char *frame, uint8_t len) {
  uint8_t copy_len = len;
  if (copy_len >= MAIXCAM_CMD_FRAME_MAX) {
    copy_len = MAIXCAM_CMD_FRAME_MAX - 1U;
  }

  maixcam_cmd.state          = MAIXCAM_CMD_WAITING;
  maixcam_cmd.cmd_type       = cmd_type;
  maixcam_cmd.send_count     = 0;
  maixcam_cmd.timeout_cycles = 0;
  maixcam_cmd.frame_len      = copy_len;
  for (uint8_t i = 0; i < copy_len; i++) {
    maixcam_cmd.frame[i] = frame[i];
  }
  maixcam_cmd.frame[copy_len] = '\0';

  if (maixcam_send_frame(maixcam_cmd.frame, copy_len) == HAL_OK) {
    maixcam_cmd.send_count = 1;
  }
}

void maixcam_cmd_T(uint8_t on_off) {
  char buf[8];
  int len = snprintf(buf, sizeof(buf), "CT%u#", on_off ? 1u : 0u);
  if (len > 0 && len < (int)sizeof(buf)) maixcam_start_cmd('T', buf, (uint8_t)len);
}

void maixcam_cmd_D(uint8_t on_off) {
  char buf[8];
  int len = snprintf(buf, sizeof(buf), "CD%u#", on_off ? 1u : 0u);
  if (len > 0 && len < (int)sizeof(buf)) maixcam_start_cmd('D', buf, (uint8_t)len);
}

void maixcam_cmd_CDA(void) {
  maixcam_start_cmd('D', "CDA#", 4);
}

void maixcam_cmd_M(uint8_t mode) {
  char buf[8];
  int len = snprintf(buf, sizeof(buf), "CM%u#", mode);
  if (len > 0 && len < (int)sizeof(buf)) maixcam_start_cmd('M', buf, (uint8_t)len);
}

void maixcam_cmd_send_val(char type, float v) {
  char buf[16];
  int32_t v10 = (int32_t)(v * 10.0f + (v >= 0.0f ? 0.5f : -0.5f));
  uint8_t neg = 0;
  if (v10 < 0) { neg = 1; v10 = -v10; }
  int32_t integer = v10 / 10;
  int32_t frac    = v10 % 10;
  int len = snprintf(buf, sizeof(buf), "C%c%s%ld.%ld#",
                     type,
                     neg ? "-" : "",
                     (long)integer,
                     (long)frac);
  if (len > 0 && len < (int)sizeof(buf)) maixcam_start_cmd(type, buf, (uint8_t)len);
}

/* ── 超时/重发状态机 ── */

void maixcam_cmd_done(char cmd_type, uint8_t result) {
  if (maixcam_cmd.state != MAIXCAM_CMD_WAITING) return;
  if (maixcam_cmd.cmd_type != cmd_type)          return;

  maixcam_cmd.state = result ? MAIXCAM_CMD_OK : MAIXCAM_CMD_FAIL;
}

static void maixcam_tick_inner(void) {
  if (maixcam_cmd.state != MAIXCAM_CMD_WAITING) return;

  maixcam_cmd.timeout_cycles++;

  if (maixcam_cmd.timeout_cycles < MAIXCAM_TIMEOUT_CYCLES) return;

  if (maixcam_cmd.send_count < MAIXCAM_RETRY_MAX) {
    if (maixcam_send_frame(maixcam_cmd.frame, maixcam_cmd.frame_len) == HAL_OK) {
      maixcam_cmd.send_count++;
      maixcam_cmd.timeout_cycles = 0;
    }
  } else {
    maixcam_cmd.state = MAIXCAM_CMD_TIMEOUT;
  }
}

/* ── V 帧解析器 —— 状态机逐字节组帧，在主循环/8ms tick 中调用 ── */

#define MAIXCAM_FRAME_MAX  64

typedef enum {
  MC_PARSE_WAIT_V = 0,
  MC_PARSE_BODY,
} McParseState;

static McParseState mc_ps = MC_PARSE_WAIT_V;
static char  mc_buf[MAIXCAM_FRAME_MAX];
static uint8_t mc_idx;
static uint8_t mc_new_detection;

/* ── 固定一位小数解析：读入符号可选、整数、小数点、一位小数 ── */
static uint8_t parse_fixed1(const char **p, int32_t *out) {
  const char *s = *p;
  int8_t sign = 1;

  if (*s == '-') { sign = -1; s++; }

  if (*s < '0' || *s > '9') return 0;
  int32_t val = (int32_t)(*s - '0');
  s++;
  while (*s >= '0' && *s <= '9') {
    val = val * 10 + (int32_t)(*s - '0');
    s++;
  }

  if (*s != '.') return 0;
  s++;

  if (*s < '0' || *s > '9') return 0;
  val = val * 10 + (int32_t)(*s - '0');
  s++;

  *out = sign * val;
  *p = s;
  return 1;
}

/* ── 解析 VD 检测数据帧：VD,<found>,<x>,<y>,<distance># ── */
static void parse_vd_frame(const char *body) {
  const char *p = body;

  if (*p != ',') return;
  p++;

  if (*p != '0' && *p != '1') return;
  uint8_t found = (uint8_t)(*p - '0');
  p++;

  if (*p != ',') return;
  p++;

  int32_t x10;
  if (!parse_fixed1(&p, &x10)) return;
  if (*p != ',') return;
  p++;

  int32_t y10;
  if (!parse_fixed1(&p, &y10)) return;
  if (*p != ',') return;
  p++;

  int32_t d10;
  if (!parse_fixed1(&p, &d10)) return;

  if (*p != '#') return;

  maixcam_det.found       = found;
  maixcam_det.x10         = x10;
  maixcam_det.y10         = y10;
  maixcam_det.distance10  = d10;
  maixcam_det.valid       = 1;
  mc_new_detection        = 1;
  maixcam_det_new         = 1;
}

/* ── 解析 VL 位置数据帧：VL,<x>,<ts>,#   x 固定1位小数(mm)，ts 毫秒时间戳 ── */
static void parse_vl_frame(const char *body) {
  const char *p = body;

  if (*p != ',') return;
  p++;

  int32_t x10;
  if (!parse_fixed1(&p, &x10)) return;

  if (*p != ',') return;
  p++;

  uint32_t ts = 0;
  while (*p >= '0' && *p <= '9') {
    ts = ts * 10 + (uint32_t)(*p - '0');
    p++;
  }

  if (*p != ',') return;
  p++;

  if (*p != '#') return;

  maixcam_loc.x10          = x10;
  maixcam_loc.timestamp_ms = ts;
  maixcam_loc.valid        = 1;
  maixcam_loc_new          = 1;
}

/* ── 解析命令应答帧：V<type>A<result># ── */
static void parse_ack_frame(const char *body) {
  if (body[1] != 'A') return;
  char cmd_type = body[0];
  if (body[2] != '0' && body[2] != '1') return;
  if (body[3] != '#') return;

  uint8_t result = (uint8_t)(body[2] - '0');
  maixcam_cmd_done(cmd_type, result);
}

/* ── 收帧并分类 ── */
static void mc_frame_ready(const char *frame) {
  if (frame[0] != 'V') return;

  /* 保存原始帧 */
  {
    uint8_t i = 0;
    while (frame[i] && i < MAIXCAM_FRAME_MAX - 1) {
      maixcam_rx_raw_buf[i] = frame[i];
      i++;
    }
    maixcam_rx_raw_buf[i] = '\0';
    maixcam_rx_raw_ready = 1;
  }

  /* VD,...# —— 检测数据帧 */
  if (frame[1] == 'D' && frame[2] == ',') {
    parse_vd_frame(frame + 2);
    return;
  }

  /* VL,...# —— 位置数据帧 */
  if (frame[1] == 'L' && frame[2] == ',') {
    parse_vl_frame(frame + 2);
    return;
  }

  /* V<type>A<result># —— 命令应答帧 */
  if (frame[2] == 'A' && frame[3] != '\0') {
    parse_ack_frame(frame + 1);
    return;
  }
}

/* ── 逐字节消费环形缓冲，运行组帧状态机 ── */
static void mc_feed_byte(uint8_t byte) {
  switch (mc_ps) {

  case MC_PARSE_WAIT_V:
    if (byte == 'V') {
      mc_buf[0] = 'V';
      mc_idx = 1;
      mc_ps = MC_PARSE_BODY;
    }
    break;

  case MC_PARSE_BODY:
    if (byte == 'V') {
      mc_buf[0] = 'V';
      mc_idx = 1;
      break;
    }

    if (mc_idx < MAIXCAM_FRAME_MAX - 1) {
      mc_buf[mc_idx++] = (char)byte;
    } else {
      mc_ps = MC_PARSE_WAIT_V;
      mc_idx = 0;
      break;
    }

    if (byte == '#') {
      mc_buf[mc_idx] = '\0';
      mc_frame_ready(mc_buf);
      mc_ps = MC_PARSE_WAIT_V;
      mc_idx = 0;
    }
    break;
  }
}

/* ── 每 8ms 调用一次：消费环形缓冲 + 超时重发 ── */

void maixcam_poll(uint32_t time_ms) {
  uint8_t byte;

  while (maixcam_rx_read(&byte)) {
    mc_feed_byte(byte);
  }

  if (mc_new_detection) {
    mc_new_detection = 0;
    maixcam_det.update_tick = time_ms;
  }

  maixcam_tick_inner();
}
