#ifndef __MAIXCAM_H
#define __MAIXCAM_H

#include <stdint.h>

/* ── 环形接收缓冲区 ── */
#define MAIXCAM_RX_BUF_SIZE   128

typedef struct {
  uint8_t buf[MAIXCAM_RX_BUF_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint8_t  overflow;
} MaixcamRxRing;

/* ── 视觉检测数据 ── */
typedef struct {
  uint8_t found;
  int32_t x10;
  int32_t y10;
  int32_t distance10;
  uint32_t update_tick;
  uint8_t valid;
} VisionDetectionData;

/* ── 位置数据 (VL 帧) ── */
typedef struct {
  int32_t x10;           /* x 坐标，单位 0.1mm */
  uint32_t timestamp_ms; /* 视觉模块时间戳，单位 ms */
  uint8_t valid;
  uint32_t update_tick;
} VisionLocationData;

/* ── 命令等待/重发状态 ── */
#define MAIXCAM_CMD_FRAME_MAX  16
#define MAIXCAM_RETRY_MAX       3
#define MAIXCAM_TIMEOUT_CYCLES 10   /* 10 × 8ms = 80ms */

typedef enum {
  MAIXCAM_CMD_IDLE = 0,
  MAIXCAM_CMD_WAITING,
  MAIXCAM_CMD_OK,
  MAIXCAM_CMD_FAIL,
  MAIXCAM_CMD_TIMEOUT,
} MaixcamCmdState;

typedef struct {
  MaixcamCmdState state;
  char    cmd_type;
  char    frame[MAIXCAM_CMD_FRAME_MAX];
  uint8_t frame_len;
  uint8_t send_count;
  uint8_t timeout_cycles;
} MaixcamCmdReq;

/* ── 模块全局 ── */
extern MaixcamRxRing        maixcam_rx;
extern VisionDetectionData  maixcam_det;
extern VisionLocationData   maixcam_loc;
extern MaixcamCmdReq        maixcam_cmd;
extern uint8_t              maixcam_det_new;
extern uint8_t              maixcam_loc_new;
extern char                 maixcam_rx_raw_buf[64];
extern uint8_t              maixcam_rx_raw_ready;

/* ── 初始化 ── */
void maixcam_init(void);

/* ── 命令发送 ── */
void maixcam_cmd_T(uint8_t on_off);            /* CT0# / CT1# */
void maixcam_cmd_D(uint8_t on_off);            /* CD0# / CD1# */
void maixcam_cmd_CDA(void);                    /* CDA# — start continuous VL stream */
void maixcam_cmd_M(uint8_t mode);              /* CM<mode># */
void maixcam_cmd_send_val(char type, float v); /* C<type><1位小数># 通用 */
uint8_t maixcam_uart_tx_enqueue(const uint8_t *data, uint16_t len);
void maixcam_uart_tx_complete(void);

/* ── UART ISR 中调用：喂入一个接收字节到环形缓冲 ── */
void maixcam_rx_feed(uint8_t byte);

/* ── 每 8ms 调用一次：帧解析 + 超时重发 ── */
void maixcam_poll(uint32_t time_ms);

/* ── 由帧解析器在收到匹配应答后调用：结束当前命令等待 ── */
void maixcam_cmd_done(char cmd_type, uint8_t result);

#endif /* __MAIXCAM_H */
