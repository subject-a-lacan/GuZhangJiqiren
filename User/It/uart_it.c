#include "uart_it.h"
#include "usart.h"
#include <stdint.h>
#include <stdlib.h>
#include "uart_gyro.h"
#include "status.h"
#include "maixcam.h"
#include "lora.h"
#include "Emm_v5.h"

#define MAIXCAM_DMA_SIZE 128
static uint8_t maixcam_dma_buf[MAIXCAM_DMA_SIZE];
static uint16_t maixcam_dma_pos;

void UART_PID_Tune(uint8_t cmd, float val, uint8_t has_val);

typedef struct {
  uint8_t byte;
  uint8_t cmd;
  uint8_t state;
  uint8_t index;
  char buf[20];
} UART_PID_RX;

static UART_PID_RX uart1_pid_rx = {0};
static UART_PID_RX uart2_pid_rx = {0};
static UART_PID_RX uart3_pid_rx = {0};
static uint8_t uart1_pid_byte;

static void reset_uart_pid_rx(UART_PID_RX *rx) {
  rx->cmd = 0;
  rx->state = 0;
  rx->index = 0;
}

static void parse_uart_pid_byte(UART_PID_RX *rx) {
  switch (rx->state) {
    case 0:
      if (rx->byte == 'C') {
        rx->state = 1;
      }
      break;

    case 1:
      rx->cmd = rx->byte;
      rx->index = 0;
      rx->state = 2;
      break;

    case 2:
      if (rx->byte == '\r' || rx->byte == '\n') {
        rx->buf[rx->index] = '\0';
        UART_PID_Tune(rx->cmd, atof(rx->buf), rx->index > 0);
        rx->state = 0;
      } else if (rx->index < sizeof(rx->buf) - 1) {
        rx->buf[rx->index++] = rx->byte;
      }
      break;

    default:
      rx->state = 0;
      break;
  }
}

void init_uart_pid_tune(void) {
  HAL_UART_Receive_IT(&huart1, &uart1_pid_byte, 1);
}

static uint8_t stepper_rx_byte;

typedef struct {
  uint8_t enabled;
  int32_t absolute_pulse;
  uint16_t velocity;
  uint8_t accel_param;
  uint32_t publish_seq;
} STEPPER_TARGET_SNAPSHOT;

void stepper_request_move(uint32_t clk, uint16_t vel, uint8_t acc) {
  status.stepper.reached = 0;
  status.stepper.clk = clk;
  status.stepper.busy = 1;
  if (!Emm_V5_Pos_Control(1, 0, vel, acc, clk, false, false))
    status.stepper.busy = 0;
}

void stepper_publish_absolute(int32_t absolute_pulse, uint16_t vel, uint8_t acc) {
  STEPPER_TARGET *target = &status.stepper.target;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (!target->enabled || target->absolute_pulse != absolute_pulse ||
      target->velocity != vel || target->accel_param != acc) {
    target->enabled = 1;
    target->absolute_pulse = absolute_pulse;
    target->velocity = vel;
    target->accel_param = acc;
    __DMB();
    target->publish_seq++;
  }
  if (!primask) __enable_irq();
}

void stepper_target_disable(void) {
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (status.stepper.target.enabled) {
    status.stepper.target.enabled = 0;
    __DMB();
    status.stepper.target.publish_seq++;
  }
  if (!primask) __enable_irq();
}

void stepper_service(void) {
  STEPPER_TARGET_SNAPSHOT target;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  target.enabled = status.stepper.target.enabled;
  target.absolute_pulse = status.stepper.target.absolute_pulse;
  target.velocity = status.stepper.target.velocity;
  target.accel_param = status.stepper.target.accel_param;
  target.publish_seq = status.stepper.target.publish_seq;

  if (target.publish_seq == status.stepper.tx.last_started_seq) {
    if (!primask) __enable_irq();
    return;
  }
  if (!target.enabled) {
    status.stepper.tx.last_started_seq = target.publish_seq;
    if (!primask) __enable_irq();
    return;
  }
  if (status.stepper.tx.dma_busy) {
    if (!primask) __enable_irq();
    return;
  }
  status.stepper.tx.dma_busy = 1;
  if (Emm_V5_Pos_Absolute_Try(1, target.absolute_pulse, target.velocity,
                              target.accel_param, false)) {
    status.stepper.tx.last_started_seq = target.publish_seq;
    status.stepper.reached = 0;
    status.stepper.busy = 1;
    status.stepper.clk = target.absolute_pulse < 0
                             ? (uint32_t)(-(int64_t)target.absolute_pulse)
                             : (uint32_t)target.absolute_pulse;
  } else {
    status.stepper.tx.dma_busy = 0;
  }
  if (!primask) __enable_irq();
}

void init_stepper_uart(void) {
  status.stepper.reached = 0;
  status.stepper.busy = 0;
  status.stepper.clk = 0;
  status.stepper.target.enabled = 0;
  status.stepper.target.absolute_pulse = 0;
  status.stepper.target.velocity = 0;
  status.stepper.target.accel_param = 0;
  status.stepper.target.publish_seq = 0;
  status.stepper.tx.dma_busy = 0;
  status.stepper.tx.last_started_seq = 0;
  HAL_UART_Receive_IT(&huart3, &stepper_rx_byte, 1);
}

void init_uart_gyr(void) {
  uart_gyr_init(&status.sensor.uart_gyr);
  uart_gyr_start_receive(&status.sensor.uart_gyr);
}

void init_maixcam_uart(void) {
  maixcam_init();
  maixcam_dma_pos = 0;
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, maixcam_dma_buf, MAIXCAM_DMA_SIZE);
  __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
}

void UART_PID_Tune(uint8_t cmd, float val, uint8_t has_val) {
  uint8_t handled = 1;
  if (!has_val) {
    if (cmd == 's') status.task.start_request = 1;
    if (cmd == 'p') status.task.stop_request = 1;
    return;
  }
  switch (cmd) {
    case 'a': status.state.status_pid.follow_line_pid.kp = val; break;
    case 'c': status.state.status_pid.follow_line_pid.ki = val; break;
    case 'e': status.state.status_pid.follow_line_pid.kd = val; break;
    case 'd': status.state.status_pid.keep_angle_pid.kp = val; break;
    case 'f': status.state.status_pid.keep_angle_pid.ki = val; break;
    case 'g': status.state.status_pid.keep_angle_pid.kd = val; break;
    case 'm': status.motor.wheel[0].wheel_pid.kp = val; break;
    case 'o': status.motor.wheel[0].wheel_pid.ki = val; break;
    case 'q': status.motor.wheel[0].wheel_pid.kd = val; break;
    case 's': status.motor.wheel[1].wheel_pid.kp = val; break;
    case 'u': status.motor.wheel[1].wheel_pid.ki = val; break;
    case 'w': status.motor.wheel[1].wheel_pid.kd = val; break;
    case 'i': status.motor.wheel[2].wheel_pid.kp = val; break;
    case 'k': status.motor.wheel[2].wheel_pid.ki = val; break;
    case 'l': status.motor.wheel[2].wheel_pid.kd = val; break;
    case 'j': status.motor.wheel[3].wheel_pid.kp = val; break;
    case 'p': status.motor.wheel[3].wheel_pid.ki = val; break;
    case 'r': status.motor.wheel[3].wheel_pid.kd = val; break;
    case 'b': status.motor.wheel[0].wheel_pid.integral_max = val; break;
    case 'n': status.motor.wheel[1].wheel_pid.integral_max = val; break;
    case 'x': status.motor.wheel[2].wheel_pid.integral_max = val; break;
    case 'y': status.motor.wheel[3].wheel_pid.integral_max = val; break;
    case 'z': status.motor.wheel[0].wheel_pid.kp = val;
              status.motor.wheel[1].wheel_pid.kp = val;
              status.motor.wheel[2].wheel_pid.kp = val;
              status.motor.wheel[3].wheel_pid.kp = val; break;
    case 'v': status.motor.wheel[0].wheel_pid.ki = val;
              status.motor.wheel[1].wheel_pid.ki = val;
              status.motor.wheel[2].wheel_pid.ki = val;
              status.motor.wheel[3].wheel_pid.ki = val; break;
    case 'h': status.state.base_speed = (int16_t)val; break;
    case 't': task_select(&status, (uint8_t)val); break;
    default: handled = 0; break;
  }
  if (handled) {
    status.device.buzzer.on = 1;
    status.device.buzzer.off_time = status.state.time + 140;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    UART_GYRO *gyro = &status.sensor.uart_gyr;
    gyro->rx_cb_cnt++;
    uart_gyr_rx_feed(gyro, gyro->rx_byte, (uint32_t)status.state.time);
    uart_gyr_start_receive(gyro);
  }
  if (huart == &huart1) {
    uart1_pid_rx.byte = uart1_pid_byte;
    if (esp8266_ready) parse_uart_pid_byte(&uart1_pid_rx);
    HAL_UART_Receive_IT(&huart1, &uart1_pid_byte, 1);
  }
  if (huart == &huart3) {
    static uint8_t buf[4], idx;
    static const uint8_t reached_frame[4] = {0x01, 0xFD, 0x9F, 0x6B};
    buf[idx] = stepper_rx_byte;
    if (buf[idx] == reached_frame[idx]) {
      idx++;
      if (idx == 4) { status.stepper.reached = 1; status.stepper.busy = 0; idx = 0; }
    } else {
      idx = (stepper_rx_byte == 0x01) ? 1 : 0;
      if (idx) buf[0] = 0x01;
    }
    HAL_UART_Receive_IT(&huart3, &stepper_rx_byte, 1);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
  if (huart != &huart4 || size > MAIXCAM_DMA_SIZE) return;

  if (size >= maixcam_dma_pos) {
    for (uint16_t i = maixcam_dma_pos; i < size; i++) maixcam_rx_feed(maixcam_dma_buf[i]);
  } else {
    for (uint16_t i = maixcam_dma_pos; i < MAIXCAM_DMA_SIZE; i++) maixcam_rx_feed(maixcam_dma_buf[i]);
    for (uint16_t i = 0; i < size; i++) maixcam_rx_feed(maixcam_dma_buf[i]);
  }
  maixcam_dma_pos = (size == MAIXCAM_DMA_SIZE) ? 0 : size;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart3) status.stepper.tx.dma_busy = 0;
  if (huart == &huart4) maixcam_uart_tx_complete();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) HAL_UART_Receive_IT(&huart1, &uart1_pid_byte, 1);
  if (huart == &huart2) {
    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    status.sensor.uart_gyr.count = 0;
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    uart_gyr_start_receive(&status.sensor.uart_gyr);
  }
  if (huart == &huart3) {
    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    if ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U &&
        huart->gState == HAL_UART_STATE_READY && status.stepper.tx.dma_busy) {
      status.stepper.tx.dma_busy = 0;
      status.stepper.tx.last_started_seq--;
    }
    HAL_UART_Receive_IT(&huart3, &stepper_rx_byte, 1);
  }
  if (huart == &huart4) {
    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    maixcam_dma_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, maixcam_dma_buf, MAIXCAM_DMA_SIZE);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
  }
}
