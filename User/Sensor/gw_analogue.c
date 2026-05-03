#include "adc.h"
#include "gpio.h"
#include "gw_anagloge.h"
#include "log.h"
#include "status.h"
#include "main.h"
#include "stdbool.h"
#include "usart.h"

float distance[8] = {-30, -20, -15, -10, 10, 15, 20, 30};

void gw_analogue_gray_show(GW_ANALOGUE *gw_analogue) {
  uint8_t buf = gw_analogue->digital_8bit;
  char str[9];
  str[8] = '\0';
  for (int i = 0; i < 8; i++) {
    str[i] = buf & 0x80 ? '#' : '.';
    buf <<= 1;
  }
}

static void init_road_determine(Cross *cross) {
  cross->integral = 0;
  cross->data_buf = 0;
  cross->cross = Straight;
  cross->cross_cnt = 0;
  cross->maybe = 0;
  cross->integral_times = 5;
}

static void init_road_cnt(Cross *cross) {
  cross->CrossRoad_cnt = 0;
  cross->LeftRoad_cnt = 0;
  cross->RightRoad_cnt = 0;
  cross->Straight_cnt = 0;
  cross->TBRoad_cnt = 0;
  cross->TLRoad_cnt = 0;
  cross->TRRoadd_cnt = 0;
  cross->UnknowRoad_cnt = 0;
}

void init_gw_analogue(GW_ANALOGUE *gw_analogue) {
  init_road_determine(&gw_analogue->cross);
  init_road_cnt(&gw_analogue->cross);

  for (int i = 0; i < 8; i++) {
    gw_analogue->channel[i] = 0;
  }
  for (int i = 0; i < 8; i++) {
    gw_analogue->correction_data_w[i] = 0;
    gw_analogue->correction_data_b[i] = 0;
  }
  gw_analogue->digital_high_threshold[0] = 81;
  gw_analogue->digital_high_threshold[1] = 137;
  gw_analogue->digital_high_threshold[2] = 77;
  gw_analogue->digital_high_threshold[3] = 82;
  gw_analogue->digital_high_threshold[4] = 61;
  gw_analogue->digital_high_threshold[5] = 115;
  gw_analogue->digital_high_threshold[6] = 129;
  gw_analogue->digital_high_threshold[7] = 75;

  gw_analogue->digital_low_threshold[0] = 49;
  gw_analogue->digital_low_threshold[1] = 95;
  gw_analogue->digital_low_threshold[2] = 49;
  gw_analogue->digital_low_threshold[3] = 55;
  gw_analogue->digital_low_threshold[4] = 37;
  gw_analogue->digital_low_threshold[5] = 77;
  gw_analogue->digital_low_threshold[6] = 89;
  gw_analogue->digital_low_threshold[7] = 47;

  for (int i = 0; i < 8; i++) {
    gw_analogue->correction_data_w[i] = 2 * gw_analogue->digital_high_threshold[i] - gw_analogue->digital_low_threshold[i];
    gw_analogue->correction_data_b[i] = 2 * gw_analogue->digital_low_threshold[i] - gw_analogue->digital_high_threshold[i];
  }

  gw_analogue->sta = 0;
  gw_analogue->digital_8bit = 0;
  gw_analogue->diff = 0.0f;

  select_channel(0);
}

void select_channel(uint8_t channel) {
  if (channel & 0x01) {
    HAL_GPIO_WritePin(AD0_GPIO_Port, AD0_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(AD0_GPIO_Port, AD0_Pin, GPIO_PIN_RESET);
  }
  if (channel & 0x02) {
    HAL_GPIO_WritePin(AD1_GPIO_Port, AD1_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(AD1_GPIO_Port, AD1_Pin, GPIO_PIN_RESET);
  }
  if (channel & 0x04) {
    HAL_GPIO_WritePin(AD2_GPIO_Port, AD2_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(AD2_GPIO_Port, AD2_Pin, GPIO_PIN_RESET);
  }
}

void get_gw_raw_data(GW_ANALOGUE *gw_analogue) {
  for (int i = 0; i < 8; i++) {
    select_channel(i);
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 1);
    gw_analogue->channel[i] = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
  }
}

void correct_gw_analogue(GW_ANALOGUE *gw_analogue) {
  if (gw_analogue->sta == 0) {
    for (int i = 0; i < 8; i++) {
      select_channel(i);
      HAL_ADC_Start(&hadc3);
      HAL_ADC_PollForConversion(&hadc3, 1);
      gw_analogue->correction_data_w[i] = HAL_ADC_GetValue(&hadc3);
      HAL_ADC_Stop(&hadc3);
    }
    status.device.led1.on = 1;
    gw_analogue->sta = 1;
    return;
  }
  if (gw_analogue->sta == 1) {
    for (int i = 0; i < 8; i++) {
      select_channel(i);
      HAL_ADC_Start(&hadc3);
      HAL_ADC_PollForConversion(&hadc3, 1);
      gw_analogue->correction_data_b[i] = HAL_ADC_GetValue(&hadc3);
      HAL_ADC_Stop(&hadc3);
    }
    status.device.led1.on = 0;
    gw_analogue->sta = 0;
    for (int i = 0; i < 8; i++) {
      gw_analogue->digital_low_threshold[i] = gw_analogue->correction_data_b[i] +
                                              (gw_analogue->correction_data_w[i] - gw_analogue->correction_data_b[i]) * 0.33;
      gw_analogue->digital_high_threshold[i] = gw_analogue->correction_data_b[i] +
                                               (gw_analogue->correction_data_w[i] - gw_analogue->correction_data_b[i]) * 0.66;
    }
    return;
  }
}

void get_gw_analoge_digital_data(GW_ANALOGUE *gw_analogue) {
  for (int i = 0; i < 8; i++) {
    if (gw_analogue->channel[i] > gw_analogue->digital_high_threshold[i]) {
      gw_analogue->digital_8bit &= ~(1 << i);
    } else if (gw_analogue->channel[i] < gw_analogue->digital_low_threshold[i]) {
      gw_analogue->digital_8bit |= (1 << i);
    }
  }
}

float normalize_gray_data(uint8_t max, uint8_t min, uint8_t now) {
  return (((float)(now - min) / (float)(max - min)) * 100);
}

void normalize_gray_weight(float *raw_data) {
  float total = 0;
  for (int i = 2; i < 6; i++) {
    total += raw_data[i];
  }
  if (total == 0) return;
  for (int i = 2; i < 6; i++) {
    raw_data[i] = (raw_data[i] / total);
  }
}

void get_gw_analogue_analogue_diff(GW_ANALOGUE *gw_analogue) {
  float buff[8] = {0};
  float diff = 0;
  for (int i = 2; i < 6; i++) {
    if (gw_analogue->channel[i] < gw_analogue->digital_high_threshold[i])
      buff[i] = 100 - normalize_gray_data(gw_analogue->correction_data_w[i], gw_analogue->correction_data_b[i], gw_analogue->channel[i]);
  }
  normalize_gray_weight(buff);
  for (int i = 2; i < 6; i++) {
    diff += buff[i] * distance[i];
  }
  status.sensor.gw_analogue.diff = diff;
}

// ── 路口判断（来自 xiao 项目）──

enum Road road_new_from_bit(bool L, bool F, bool R) {
  uint8_t left = L ? 0b100 : 0;
  uint8_t font = F ? 0b010 : 0;
  uint8_t right = R ? 0b001 : 0;

  return left | font | right;
}

Road road_decision(Cross *cross) {
  bool left = (cross->integral >> 6) == 0x03;     // 0b1100_0000
  bool right = (cross->integral & 0x03) == 0x03;  // 0b0000_0011
  bool font = cross->data_buf & 0x3C;             // 0b0011_1100
  Road road = road_new_from_bit(left, font, right);
  return road;
}

void serve_road(Cross *cross, Road road) {
  switch (road) {
    case CrossRoad:
      cross->CrossRoad_cnt++;
      break;
    case TBRoad:
      cross->TBRoad_cnt++;
      break;
    case TLRoad:
      cross->TLRoad_cnt++;
      break;
    case TRRoad:
      cross->TRRoadd_cnt++;
      break;
    case LeftRoad:
      cross->LeftRoad_cnt++;
      break;
    case RightRoad:
      cross->RightRoad_cnt++;
      break;
    case Straight:
      cross->Straight_cnt++;
      break;
    case UnknowRoad:
      cross->UnknowRoad_cnt++;
      break;
  }
  cross->cross = road;
}

void get_road_type(Cross *cross, uint8_t road_data) {
  cross->data_buf = road_data;
  if (cross->cross == Straight) {
    if ((cross->data_buf & 0x81)) {
      if (cross->maybe == 0) {
        log_uprintf(&huart1, "Maybe\n");
        cross->maybe = cross->integral_times;
        cross->integral = cross->integral | cross->data_buf;
      }
    }
    if (cross->maybe > 1) {
      cross->integral = cross->integral | cross->data_buf;
      cross->maybe--;
    } else if (cross->maybe == 1) {
      switch (road_decision(cross)) {
        case UnknowRoad:
          log_uprintf(&huart1, "Unknow road\n");
          serve_road(cross, UnknowRoad);
          break;
        case CrossRoad:
          log_uprintf(&huart1, "Cross road\n");
          serve_road(cross, CrossRoad);
          break;
        case TBRoad:
          log_uprintf(&huart1, "T B road\n");
          serve_road(cross, TBRoad);
          break;
        case TLRoad:
          log_uprintf(&huart1, "T L road\n");
          serve_road(cross, TLRoad);
          break;
        case TRRoad:
          log_uprintf(&huart1, "T R road\n");
          serve_road(cross, TRRoad);
          break;
        case LeftRoad:
          log_uprintf(&huart1, "Left road\n");
          serve_road(cross, LeftRoad);
          break;
        case RightRoad:
          log_uprintf(&huart1, "Right road\n");
          serve_road(cross, RightRoad);
          break;
        case Straight:
          serve_road(cross, Straight);
          break;
      }
      cross->maybe = 0;
      cross->integral = 0;
    }
  } else if ((status.sensor.gw_analogue.digital_8bit & 0x81) == 0
             && (status.sensor.gw_analogue.digital_8bit & 0x3C) != 0) {
    log_uprintf(&huart1, "Straight road\n");
    serve_road(cross, Straight);
  }
}

// ── 统一驱动入口 ──

void driver_gw_analogue(GW_ANALOGUE *gw_analogue) {
  get_gw_raw_data(gw_analogue);
  get_gw_analoge_digital_data(gw_analogue);
  get_gw_analogue_analogue_diff(gw_analogue);
  get_road_type(&gw_analogue->cross, gw_analogue->digital_8bit);
}
