#include "adc.h"
#include "gpio.h"
#include "gw_anagloge.h"
#include "log.h"
#include "main.h"
#include "status.h"
#include "math_tool.h"
#include "Defect.h"
#include "stdbool.h"
float distance[8] = {-30, -20, -15, -10, 10, 15, 20, 30};

uint8_t cross_cnt = 0;
uint8_t left_cnt = 0;

/*
 * 璋冭瘯鐢細灏?digital_8bit 鐢?ASCII 瀛楃鎵撳嵃鍒颁覆鍙ｏ紙# = 鐪嬪埌绾匡紝. = 娌＄湅鍒帮級銆?
 * 鏁版嵁鏉ユ簮锛歴tatus.sensor.gw_analogue.digital_8bit锛堢敱 get_gw_analoge_digital_data 鏇存柊锛夈€?
 * 璋冪敤鏃舵満锛氫粎鍦ㄨ皟璇曟椂鎵嬪姩璋冪敤锛屼笉鍙備笌宸＄嚎/璺彛涓婚摼璺€?
 */
void gw_analogue_gray_show(GW_ANALOGUE *gw_analogue) {
  uint8_t buf = gw_analogue->digital_8bit;
  char str[9];
  str[8] = '\0';
  for (int i = 0; i < 8; i++) {
    str[i] = buf & 0x80 ? '#' : '.';
    buf <<= 1;
  }
  PRINTLN("%s", str);
}

/*
 * 鍒濆鍖栬矾鍙ｅ垽瀹氱紦瀛橈紙status.sensor.gw_analogue.cross 涓殑鍒ゅ畾瀛楁锛夈€?
 * 灏?integral / data_buf / maybe / cross / cross_cnt 缃负瀹夊叏鍒濆€硷紝
 * integral_times 璁句负 70锛堥渶杩炵画 70 甯х‘璁よ矾鍙ｏ紝璇﹁ get_road_type锛夈€?
 * 璋冪敤鏃舵満锛歩nit_gw_analogue() 涓婄數鍒濆鍖栵紱task_start() 鍙戣溅澶嶄綅锛圖efect.c锛夈€?
 */
void init_road_determine(Cross *cross) {
  cross->integral = 0;
  cross->data_buf = 0;
  cross->maybe = 0;
  cross->cross = Straight;
  cross->cross_cnt = 0;
  cross->integral_times = 4;

  return;
}

/*
 * 娓呴浂璺彛绫诲瀷璋冭瘯璁℃暟鍣紙CrossRoad_cnt ~ UnknowRoad_cnt锛夈€?
 * 杩欎簺璁℃暟鍣ㄥ彧璁板綍浼犳劅鍣ㄥ眰瑙傛祴鍒扮殑鍚勭被鍨嬭矾鍙ｆ鏁帮紝浠呯敤浜庤皟璇?VOFA 鎵撳嵃锛?
 * 涓嶈兘鏇夸唬 status.task.cross_cnt锛圱ASK 鐘舵€佹満纭鐨勬湁鏁堣矾鍙ｈ鏁帮級銆?
 * 璋冪敤鏃舵満锛歩nit_gw_analogue() 涓婄數鍒濆鍖栥€?
 */
void init_road_cnt(Cross *cross) {
  cross->CrossRoad_cnt = 0;
  cross->LeftRoad_cnt = 0;
  cross->RightRoad_cnt = 0;
  cross->Straight_cnt = 0;
  cross->TBRoad_cnt = 0;
  cross->TLRoad_cnt = 0;
  cross->TRRoad_cnt = 0;
  cross->UnknowRoad_cnt = 0;
  return;
}

/*
 * 妯℃嫙鐏板害浼犳劅鍣ㄤ笂鐢靛垵濮嬪寲銆?
 * 鎿嶄綔瀵硅薄锛歴tatus.sensor.gw_analogue锛堟暣涓?GW_ANALOGUE 缁撴瀯浣擄級銆?
 * 1. 鍒濆鍖栬矾鍙ｅ垽瀹氱紦瀛樺拰璋冭瘯璁℃暟鍣紙璋冪敤 init_road_determine / init_road_cnt锛夈€?
 * 2. 娓呴浂 channel[] / correction_data_w[] / correction_data_b[]銆?
 * 3. 鍐欏叆榛樿楂樹綆闃堝€硷紙楂?46-47 / 浣?24-25锛屽疄杞﹁皟鍑烘潵鐨勭粡楠屽€硷級銆?
 * 4. 鐢ㄩ粯璁ら槇鍊兼帹绠楀嚭鏍″噯鏁版嵁鐨勫叕寮忓€硷紙浣滀负鏍″噯鍓嶅厹搴曪級銆?
 * 5. 娓呴浂 sta / digital_8bit / diff銆?
 * 6. 閫変腑閫氶亾 0 浣滀负鍒濆 ADC 閫氶亾銆?
 * 璋冪敤鏃舵満锛歩nit_sensor() 鈫?status.c 涓婄數鍒濆鍖栭摼璺€?
 */
void init_gw_analogue(GW_ANALOGUE *gw_analogue) {
  init_road_determine(&gw_analogue->cross);
  init_road_cnt(&gw_analogue->cross);

  // Initialize the ADC and GPIO for the analogue channels
  for (int i = 0; i < 8; i++) {
    gw_analogue->channel[i] = 0;  // Initialize channel values to 0
  }
  for (int i = 0; i < 8; i++) {
    gw_analogue->correction_data_w[i] = 0;  // Initialize correction data to 0
    gw_analogue->correction_data_b[i] = 0;  // Initialize correction data to 0
  }
 gw_analogue->digital_high_threshold[0] = 46;
  gw_analogue->digital_high_threshold[1] = 46;
  gw_analogue->digital_high_threshold[2] = 47;
  gw_analogue->digital_high_threshold[3] = 47;
  gw_analogue->digital_high_threshold[4] = 47;
  gw_analogue->digital_high_threshold[5] = 47;
  gw_analogue->digital_high_threshold[6] = 46;
  gw_analogue->digital_high_threshold[7] = 46;

  gw_analogue->digital_low_threshold[0] = 24;
  gw_analogue->digital_low_threshold[1] = 24;
  gw_analogue->digital_low_threshold[2] = 24;
  gw_analogue->digital_low_threshold[3] = 25;
  gw_analogue->digital_low_threshold[4] = 25;
  gw_analogue->digital_low_threshold[5] = 25;
  gw_analogue->digital_low_threshold[6] = 25;
  gw_analogue->digital_low_threshold[7] = 24;

  for (int i = 0; i < 8; i++) {
    gw_analogue->correction_data_w[i] = 2 * gw_analogue->digital_high_threshold[i] - gw_analogue->digital_low_threshold[i];  // Initialize high threshold to 0
    gw_analogue->correction_data_b[i] = 2 * gw_analogue->digital_low_threshold[i] - gw_analogue->digital_high_threshold[i];  // Initialize low threshold to 0
  }

  gw_analogue->sta = 0;           // Set the state to 0 (normal mode)
  gw_analogue->digital_8bit = 0;  // Initialize the 8-bit digital value to 0

  gw_analogue->diff = 0.0f;  // Initialize the difference value to 0.0f

  select_channel(0);  // Select channel 0 for initial setup
}

/*
 * 纭欢灞傦細閫氳繃 IO2/IO3/IO4 涓夋牴 GPIO 閫夋嫨 8 閫?1 妯℃嫙寮€鍏崇殑閫氶亾銆?
 * channel 鐨勪綆 3 浣嶅垎鍒帶鍒朵笁鏍?IO锛?
 *   bit0 鈫?IO2,  bit1 鈫?IO3,  bit2 鈫?IO4
 * 璋冪敤鏃舵満锛歡et_gw_raw_data() 鍜?correct_gw_analogue() 閬嶅巻 8 閫氶亾鏃躲€?
 */
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

/*
 * 閬嶅巻 8 涓€氶亾锛屼緷娆￠€夋嫨閫氶亾 鈫?鍚姩 ADC3 鈫?绛夎浆鎹㈠畬鎴?鈫?璇诲彇 12bit ADC 鍊笺€?
 * 缁撴灉鍐欏叆 status.sensor.gw_analogue.channel[0..7]锛?-4095 鍘熷 ADC 鍊硷級銆?
 * 璋冪敤鏃舵満锛歞river_gw_analogue() 姣忎釜鎺у埗鍛ㄦ湡璋冪敤涓€娆°€?
 */
void get_gw_raw_data(GW_ANALOGUE *gw_analogue) {
  // Read the ADC value for the selected channel
  for (int i = 0; i < 8; i++) {
    select_channel(i);                                   // Select the channel to read from
    for (volatile uint32_t _d = 0; _d < 300; _d++);     // ~3渭s delay for mux settling
    HAL_ADC_Start(&hadc3);                               // Start the ADC conversion
    HAL_ADC_PollForConversion(&hadc3, 1);                // Wait for conversion to complete
    gw_analogue->channel[i] = HAL_ADC_GetValue(&hadc3);  // Get the ADC value
    HAL_ADC_Stop(&hadc3);                                // Stop the ADC conversion
  }
}

/*
 * 涓ら樁娈电伆搴︽牎鍑嗭紝鐢卞閮ㄦ牎鍑嗘祦绋嬫墜鍔ㄩ┍鍔紙涓嶅湪 driver_gw_analogue 鍐呰嚜鍔ㄨ皟鐢級銆?
 * 鎿嶄綔瀵硅薄锛歴tatus.sensor.gw_analogue銆?
 * sta=0锛堢櫧鏍″噯锛夛細璇诲彇 8 閫氶亾 ADC 鈫?correction_data_w[]锛岀偣浜澘杞?LED 鎻愮ず鐢ㄦ埛锛?
 *                  sta 鍒囧埌 1锛宺eturn 绛変笅娆¤皟鐢ㄣ€?
 * sta=1锛堥粦鏍″噯锛夛細璇诲彇 8 閫氶亾 ADC 鈫?correction_data_b[]锛岀唲鐏?LED锛?
 *                 鎸夌櫧榛戝樊鍊?脳0.33/0.66 璁＄畻 digital_low_threshold / digital_high_threshold锛?
 *                 sta 鍒囧洖 0锛屼覆鍙ｆ墦鍗版牎鍑嗙粨鏋溿€?
 * 娉ㄦ剰锛氭牎鍑嗘椂鎿嶄綔鐨?led_on_board 涓?TASK LED 缂栫爜鍏变韩鍚屼竴纭欢锛屾牎鍑嗘椂 TASK 涓嶅簲鍦ㄨ繍琛屻€?
 */
void correct_gw_analogue(GW_ANALOGUE *gw_analogue) {
  if (gw_analogue->sta == 0) {
    for (int i = 0; i < 8; i++) {
      select_channel(i);                                             // Select the channel to read from
      for (volatile uint32_t _d = 0; _d < 300; _d++);               // ~3渭s delay for mux settling
      HAL_ADC_Start(&hadc3);                                         // Start the ADC conversion
      HAL_ADC_PollForConversion(&hadc3, 1);                          // Wait for conversion to complete
      gw_analogue->correction_data_w[i] = HAL_ADC_GetValue(&hadc3);  // Get the ADC value
      HAL_ADC_Stop(&hadc3);                                // Stop the ADC conversion
      status.device.buzzer.on = 1;
      status.device.buzzer.off_time = status.state.time + 350;
    }
    status.device.led_on_board.on = 0;
    status.device.led1.on = 0;
    status.device.led2.on = 1;
    gw_analogue->sta = 1;  // Set the state to calibration mode 1
    return;
  }
  if (gw_analogue->sta == 1) {
    for (int i = 0; i < 8; i++) {
      select_channel(i);                                             // Select the channel to read from
      for (volatile uint32_t _d = 0; _d < 300; _d++);               // ~3渭s delay for mux settling
      HAL_ADC_Start(&hadc3);                                         // Start the ADC conversion
      HAL_ADC_PollForConversion(&hadc3, 1);                          // Wait for conversion to complete
      gw_analogue->correction_data_b[i] = HAL_ADC_GetValue(&hadc3);  // Get the ADC value
      HAL_ADC_Stop(&hadc3);                                          // Stop the ADC conversion
      status.device.buzzer.on = 1;
      status.device.buzzer.off_time = status.state.time + 350;
    }
    status.device.led_on_board.on = 0;
    status.device.led1.on = 1;
    status.device.led2.on = 0;
    gw_analogue->sta = 0;  // Set the state to calibration mode 2
    for (int i = 0; i < 8; i++) {
      gw_analogue->digital_low_threshold[i] = gw_analogue->correction_data_b[i] +
                                              (gw_analogue->correction_data_w[i] - gw_analogue->correction_data_b[i]) * 0.33;
      // Calculate the low threshold
      gw_analogue->digital_high_threshold[i] = gw_analogue->correction_data_b[i] +
                                               (gw_analogue->correction_data_w[i] - gw_analogue->correction_data_b[i]) * 0.66;
      // Calculate the high threshold
    }
    for (int i = 0; i < 8; i++) {
      // log_uprintf(&huart1, "%d ", gw_analogue->digital_low_threshold[i]);
    }
    // log_uprintf(&huart1, "\n\n");
    for (int i = 0; i < 8; i++) {
      // log_uprintf(&huart1, "%d ", gw_analogue->digital_high_threshold[i]);
    }
    return;
  }
}

/*
 * 杩熸粸姣旇緝鍣細灏?8 璺師濮?ADC 鍊艰浆鎹负 8bit 鏁板瓧閲忋€?
 * 杈撳叆锛歴tatus.sensor.gw_analogue.channel[0..7]锛堝師濮?ADC锛?
 * 杈撳嚭锛歴tatus.sensor.gw_analogue.digital_8bit锛坆it=1 琛ㄧず璇ヨ矾鐪嬪埌榛戠嚎锛?
 * 瑙勫垯锛?
 *   channel[i] > digital_high_threshold[i] 鈫?bit 娓呴浂锛堢櫧锛屾病绾匡級
 *   channel[i] < digital_low_threshold[i]  鈫?bit 缃?1锛堥粦锛屾湁绾匡級
 *   浠嬩簬涓よ€呬箣闂?鈫?淇濇寔涓婁竴娆＄殑 bit 鍊硷紙杩熸粸甯︼紝闃叉姈鍔級
 * digital_8bit 鍦?init_gw_analogue 鏃舵竻闆朵竴娆★紝涔嬪悗姣忓抚灏卞湴淇敼锛屼笉鏁翠綋澶嶄綅銆?
 * 璋冪敤鏃舵満锛歞river_gw_analogue() 姣忎釜鎺у埗鍛ㄦ湡璋冪敤銆?
 */
void get_gw_analoge_digital_data(GW_ANALOGUE *gw_analogue) {
  for (int i = 0; i < 8; i++) {
    if (gw_analogue->channel[i] > gw_analogue->digital_high_threshold[i]) {
      gw_analogue->digital_8bit &= ~(1 << i);
    } else if (gw_analogue->channel[i] < gw_analogue->digital_low_threshold[i]) {
      gw_analogue->digital_8bit |= (1 << i);
    }
  }
}

/*
 * 鍐呴儴宸ュ叿鍑芥暟锛氬皢鍗曚釜閫氶亾鐨?ADC 鍊肩嚎鎬ф槧灏勫埌 0-100銆?
 * (now - min) / (max - min) * 100
 * 娉ㄦ剰锛氳皟鐢ㄦ柟闇€淇濊瘉 max != min锛屽惁鍒欓櫎闆躲€?
 */
float normalize_gray_data(uint16_t max, uint16_t min, uint16_t now) {
  return (((float)(now - min) / (float)(max - min)) * 100);
}

/*
 * 鍐呴儴宸ュ叿鍑芥暟锛氬皢涓棿 4 璺紶鎰熷櫒鐨勫綊涓€鍖栧€艰浆涓烘潈閲嶏紙4 璺潈閲嶄箣鍜?= 1.0锛夈€?
 * 鍙鐞嗛€氶亾 2-5锛堜腑闂村洓璺級锛岄€氶亾 0,1,6,7 涓嶅弬涓庡惊杩?diff 璁＄畻銆?
 * 鑻?4 璺€诲拰涓?0锛堝畬鍏ㄦ病鐪嬪埌绾匡級锛岀洿鎺?return 涓嶉櫎闆躲€?
 */
void normalize_gray_weight(float *raw_data) {
  float total = 0;
  for (int i = 0; i < 8; i++) {
    total += raw_data[i];
  }
  if (total == 0) {
    return;
  }
  for (int i = 0; i < 8; i++) {
    raw_data[i] = (raw_data[i] / total);
  }

  return;
}

/*
 * 璁＄畻榛戠嚎鐩稿浜庝紶鎰熷櫒涓績鐨勫亸宸€硷紙妯℃嫙寰抗 diff锛夈€?
 * 鍙敤涓棿 4 璺紙閫氶亾 2-5锛屽搴?distance = {-15, -10, 10, 15}锛夛細
 *   1. 瀵规瘡璺?ADC 鍙栧弽褰掍竴鍖栵紙瓒婇粦鍊艰秺澶?鈫?channel 鍊间綆 鈫?buff 鍊奸珮锛夈€?
 *   2. 褰掍竴鍖栦负鏉冮噸锛? 璺潈閲嶅拰=1锛夈€?
 *   3. 鍔犳潈姹傚拰寰楀埌 diff锛堣礋鍊?绾垮亸宸︼紝姝ｅ€?绾垮亸鍙筹紝0=灞呬腑锛夈€?
 * 缁撴灉鍐欏叆 status.sensor.gw_analogue.diff 鈫?follow_line() 鐢ㄦ鍊肩畻 PID 宸€熴€?
 * 璋冪敤鏃舵満锛歞river_gw_analogue() 姣忎釜鎺у埗鍛ㄦ湡璋冪敤銆?
 */
void get_gw_analogue_analogue_diff(GW_ANALOGUE *gw_analogue) {
  float buff[8] = {0};
  float diff = 0;
  for (int i = 0; i < 8; i++) {
    if (gw_analogue->channel[i] < gw_analogue->digital_high_threshold[i])
      buff[i] = 100 - normalize_gray_data(gw_analogue->correction_data_w[i], gw_analogue->correction_data_b[i], gw_analogue->channel[i]);
  }
  normalize_gray_weight(buff);
  for (int i = 0; i < 8; i++) {
    diff += buff[i] * distance[i];
  }

  gw_analogue->diff = diff;
}

/*
 * 鍐呴儴宸ュ叿鍑芥暟锛氬皢涓変釜甯冨皵鏂瑰悜缂栫爜涓?Road 鏋氫妇鍊笺€?
 * L 鈫?bit2(0b100), F 鈫?bit1(0b010), R 鈫?bit0(0b001)
 * 娉ㄦ剰锛氱敱浜庡巻鍙查仐鐣欑殑鍙?BUG 鎶垫秷锛孡 瀵瑰簲鐗╃悊鍙充晶浼犳劅鍣ㄣ€丷 瀵瑰簲鐗╃悊宸︿晶浼犳劅鍣紝
 * 鏋氫妇鍊?LeftRoad/RightRoad 涔熼殢涔嬩氦鍙夈€傚疄鐗╄矾鍙ｅ垽鏂凡楠岃瘉姝ｇ‘锛岀姝慨鏀广€?
 */
enum Road road_new_from_bit(bool L, bool F, bool R) {
  uint8_t left = L ? 0b100 : 0;
  uint8_t font = F ? 0b010 : 0;
  uint8_t right = R ? 0b001 : 0;

  return left | font | right;
}

/*
 * 鍐呴儴鍑芥暟锛氭牴鎹疮绉殑澶氬抚绉垎鍊?+ 褰撳墠甯?data_buf 鍒ゅ畾璺彛绫诲瀷銆?
 * left  = integral 楂?2 浣?== 0b11锛堢墿鐞嗗乏渚т紶鎰熷櫒杩炵画鐪嬪埌绾匡級
 * right = integral 浣?2 浣?== 0b11锛堢墿鐞嗗彸渚т紶鎰熷櫒杩炵画鐪嬪埌绾匡級
 * font  = data_buf 涓棿 4 浣嶄换涓€涓?1锛堜腑闂翠紶鎰熷櫒褰撳墠鐪嬪埌绾匡級
 * 閫氳繃 road_new_from_bit() 缂栫爜鍚庤繑鍥?Road 鏋氫妇銆?
 * 浠呯敱 get_road_type() 鍦?maybe 鍊掕鏃跺埌 1 鏃惰皟鐢ㄣ€?
 */
Road road_decision(Cross *cross) {
  bool left = (cross->integral >> 6) == 0x03;     // 0b1100_0000
  bool right = (cross->integral & 0x03) == 0x03;  // 0b0000_0011
  bool font = cross->data_buf & 0x3C;             // 0b0011_1100
  Road road = road_new_from_bit(left, font, right);
  return road;
}

/*
 * 鍐呴儴鍑芥暟锛氳褰曚竴娆¤矾鍙ｈ娴嬬粨鏋滐紙浼犳劅鍣ㄥ眰锛屼笉鍐冲畾杩愬姩锛夈€?
 * 1. 瀵瑰簲 road 绫诲瀷鐨勮皟璇曡鏁板櫒 +1銆?
 * 2. 鏇存柊 cross->cross = road锛堝啓鍏?status.sensor.gw_analogue.cross.cross锛夈€?
 * 3. 鑻?road 涓嶆槸 Straight/UnknowRoad锛岄€掑 cross->cross_cnt 鍜屽叏灞€ cross_cnt銆?
 * 涓嶄細淇敼 base_speed / motion / wheel.tar_speed / PID / status.task.cross_cnt銆?
 * 鍏ㄥ眬 cross_cnt 浠呬緵璋冭瘯鍜?follow_line 鍏滃簳閫昏緫浣跨敤锛涙寮忎换鍔′互 TASK 鐨?cross_cnt 涓哄噯銆?
 * 浠呯敱 get_road_type() 鍦ㄨ矾鍙ｇ‘璁ゆ垨鍥炲埌 Straight 鏃惰皟鐢ㄣ€?
 */
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
      cross->TRRoad_cnt++;
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

  if (road != Straight && road != UnknowRoad) {
    cross->cross_cnt++;
    cross_cnt++;
  }
}

/*
 * 澶氬抚璺彛妫€娴嬬姸鎬佹満锛屾瘡甯х敱 driver_gw_analogue() 璋冪敤涓€娆°€?
 * 杈撳叆锛歳oad_data = status.sensor.gw_analogue.digital_8bit锛堝綋鍓嶅抚浜屽€煎寲缁撴灉锛?
 * 杈撳嚭锛歴tatus.sensor.gw_analogue.cross.cross锛堣矾鍙ｈ娴嬬粨鏋滐級
 *
 * 鐘舵€佹満閫昏緫锛?
 * A. 褰撳墠涓?Straight锛堟甯稿贰绾夸腑锛夛細
 *    - 鑻ュ渚т紶鎰熷櫒锛坆it7 鎴?bit0锛夌湅鍒扮嚎 鈫?鍚姩 maybe 璁℃暟鍣紙= integral_times=5锛夈€?
 *    - 姣忓抚灏?digital_8bit 鎸変綅 OR 绱Н鍒?integral銆?
 *    - maybe 浠?5 鍑忓埌 1 鐨勮繃绋嬩腑鎸佺画绱Н銆?
 *    - maybe 鍑忓埌 1 鏃讹細璋冪敤 road_decision(integral, data_buf) 鍒ゅ畾璺彛绫诲瀷锛?
 *      璋冪敤 serve_road() 璁板綍缁撴灉锛屾竻闆?maybe 鍜?integral銆?
 * B. 褰撳墠涓洪潪 Straight锛堝凡鍒ゅ畾鍦ㄨ矾鍙ｅ唴锛夛細
 *    - 妫€娴?digital_8bit 鏄惁涓?0x18 / 0x10 / 0x08锛堜腑闂翠紶鎰熷櫒鐪嬪埌绾匡紝澶栦晶妯嚎娑堝け锛夈€?
 *    - 婊¤冻鏉′欢鍒?serve_road(Straight)锛屽洖鍒版甯稿贰绾跨姸鎬併€?
 *    - 娉ㄦ剰锛氬綋鍓嶅洖 Straight 鏉′欢鍋忕獎锛屽疄娴嬪彲鑳介渶瑕佹墿灞曟洿澶氫綅鍥撅紙濡?0x1C, 0x38 绛夛級銆?
 */
void get_road_type(Cross *cross, uint8_t road_data) {
  cross->data_buf = road_data;
  if (cross->cross == Straight) {
    if ((cross->data_buf & 0x81)) {
      if (cross->maybe == 0) {
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
          serve_road(cross, UnknowRoad);
          break;
        case CrossRoad:
          serve_road(cross, CrossRoad);
          break;
        case TBRoad:
          serve_road(cross, TBRoad);
          break;
        case TLRoad:
          serve_road(cross, TLRoad);
          break;
        case TRRoad:
          serve_road(cross, TRRoad);
          break;
        case LeftRoad:
          serve_road(cross, LeftRoad);
          break;
        case RightRoad:
          serve_road(cross, RightRoad);
          break;
        case Straight:
          serve_road(cross, Straight);
          break;
      }
      cross->maybe = 0;
      cross->integral = 0;
    }
  } else if (((road_data & 0x81) == 0) && ((road_data & 0x3C) != 0)) {
    serve_road(cross, Straight);
  }
}

/*
 * 妯℃嫙鐏板害浼犳劅鍣ㄦ€婚┍鍔ㄥ叆鍙ｏ紝姣忎釜鎺у埗鍛ㄦ湡璋冪敤涓€娆°€?
 * 璋冪敤鏂癸細update_status() 鈫?status.c 涓诲惊鐜€?
 * 璋冪敤閾撅紙鎸夐『搴忥級锛?
 *   1. get_gw_raw_data()          鈫?channel[0..7] 鍘熷 ADC
 *   2. get_gw_analoge_digital_data() 鈫?digital_8bit 浜屽€煎寲
 *   3. get_gw_analogue_analogue_diff() 鈫?diff 寰抗鍋忓樊
 *   4. get_road_type()            鈫?cross.cross 璺彛瑙傛祴
 * 璋冪敤瀹屾垚鍚庯紝澶栭儴鍙鍙?status.sensor.gw_analogue 鐨勫叏閮ㄨ娴嬬粨鏋滐細
 *   channel[] / digital_8bit / diff / cross.cross
 */
void driver_gw_analogue(GW_ANALOGUE *gw_analogue) {
  get_gw_raw_data(gw_analogue);
  get_gw_analoge_digital_data(gw_analogue);
  get_gw_analogue_analogue_diff(gw_analogue);
  get_road_type(&gw_analogue->cross, gw_analogue->digital_8bit);
}

