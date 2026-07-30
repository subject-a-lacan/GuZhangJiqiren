#include "status.h"
#include "gw_anagloge.h"
#include "button.h"
#include "buzzer.h"
#include "Defect.h"
#include "i2c.h"
#include "led.h"
#include "log.h"
#include "math_tool.h"
#include "servo.h"
#include "wheel.h"
#include "uart_it.h"

STATUS status;

extern int16_t cmd_speed;
int32_t rw_time_cur = -1;
int32_t rw_time_tar = -1;
extern uint8_t cross_cnt;      // 璺彛璁℃暟鍣?
uint8_t cross_delay = 0;       // 璺彛寤舵椂璁℃暟鍣?
uint8_t speed_show_flag = 0;   // 鏄剧ず閫熷害鏍囧織浣?

/**
 * @brief 鍒濆鍖栬埖鏈哄拰杞瓙
 * @param 鏃?
 * @return 鏃?
 *@note 璋冪敤 init_servo(&status.motor.servo[0], 1, 180)
 *       鎶?1 鍙疯埖鏈烘寕鍒扮姸鎬佹爲閲岋紝180 鏄繖涓埖鏈虹殑鏈€澶ц搴︼紝鍚庣画鍙寜瀹炵墿淇敼
 *@note 璋冪敤 init_servo(&status.motor.servo[1], 2, 270)
 *       鎶?2 鍙疯埖鏈烘寕鍒扮姸鎬佹爲閲岋紝270 鏄繖涓埖鏈虹殑鏈€澶ц搴︼紝鍚庣画鍙寜瀹炵墿淇敼
 *@note init_wheel(&status.motor.wheel[0], 1, -1): MOTOR0 front-right wheel
 *@note init_wheel(&status.motor.wheel[1], 2, 1):  MOTOR1 front-left wheel
 *@note init_wheel(&status.motor.wheel[2], 3, 1):  MOTOR2 rear-right wheel
 *@note init_wheel(&status.motor.wheel[3], 4, 1):  MOTOR3 rear-left wheel

 */
void init_motor() {
  init_servo(&status.motor.servo[0], 1, 180);
  init_servo(&status.motor.servo[1], 2, 270);

  init_wheel(&status.motor.wheel[0], 1, -1);
  init_wheel(&status.motor.wheel[1], 2, 1);
  init_wheel(&status.motor.wheel[2], 3, 1);
  init_wheel(&status.motor.wheel[3], 4, 1);

  return;
}

/**
 * @brief 鍒濆鍖栨寜閿€佺伅鍜岃渹楦ｅ櫒
 * @param 鏃?
 * @return 鏃?
 *@note 璋冪敤 init_button(&status.device.button_D2, 1, 0)
 *       鍒濆鍖?D2 鎸夐敭锛? 鏄寜閿紪鍙凤紝0 琛ㄧず浣庣數骞虫寜涓嬶紝鍚庣画鍙寜鎺ョ嚎鏀?
 *@note 璋冪敤 init_button(&status.device.button_B11, 2, 0)
 *       鍒濆鍖?B11 鎸夐敭锛? 鏄寜閿紪鍙凤紝0 琛ㄧず浣庣數骞虫寜涓嬶紝鍚庣画鍙寜鎺ョ嚎鏀?
 *@note 璋冪敤 init_LED(&status.device.led_on_board, 1, 0)
 *       鍒濆鍖栨澘杞界伅锛? 鏄澶囩紪鍙凤紝0 琛ㄧず浣庣數骞崇偣浜紝鍚庣画鍙寜鐢佃矾鏀?
 *@note 璋冪敤 init_LED(&status.device.led1, 2, 0)
 *       鍒濆鍖栧鎺?LED1锛? 鏄澶囩紪鍙凤紝鐐逛寒鐢靛钩鏂瑰紡鍚庣画鍙皟
 *@note 璋冪敤 init_LED(&status.device.led2, 3, 0)
 *       鍒濆鍖栧鎺?LED2锛? 鏄澶囩紪鍙凤紝鐐逛寒鐢靛钩鏂瑰紡鍚庣画鍙皟
 *@note 璋冪敤 init_BUZZER(&status.device.buzzer, 1, 1)
 *       鍒濆鍖栬渹楦ｅ櫒锛? 鏄澶囩紪鍙凤紝1 琛ㄧず楂樼數骞冲搷锛屽悗缁彲鎸夌數璺敼
 */
void init_device() {
  init_button(&status.device.button_D2, 1, 0);
  init_button(&status.device.button_B11, 2, 0);
  init_LED(&status.device.led_on_board, 1, 1);
  init_LED(&status.device.led1, 2, 1);
  init_LED(&status.device.led2, 3, 1);
  init_BUZZER(&status.device.buzzer, 1, 1);

  return;
}

/**
 * @brief 鍒濆鍖栦紶鎰熷櫒
 * @param status 鐘舵€佺粨鏋勪綋鎸囬拡锛岀敤鏉ユ壘鍒板悇涓紶鎰熷櫒鍦ㄧ姸鎬佹爲涓殑浣嶇疆
 * @return 鏃?
 *@note 璋冪敤 iic_gyr_init(&status->sensor.gy901)
 *       鍒濆鍖栭檧铻轰华鐨勭紦瀛樺尯銆佽澶囧湴鍧€鍜岃捣濮嬪瘎瀛樺櫒鍦板潃锛屽叾涓湴鍧€鍙傛暟鍚庣画鍙皟
 *@note 璋冪敤 init_gw_8bit(&status->sensor.gw_8bit)
 *       鍒濆鍖?8 璺暟瀛楃伆搴︾殑鏉冮噸銆佽矾鍙ｇ紦瀛樺拰宸＄嚎 PID锛屽叾涓潈閲嶅拰 PID 鍙傛暟鍚庣画鍙皟
 *@note 璋冪敤 init_gw_analogue(&status->sensor.gw_analogue)
 *       鍒濆鍖?8 璺ā鎷熺伆搴︾殑閫氶亾鍊笺€侀槇鍊煎拰鏍″噯鏁版嵁锛屽叾涓槇鍊煎弬鏁板悗缁彲璋?
 */
void init_sensor(STATUS *status) {
  iic_gyr_init(&status->sensor.gy901);
  init_gw_analogue(&status->sensor.gw_analogue);
}


/**
 * @brief 鍒濆鍖栧皬杞﹁繍琛屾椂瑕佺敤鍒扮殑鍩虹鐘舵€?
 * @param status 鐘舵€佺粨鏋勪綋鎸囬拡锛岀敤鏉ュ啓鍏ユ暣杞︾殑鍒濆鐘舵€?
 * @param T 绯荤粺鎺у埗鍛ㄦ湡锛屽崟浣?ms锛岃〃绀虹姸鎬佸涔呮洿鏂颁竴娆?
 * @return 鏃?
 *@note 杩欓噷浼氭妸鏃堕棿娓呴浂锛屽苟鎶婅繍鍔ㄦā寮忓厛璁炬垚 STOP锛屼繚璇佷笂鐢垫椂杞︿笉浼氱洿鎺ュ姩
 *@note 杩欓噷浼氱粰褰撳墠瑙掑害銆佺洰鏍囪搴︺€佸熀纭€閫熷害鍜岀伆搴︾姸鎬佽繖浜涜繍琛屽彉閲忚榛樿鍊?
 *@note 杩欓噷浼氭妸 road_determine 閲岀殑璺彛缂撳瓨銆佽鏁板拰鍒よ矾鍙傛暟璁炬垚鍒濆€?
 *       鍏朵腑 integral_times = 6 鏄悗缁彲璋冪殑涓€涓伒鏁忓害鍙傛暟
 *@note 鍙傛暟 T 浼氬奖鍝嶅悗缁帶鍒惰妭濂忥紝鏄悗缁父璋冪殑鍩虹鍙傛暟
 */
void init_state(STATUS *status, uint8_t T) {
  status->state.T = T;
  status->state.time = 0;
  status->state.motion = STOP;
  status->state.cur_angle = 0;
  status->state.tar_angle = 90;

  status->state.gw_8bit = 0x00;

  status->state.base_speed = 0;

  status->state.motion = STOP;

  return;
}

/**
 * @brief 鍒濆鍖栫姸鎬佸眰鎺у埗鐢ㄧ殑 PID
 * @param status 鐘舵€佺粨鏋勪綋鎸囬拡锛岀敤鏉ヤ繚瀛樺贰绾垮拰淇濊 PID
 * @return 鏃?
 *@note 璋冪敤 init_pid(1.5, 0, 0, 8, 20)
 *       鍒濆鍖栧贰绾?PID锛岃繖鍑犱釜鏁板瓧鍒嗗埆鎺у埗璺熺嚎鍙嶅簲蹇參鍜岀Н鍒嗛檺鍒讹紝鍚庣画閮藉彲璋?
 *@note 璋冪敤 init_pid(1, 0, 1, 8, 20)
 *       鍒濆鍖栦繚瑙?PID锛岃繖鍑犱釜鏁板瓧鍐冲畾杞悜绾犲亸鍔涘害鍜岀ǔ瀹氭€э紝鍚庣画閮藉彲璋?
 */
void init_status_pid(STATUS *status) {
  status->state.status_pid.follow_line_pid = init_pid(1.8, 0.006, 0.4, 5,1, 0.0f);
  status->state.status_pid.keep_angle_pid = init_pid(1.2, 0.4, 0, 5,1, 0.0f);
  status->state.status_pid.balance_pid = init_pid(0.1, 0, 180, 5,1, 0.0f);
  status->state.status_pid.mileage_pid = init_pid(0.00035, 0, 6, 5,1, 0.0f);
  status->state.status_pid.angle_output_limit = 25.0f;
}

static void apply_control_param(STATUS *status, CONTROL_PARAM p) {
  status->state.status_pid.follow_line_pid = p.follow_line_pid;
  status->state.status_pid.keep_angle_pid = p.keep_angle_pid;
  status->motor.wheel[0].wheel_pid = p.wheel_right_pid;
  status->motor.wheel[1].wheel_pid = p.wheel_left_pid;
  set_wheel_ff_param_by_which(1, p.ff_offset, p.ff_k, p.ff_min);
  set_wheel_ff_param_by_which(2, p.ff_offset_r, p.ff_k_r, p.ff_min_r);
  status->state.status_pid.angle_output_limit = 25.0f;
}

void apply_basic_control_param(STATUS *status) {
  CONTROL_PARAM p;
  p.follow_line_pid = init_pid(1, 0.03, 0, 5,1, 0.0f);
  p.keep_angle_pid  = init_pid(1, 0, 0, 5,1, 0.0f);
  p.wheel_left_pid  = init_pid(8, 0, 0, 5,100, 0.50f);
  p.wheel_right_pid = init_pid(8, 0, 0, 5,100, 0.50f);
  p.ff_offset = 200.68f;
  p.ff_k = 47.24f;
  p.ff_min = 250.0f;
  p.ff_offset_r = 107.12f;
  p.ff_k_r = 46.27f;
  p.ff_min_r = 260.0f;
  apply_control_param(status, p);
}

void apply_adv_control_param(STATUS *status) {
  CONTROL_PARAM p;
  p.follow_line_pid = init_pid(1, 0.03, 0, 5,1, 0.0f);   // TODO: 璐熼噸鍚庡疄杞︽爣瀹?  p.keep_angle_pid  = init_pid(1, 0, 0, 5,1, 0.0f);       // TODO: 璐熼噸鍚庡疄杞︽爣瀹?  p.wheel_left_pid  = init_pid(8, 0, 0, 5,100, 0.50f);    // TODO: 璐熼噸鍚庡疄杞︽爣瀹?  p.wheel_right_pid = init_pid(8, 0, 0, 5,100, 0.50f);    // TODO: 璐熼噸鍚庡疄杞︽爣瀹?  p.ff_offset = 200.68f;   // TODO: 璐熼噸鍚庡疄杞︽爣瀹?
  p.ff_k = 47.24f;         // TODO: 璐熼噸鍚庡疄杞︽爣瀹?
  p.ff_min = 250.0f;       // TODO: 璐熼噸鍚庡疄杞︽爣瀹?
  p.ff_offset_r = 107.12f; // TODO: 璐熼噸鍚庡疄杞︽爣瀹?
  p.ff_k_r = 46.27f;       // TODO: 璐熼噸鍚庡疄杞︽爣瀹?
  p.ff_min_r = 260.0f;     // TODO: 璐熼噸鍚庡疄杞︽爣瀹?
  apply_control_param(status, p);
}

/**
 * @brief 鍒濆鍖栨暣妫?status 鐘舵€佹爲
 * @param status 鐘舵€佺粨鏋勪綋鎸囬拡锛屾暣杞︽墍鏈夌姸鎬侀兘浼氭寕鍦ㄨ繖閲?
 * @param T 绯荤粺鎺у埗鍛ㄦ湡锛屽崟浣?ms
 * @return 鏃?
 *@note 璋冪敤 init_state(status, T)
 *       鍏堟妸鏃堕棿銆佹ā寮忋€佽搴︺€佸熀纭€閫熷害鍜岃矾鍙ｅ垽鏂紦瀛樿鎴愯捣濮嬪€硷紝鍏朵腑 T 鍜?integral_times 鍚庣画鍙皟
 *@note 璋冪敤 init_status_pid(status)
 *       鎶婂贰绾?PID 鍜屼繚瑙?PID 鍏堝噯澶囧ソ锛岃繖涓ょ粍鍙傛暟鍚庣画璋冭溅鏃跺父鏀?
 *@note 璋冪敤 init_sensor(status)
 *       鎶婇檧铻轰华銆佹暟瀛楃伆搴︺€佹ā鎷熺伆搴︾殑榛樿鍙傛暟鍑嗗濂斤紝鍦板潃銆侀槇鍊笺€佹潈閲嶇瓑鍚庣画鍙皟
 *@note 璋冪敤 init_motor()
 *       鎶婅埖鏈哄拰杞瓙涓庡疄闄呯‖浠堕€氶亾瀵瑰簲璧锋潵锛岃埖鏈烘渶澶ц搴﹀拰杞瓙鏂瑰悜鍙傛暟鍚庣画鍙皟
 *@note 璋冪敤 init_device()
 *       鎶婃寜閿€丩ED銆佽渹楦ｅ櫒鐨勭紪鍙峰拰鐢靛钩閫昏緫璁惧ソ锛岀數骞虫湁鏁堟柟寮忓悗缁彲璋?
 */
void init_status(STATUS *status, uint8_t T) {
  init_state(status, T);

  init_status_pid(status);

  init_sensor(status);

  init_motor();

  init_device();

  init_task(&status->task);

  return;
}

Road road_buf = Straight;  //瀛樺偍涓婁竴娆℃娴嬪埌鐨勮矾鍙ｇ被鍨?

/*
 * @brief 鍒ゆ柇褰撳墠璺喌鏄洿琛岃繕鏄浆寮?
 * @param 鏃?
 * @return 璺彛绫诲瀷
 */
Road Turn_or_Straight() {
  if (road_buf != status.sensor.gw_analogue.cross.cross) {
    status.motor.wheel[0].tar_speed = 0; //璺喌鍙戠敓鍙樺寲灏卞厛鍋滆溅
    status.motor.wheel[1].tar_speed = 0;
    if ((ABS(status.motor.wheel[0].cur_speed) < 2) && (ABS(status.motor.wheel[1].cur_speed) < 2)) {
      road_buf = status.sensor.gw_analogue.cross.cross;
    }
  }
  // if (status.state.road_determine.cross == LeftRoad && left_cnt == 1) {
  //   status.state.base_speed = 60;
  //   status.state.road_determine.integral = 4;
  // }  杩樻病鐪嬫噦涓轰粈涔堢壒鍖栧乏杞?
  return road_buf;
}
/*
 * @brief 宸＄嚎鎺у埗锛堢函 PID + 宸€燂級銆?
 *        鍙牴鎹綋鍓嶄紶鎰熷櫒鍋忓樊璁＄畻宸﹀彸杞洰鏍囬€熷害锛屼笉鍒ゆ柇璺彛锛屼笉鎵ц杞集銆?
 *        璺彛瑙傛祴缁撴灉鐢?driver_gw_analogue 鍐欏叆 status.sensor.gw_analogue.cross.cross锛?
 *        杞集/鍋滆溅/璁℃暟鐢?update_task 鍐呯殑灏忕姸鎬佹満鏍规嵁 race_phase 鍐冲畾銆?
 * @param status 鐘舵€佺粨鏋勪綋鎸囬拡
 * @return 鏃?
 */
void follow_line(STATUS *status) {
  float diff = compute_pid(&status->state.status_pid.follow_line_pid, status->sensor.gw_analogue.diff);
  status->motor.wheel[0].tar_speed = (float)status->state.base_speed - diff;
  status->motor.wheel[1].tar_speed = (float)status->state.base_speed + diff;
}

void keep_angle(STATUS *status) {
  float target = status->state.tar_angle + status->state.initial_angle;  // 鐩爣瑙掑害
  float diff_angle = target - status->state.cur_angle;
  if (diff_angle > 180.0) {
    diff_angle -= 360.0;
  } else if (diff_angle < -180.0) {
    diff_angle += 360.0;
  }
  float diff = compute_pid(&status->state.status_pid.keep_angle_pid, diff_angle);  // PID璁＄畻
  float angle_limit = status->state.status_pid.angle_output_limit;
  diff = CONFINE(diff, -angle_limit, angle_limit);                                 // 闄愬埗閫熷害鑼冨洿
  status->motor.wheel[0].tar_speed = (float)status->state.base_speed + diff;
  status->motor.wheel[1].tar_speed = (float)status->state.base_speed - diff;  // 璁剧疆鐢垫満閫熷害
}

void keep_balance(STATUS *status) {
  float roll = iic_gyr_get_value(&status->sensor.gy901, gyr_x_roll);
  float roll_gyro = iic_gyr_get_value(&status->sensor.gy901, gyr_w_x);
  float target = 0;
  float error = target - roll;
  PID *bp = &status->state.status_pid.balance_pid;
  float balance_out = bp->kp * error + bp->kd * roll_gyro;
  status->state.base_speed = (int16_t)balance_out;
  status->motor.wheel[0].tar_speed = status->state.base_speed;
  status->motor.wheel[1].tar_speed = status->state.base_speed;
}

/*
 * @brief 鏇存柊鎸夐挳鐘舵€?璋冪敤srver_button鍑芥暟鎵ц鍏蜂綋鎸夐敭閫昏緫
 * @param status 鐘舵€佺粨鏋勪綋鎸囬拡
 * @return 鏃?
 */
void update_status(STATUS *status) {
  consume_uart_gyr();
  if (gyro_dma_ready) {
    gyro_dma_ready = 0;
    status->state.cur_angle = iic_gyr_get_value(&status->sensor.gy901, gyr_z_yaw);
    if (!iic_gyr_initial_ready) {
      status->state.initial_angle = status->state.cur_angle;
      iic_gyr_initial_ready = 1;
    }
  }
  status->motor.wheel[0].cur_speed = get_wheel_speed(&status->motor.wheel[0]);
  status->motor.wheel[1].cur_speed = get_wheel_speed(&status->motor.wheel[1]);
  status->motor.wheel[2].cur_speed = get_wheel_speed(&status->motor.wheel[2]);
  status->motor.wheel[3].cur_speed = get_wheel_speed(&status->motor.wheel[3]);

  

  // log_uprintf(&huart1, "%d %d %d %d\r\n", cross_cnt, cross_delay, Turn_or_Straight(), status->state.road_determine.cross);

  driver_button(&status->device.button_D2);
  driver_button(&status->device.button_B11);

  update_task(status);
 if (status->state.motion == FIND_LINE) {
    status->task.stop_cmd = 0;
    if (!(status->task.armed && status->task.task_running)) {
      status->state.base_speed = cmd_speed;
    }
    follow_line(status);
  }
  if (status->state.motion == KEEP_ANGLE) {
    status->task.stop_cmd = 0;
    keep_angle(status);
  }
  if (status->state.motion == BALANCE) {
    status->task.stop_cmd = 0;
    keep_balance(status);
  }
  if (status->state.motion == STRAIGHT) {
    status->task.stop_cmd = 0;
    status->motor.wheel[0].tar_speed = status->state.base_speed;
    status->motor.wheel[1].tar_speed = status->state.base_speed;
  }
  if (status->state.motion == STOP) {
    status->task.stop_cmd = 1;
    status->motor.wheel[0].tar_speed = 0;
    status->motor.wheel[1].tar_speed = 0;
  }
  if (status->state.motion == MOTOR_TEST) {
    status->task.stop_cmd = 0;
    status->state.base_speed = cmd_speed;
    status->motor.wheel[0].tar_speed = cmd_speed;
    status->motor.wheel[1].tar_speed = cmd_speed;
  }
  driver_LED(&status->device.led_on_board);
  driver_LED(&status->device.led1);
  driver_LED(&status->device.led2);

  // driver_servo(&status->motor.servo[0]);
  // driver_servo(&status->motor.servo[1]); //鑸垫満杞姩瀹氳搴︼紙缁撴瀯浣撳厓绱犵‘瀹?

  if (status->device.buzzer.on && status->state.time >= status->device.buzzer.off_time) {
    status->device.buzzer.on = 0;
  }
  driver_BUZZER(&status->device.buzzer);

  driver_wheel(&status->motor.wheel[0]);
  driver_wheel(&status->motor.wheel[1]);
  driver_wheel(&status->motor.wheel[2]);
  driver_wheel(&status->motor.wheel[3]);

  if (!gyro_dma_busy) {
    iic_gyr_read_dma(&hi2c1, &status->sensor.gy901);
  }

  return;
}

void driver_status(STATUS *status) {  // 
}
/*
 * @brief 鍒濆鍖栫姸鎬佸悗鏇存柊鍒濆瑙掑害 閬垮厤璇诲彇鏃х紦瀛樺€?
 * @param 鏃?
 * @return 鏃?
 */
void after_init_state() {
  status.state.initial_angle = iic_gyr_get_value(&status.sensor.gy901, gyr_z_yaw);
}

