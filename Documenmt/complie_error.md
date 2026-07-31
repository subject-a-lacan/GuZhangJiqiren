*** Using Compiler 'V6.7', folder: 'C:\Keil_v5\ARM\ARMCLANG\Bin'
Build target 'car_control_stm32_project'
../User/Device/led.c(5): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Device/led.c(5): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Device/led.c(5): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling led.c...
../User/Device/button.c(5): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Device/button.c(5): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Device/button.c(5): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling button.c...
../User/Status/status.c(1): warning: In file included from...
../User/Status/status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Status/status.c(1): warning: In file included from...
../User/Status/status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Status/status.c(1): warning: In file included from...
../User/Status/status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
../User/Status/status.c(242): warning: implicit declaration of function 'fabsf' is invalid in C99 [-Wimplicit-function-declaration]
  float diff_limit = fabsf(base);
                     ^
4 warnings generated.
compiling status.c...
../User/It/timer_it.c(6): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/It/timer_it.c(6): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/It/timer_it.c(6): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling timer_it.c...
../User/Device/device_test.c(4): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Device/device_test.c(4): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Device/device_test.c(4): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling device_test.c...
../Core/Src/main.c(32): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../Core/Src/main.c(32): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../Core/Src/main.c(32): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling main.c...
../User/It/uart_it.c(6): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/It/uart_it.c(6): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/It/uart_it.c(6): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling uart_it.c...
../User/Status/ball_control.c(3): warning: In file included from...
../User/Status/status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Status/ball_control.c(3): warning: In file included from...
../User/Status/status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Status/ball_control.c(3): warning: In file included from...
../User/Status/status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling ball_control.c...
../User/Status/Defect.c(2): warning: In file included from...
../User/Status/status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Status/Defect.c(2): warning: In file included from...
../User/Status/status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Status/Defect.c(2): warning: In file included from...
../User/Status/status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling Defect.c...
../User/Status/ball_id.c(4): warning: In file included from...
../User/Status/status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Status/ball_id.c(4): warning: In file included from...
../User/Status/status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Status/ball_id.c(4): warning: In file included from...
../User/Status/status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
../User/Status/ball_id.c(67): error: expected expression
    plan[0]  = {-1, low[0]};  plan[1]  = {-1, low[1]};  plan[2]  = {-1, low[2]};
               ^
../User/Status/ball_id.c(67): error: expected expression
    plan[0]  = {-1, low[0]};  plan[1]  = {-1, low[1]};  plan[2]  = {-1, low[2]};
                                         ^
../User/Status/ball_id.c(67): error: expected expression
    plan[0]  = {-1, low[0]};  plan[1]  = {-1, low[1]};  plan[2]  = {-1, low[2]};
                                                                   ^
../User/Status/ball_id.c(68): error: expected expression
    plan[3]  = { 1, high[0]}; plan[4]  = { 1, high[1]}; plan[5]  = { 1, high[2]};
               ^
../User/Status/ball_id.c(68): error: expected expression
    plan[3]  = { 1, high[0]}; plan[4]  = { 1, high[1]}; plan[5]  = { 1, high[2]};
                                         ^
../User/Status/ball_id.c(68): error: expected expression
    plan[3]  = { 1, high[0]}; plan[4]  = { 1, high[1]}; plan[5]  = { 1, high[2]};
                                                                   ^
../User/Status/ball_id.c(69): error: expected expression
    plan[6]  = { 1, high[2]}; plan[7]  = { 1, high[1]}; plan[8]  = { 1, high[0]};
               ^
../User/Status/ball_id.c(69): error: expected expression
    plan[6]  = { 1, high[2]}; plan[7]  = { 1, high[1]}; plan[8]  = { 1, high[0]};
                                         ^
../User/Status/ball_id.c(69): error: expected expression
    plan[6]  = { 1, high[2]}; plan[7]  = { 1, high[1]}; plan[8]  = { 1, high[0]};
                                                                   ^
../User/Status/ball_id.c(70): error: expected expression
    plan[9]  = {-1, low[2]};  plan[10] = {-1, low[1]};  plan[11] = {-1, low[0]};
               ^
../User/Status/ball_id.c(70): error: expected expression
    plan[9]  = {-1, low[2]};  plan[10] = {-1, low[1]};  plan[11] = {-1, low[0]};
                                         ^
../User/Status/ball_id.c(70): error: expected expression
    plan[9]  = {-1, low[2]};  plan[10] = {-1, low[1]};  plan[11] = {-1, low[0]};
                                                                   ^
3 warnings and 12 errors generated.
compiling ball_id.c...
../User/Tool/pid.c(7): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Tool/pid.c(7): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Tool/pid.c(7): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling pid.c...
../User/Motor/wheel.c(6): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Motor/wheel.c(6): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Motor/wheel.c(6): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling wheel.c...
../User/Motor/servo.c(6): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Motor/servo.c(6): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Motor/servo.c(6): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling servo.c...
../User/Sensor/gw_analogue.c(6): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Sensor/gw_analogue.c(6): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Sensor/gw_analogue.c(6): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling gw_analogue.c...
../User/Sensor/maixcam.c(4): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(7): warning: 'ENCODER_PPR' macro redefined [-Wmacro-redefined]
#define ENCODER_PPR         (13)
        ^
../User/Tool\pid.h(31): note: previous definition is here
#define ENCODER_PPR 13                    // <U+7F16><U+7801><U+5668><U+7EBF><U+6570>
        ^
../User/Sensor/maixcam.c(4): warning: In file included from...
../User/Status\status.h(17): warning: In file included from...
../User/Tool\car_speed_profile.h(9): warning: 'GEAR_RATIO' macro redefined [-Wmacro-redefined]
#define GEAR_RATIO          (28.0f)
        ^
../User/Tool\pid.h(32): note: previous definition is here
#define GEAR_RATIO 28.0f                  // <U+51CF><U+901F><U+6BD4> 1:28<U+FF08><U+7535><U+673A>:<U+8F66><U+8F6E><U+FF09>
        ^
../User/Sensor/maixcam.c(4): warning: In file included from...
../User/Status\status.h(225): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling maixcam.c...
"car_control_stm32_project\car_control_stm32_project.axf" - 12 Error(s), 46 Warning(s).
Target not created.
Build Time Elapsed:  00:00:03