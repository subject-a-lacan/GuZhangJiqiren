*** Using Compiler 'V6.7', folder: 'C:\Keil_v5\ARM\ARMCLANG\Bin'
Build target 'car_control_stm32_project'
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
../User/Status/status.h(220): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
../User/Status/ball_control.c(297): error: member reference type 'STATUS *' (aka 'struct STATUS *') is a pointer; did you mean to use '->'?
        (uint32_t)status.state.time - BALL_STEPPER_MIN_PUBLISH_MS;
                  ~~~~~~^
                        ->
../User/Status/ball_control.c(443): error: member reference type 'STATUS *' (aka 'struct STATUS *') is a pointer; did you mean to use '->'?
    uint32_t now_ms = (uint32_t)status.state.time;
                                ~~~~~~^
                                      ->
3 warnings and 2 errors generated.
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
../User/Status/status.h(220): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
3 warnings generated.
compiling Defect.c...
"car_control_stm32_project\car_control_stm32_project.axf" - 2 Error(s), 6 Warning(s).
Target not created.
Build Time Elapsed:  00:00:01