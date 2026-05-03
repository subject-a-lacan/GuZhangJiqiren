../User/Device/button.c(5): warning: In file included from...
../User/Status\status.h(114): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
1 warning generated.
compiling button.c...
../User/Tool/pid.c(7): warning: In file included from...
../User/Status\status.h(114): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
1 warning generated.
compiling pid.c...
../User/Status/status.c(1): warning: In file included from...
../User/Status/status.h(114): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
1 warning generated.
compiling status.c...
../User/Sensor/ccd.c(29): error: use of undeclared identifier 'CCD_CLK_GPIO_Port'
    CLK_DOWN;
    ^
../User/Sensor/ccd.h(24): note: expanded from macro 'CLK_DOWN'
#define CLK_DOWN HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, 0)
                                   ^
../User/Sensor/ccd.c(29): error: use of undeclared identifier 'CCD_CLK_Pin'
../User/Sensor/ccd.h(24): note: expanded from macro 'CLK_DOWN'
#define CLK_DOWN HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, 0)
                                                      ^
../User/Sensor/ccd.c(30): error: use of undeclared identifier 'CCD_SI_GPIO_Port'
    SI_DOWN;
    ^
../User/Sensor/ccd.h(26): note: expanded from macro 'SI_DOWN'
#define SI_DOWN HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, 0)
                                  ^
../User/Sensor/ccd.c(30): error: use of undeclared identifier 'CCD_SI_Pin'
../User/Sensor/ccd.h(26): note: expanded from macro 'SI_DOWN'
#define SI_DOWN HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, 0)
                                                    ^
../User/Sensor/ccd.c(34): error: use of undeclared identifier 'CCD_CLK_GPIO_Port'
    CLK_UP;
    ^
../User/Sensor/ccd.h(23): note: expanded from macro 'CLK_UP'
#define CLK_UP HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, 1)
                                 ^
../User/Sensor/ccd.c(34): error: use of undeclared identifier 'CCD_CLK_Pin'
../User/Sensor/ccd.h(23): note: expanded from macro 'CLK_UP'
#define CLK_UP HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, 1)
                                                    ^
../User/Sensor/ccd.c(35): error: use of undeclared identifier 'CCD_SI_GPIO_Port'
    SI_UP;
    ^
../User/Sensor/ccd.h(25): note: expanded from macro 'SI_UP'
#define SI_UP HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, 1)
                                ^
../User/Sensor/ccd.c(35): error: use of undeclared identifier 'CCD_SI_Pin'
../User/Sensor/ccd.h(25): note: expanded from macro 'SI_UP'
#define SI_UP HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, 1)
                                                  ^
../User/Sensor/ccd.c(39): error: use of undeclared identifier 'CCD_CLK_GPIO_Port'
    CLK_UP;
    ^
../User/Sensor/ccd.h(23): note: expanded from macro 'CLK_UP'
#define CLK_UP HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, 1)
                                 ^
../User/Sensor/ccd.c(39): error: use of undeclared identifier 'CCD_CLK_Pin'
../User/Sensor/ccd.h(23): note: expanded from macro 'CLK_UP'
#define CLK_UP HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, 1)
                                                    ^
../User/Sensor/ccd.c(40): error: use of undeclared identifier 'CCD_SI_GPIO_Port'
    SI_DOWN;
    ^
../User/Sensor/ccd.h(26): note: expanded from macro 'SI_DOWN'
#define SI_DOWN HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, 0)
                                  ^
../User/Sensor/ccd.c(40): error: use of undeclared identifier 'CCD_SI_Pin'
../User/Sensor/ccd.h(26): note: expanded from macro 'SI_DOWN'
#define SI_DOWN HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, 0)
                                                    ^
../User/Sensor/ccd.c(44): error: use of undeclared identifier 'CCD_CLK_GPIO_Port'
    CLK_DOWN;
    ^
../User/Sensor/ccd.h(24): note: expanded from macro 'CLK_DOWN'
#define CLK_DOWN HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, 0)
                                   ^
../User/Sensor/ccd.c(44): error: use of undeclared identifier 'CCD_CLK_Pin'
../User/Sensor/ccd.h(24): note: expanded from macro 'CLK_DOWN'
#define CLK_DOWN HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, 0)
                                                      ^
../User/Sensor/ccd.c(45): error: use of undeclared identifier 'CCD_SI_GPIO_Port'
    SI_DOWN;
    ^
../User/Sensor/ccd.h(26): note: expanded from macro 'SI_DOWN'
#define SI_DOWN HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, 0)
                                  ^
../User/Sensor/ccd.c(45): error: use of undeclared identifier 'CCD_SI_Pin'
../User/Sensor/ccd.h(26): note: expanded from macro 'SI_DOWN'
#define SI_DOWN HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, 0)
                                                    ^
../User/Sensor/ccd.c(49): warning: incompatible pointer types passing 'uint16_t *' (aka 'unsigned short *') to parameter of type 'uint32_t *' (aka 'unsigned int *') [-Wincompatible-pointer-types]
    HAL_ADC_Start_DMA(&hadc3, &BUFF_DATA_1[(cnt - 10) / 3], 1);
                              ^~~~~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32G4xx_HAL_Driver/Inc\stm32g4xx_hal_adc.h(1959): note: passing argument to parameter 'pData' here
HAL_StatusTypeDef       HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
                                                                             ^
../User/Sensor/ccd.c(53): error: use of undeclared identifier 'CCD_CLK_GPIO_Port'
    CLK_UP;
    ^
../User/Sensor/ccd.h(23): note: expanded from macro 'CLK_UP'
#define CLK_UP HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, 1)
                                 ^
../User/Sensor/ccd.c(53): error: use of undeclared identifier 'CCD_CLK_Pin'
../User/Sensor/ccd.h(23): note: expanded from macro 'CLK_UP'
#define CLK_UP HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, 1)
                                                    ^
../User/Sensor/ccd.c(54): error: use of undeclared identifier 'CCD_SI_GPIO_Port'
    SI_DOWN;
    ^
../User/Sensor/ccd.h(26): note: expanded from macro 'SI_DOWN'
#define SI_DOWN HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, 0)
                                  ^
fatal error: too many errors emitted, stopping now [-ferror-limit=]
1 warning and 20 errors generated.
compiling ccd.c...
"car_control_stm32_project\car_control_stm32_project.axf" - 19 Error(s), 12 Warning(s).