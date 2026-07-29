../User/Motor/wheel.c(6): warning: In file included from...
../User/Status\status.h(132): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
1 warning generated.
compiling wheel.c...
../User/Sensor/gw_analogue.c(6): warning: In file included from...
../User/Status\status.h(132): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
1 warning generated.
compiling gw_analogue.c...
linking...
car_control_stm32_project\car_control_stm32_project.axf: Error: L6218E: Undefined symbol uart_gyr_init (referred from uart_it.o).
car_control_stm32_project\car_control_stm32_project.axf: Error: L6218E: Undefined symbol uart_gyr_rx_feed (referred from uart_it.o).
car_control_stm32_project\car_control_stm32_project.axf: Error: L6218E: Undefined symbol uart_gyr_start_receive (referred from uart_it.o).
car_control_stm32_project\car_control_stm32_project.axf: Error: L6218E: Undefined symbol uart_gyr_get_yaw (referred from defect.o).
car_control_stm32_project\car_control_stm32_project.axf: Error: L6218E: Undefined symbol uart_gyr_get_z (referred from defect.o).
Not enough information to list image symbols.