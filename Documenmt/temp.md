*** Using Compiler 'V6.7', folder: 'C:\Keil_v5\ARM\ARMCLANG\Bin'
Build target 'car_control_stm32_project'
../User/Status/Defect.c(2): warning: In file included from...
../User/Status/status.h(134): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
../User/Status/Defect.c(166): error: member reference type 'STATUS *' (aka 'struct STATUS *') is a pointer; did you mean to use '->'?
../User/Status/Defect.c(167): error: member reference type 'STATUS *' (aka 'struct STATUS *') is a pointer; did you mean to use '->'?
      (float)status.sensor.uart_gyr.gyro_z,
             ~~~~~~^
                   ->
../User/Status/Defect.c(168): error: member reference type 'STATUS *' (aka 'struct STATUS *') is a pointer; did you mean to use '->'?
      (float)status.sensor.uart_gyr.yaw);
             ~~~~~~^
                   ->
1 warning and 3 errors generated.
compiling Defect.c...
"car_control_stm32_project\car_control_stm32_project.axf" - 3 Error(s), 1 Warning(s).
Target not created.