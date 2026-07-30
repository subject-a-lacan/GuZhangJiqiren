../User/Motor/yuntai.c(5): warning: implicit declaration of function 'ABS' is invalid in C99 [-Wimplicit-function-declaration]
void driver_yuntai(YUNTAI*y,float x,float p){point_calculation(y,x,p);float dx=x-y->temp_x,dy=p-y->temp_y,d=sqrtf(dx*dx+dy*dy);if(d>0.001f){y->yaw_datou.speed=(uint16_t)(y->yuntai_speed*ABS(dx)/d);y->pitch_datou.speed=(uint16_t)(y->yuntai_speed*ABS(dy)/d);}driver_datou(&y->yaw_datou);driver_datou(&y->pitch_datou);y->temp_x=x;y->temp_y=p;}
                                                                                                                                                                                          ^
1 warning generated.
compiling yuntai.c...
../User/Status/Defect.c(2): warning: In file included from...
../User/Status/status.h(134): warning: redefinition of typedef 'STATUS' is a C11 feature [-Wtypedef-redefinition]
} STATUS;
  ^
../User/Status/Defect.h(6): note: previous definition is here
typedef struct STATUS STATUS;
                      ^
1 warning generated.
compiling Defect.c...
linking...
car_control_stm32_project\car_control_stm32_project.axf: Error: L6218E: Undefined symbol ABS (referred from yuntai.o).
Not enough information to list image symbols.
Not enough information to list load addresses in the image map.
Finished: 2 information, 0 warning and 1 error messages.
"car_control_stm32_project\car_control_stm32_project.axf" - 1 Error(s), 2 Warning(s).
Target not created.