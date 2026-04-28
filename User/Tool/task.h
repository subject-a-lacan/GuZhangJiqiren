#ifndef __TASK_H
#define __TASK_H

#include "main.h" // HAL 库环境下包含 main.h 即可使用 HAL_GetTick()

// --- 宏 1：函数门卫模式 ---
// 如果时间未到，直接 return（跳出当前函数）
#define PERIODIC(T) \
    static uint32_t nxt = 0; \
    if(HAL_GetTick() < nxt) return; \
    nxt += (T);

// --- 宏 2：代码块包裹模式 (开始) ---
// NAME 需要唯一，用于生成独立的计时变量
#define PERIODIC_START(NAME, T) \
    static uint32_t NAME##_nxt = 0; \
    if(HAL_GetTick() >= NAME##_nxt) { \
        NAME##_nxt += (T);

// --- 宏 3：代码块包裹模式 (结束) ---
#define PERIODIC_END }

#endif
