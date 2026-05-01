等到架构完成 我必亲手杀死旧时代的残党
# 旧代码待清理清单

> 以下逻辑与 TASK 架构不兼容，后续由新 task_xxx_update() 替代。

## road.c

| 函数/变量 | 问题 |
|---|---|
| `cross_cnt` 全局变量 | 应归属 `status.task.cross_cnt`，旧逻辑直接读写全局 |
| `left_cnt` 全局变量 | LeftRoad 特化计数器，旧逻辑用第一个 LeftRoad 做特殊处理，与任务无关 |
| `cross_delay` 全局变量 | `cross_cnt==3` 时硬编码 55 帧（1.1s）延迟抑制路型检测。旧赛道需要，我们的正方形赛道不需要 |
| `serve_road()` | 混在一起：CrossRoad 清零 base_speed、LeftRoad 自增 left_cnt、cross_cnt==4 时改 PID 甚至写到 Straight 分支里重 init PID。全是为旧赛道硬编码 |
| `get_road_type()` | 用全局 `cross_cnt` 判定，非 `status.task.cross_cnt`；`cross_delay` 耦合 |

## status.c

| 函数/变量 | 问题 |
|---|---|
| `follow_line()` | LeftRoad→开环 pivot(20/-20)，应改为陀螺仪闭环转弯；CrossRoad+cross_cnt==4→右转，旧赛道逻辑 |
| `Turn_or_Straight()` | 用全局 `road_buf` 缓存，路口变化时先停车，等轮速降到 5 再切。不适合新架构 |
| `road_buf` 全局变量 | 旧 state 应在 TASK 内管理 |
| `cross_delay` 全局变量 | 同上 |

## 已废弃但未删除

- `timer_it.c` 里注释掉的 time-axis 硬编码写法（基于 ms 偏移的时间轴）

---

# TASK 架构进展

## 已完成

- [x] `Defect.h` — TASK 结构体、TASK_ID / START_POSE / Q1_RACE_PHASE 枚举
- [x] `Defect.c` — `init_task()`、`task_start()`
- [x] `status.h` — STATUS 新增 `TASK task` 成员
- [x] `status.c` — `init_status()` 末尾调用 `init_task()`
- [x] `task_start()` 按 jiegou.md §12 清零全部任务状态

## 下一步

- [ ] `update_task()` 调度入口，挂入 `update_status()`
- [ ] `task_finish()` / `task_abort()`
- [ ] `task_basic_1_update()` — Q1 状态机（Q1_START_A_TURN → ... → Q1_BA_FINAL）
- [ ] 陀螺仪闭环转弯（替换 follow_line 的 LeftRoad 开环 pivot）
- [ ] 编码器里程（phase_mileage 累加 + 停车触发）
- [ ] 按键/蓝牙对接 request 机制
- [ ] 杀死上述旧代码
