#include "task_defer.h"

#include <stdint.h>
#include <stdio.h>

#include "log.h"

#define ABS(x) ((x) < 0 ? -(x) : (x))

int32_t find_unused_cell(TASK_DEFER* ctrl) {
  if (ctrl == NULL) {
    return -1;
  } else if (ctrl->defer_num >= MAX_DEFER_NUM) {
    return -1;
  } else {
    for (int i = 0; i < MAX_DEFER_NUM; i++) {
      if (ctrl->waiting[i] == false) {
        return i;
      }
    }
  }
}

bool add_defer(TASK_DEFER* ctrl, void (*task)(void* para), void* para, bool(*judge), void* cur_addr, float offset, TYPE type, uint32_t timeout) {
  int32_t pos = find_unused_cell(ctrl);
  if (pos == -1) {
    return false;
  } else {
    ctrl->cell[pos].task = task;
    ctrl->cell[pos].para = para;
    ctrl->cell[pos].judge = judge;
    ctrl->cell[pos].tar = get_cur_val(cur_addr, type) + offset;
    ctrl->cell[pos].cur_addr = cur_addr;
    ctrl->cell[pos].type = type;
    ctrl->cell[pos].timeout = ctrl->defer_clock + timeout;
    ctrl->waiting[pos] = true;
    ctrl->defer_num++;
    return true;
  }
}

bool is_timeout(TASK_DEFER* ctrl, defer_cell* cell) {
  if (cell->timeout <= 0) {
    return false;
  }
  if (ctrl->defer_clock >= cell->timeout) {
    return true;
  }
  return false;
}

void driver_defer(TASK_DEFER* ctrl) {
  if (ctrl == NULL) {
    return;
  } else if (ctrl->defer_num == 0) {
    return;
  }
  for (int i = 0; i < MAX_DEFER_NUM; i++) {
    if (ctrl->waiting[i] == true) {
      ctrl->cell[i].cur = get_cur_val(ctrl->cell[i].cur_addr, ctrl->cell[i].type);
      if (ctrl->cell[i].judge(ctrl->cell[i].cur, ctrl->cell[i].tar) || is_timeout(ctrl, &ctrl->cell[i])) {
        ctrl->cell[i].task(ctrl->cell[i].para);
        ctrl->waiting[i] = false;
        ctrl->defer_num--;
      }
    }
  }
}

void init_TASK_DEFER(TASK_DEFER* ctrl) {
  if (ctrl == NULL) {
    return;
  }
  ctrl->defer_num = 0;
  ctrl->defer_clock = 0;
  for (int i = 0; i < MAX_DEFER_NUM; i++) {
    ctrl->waiting[i] = false;
    ctrl->cell[i].task = NULL;
    ctrl->cell[i].para = NULL;
    ctrl->cell[i].judge = NULL;
    ctrl->cell[i].cur_addr = NULL;
    ctrl->cell[i].type = UNKNOWN;
    ctrl->cell[i].timeout = 0;
  }
}

void update_defer_clock(TASK_DEFER* ctrl, uint64_t system_time) {
  if (ctrl == NULL) {
    return;
  }
  ctrl->defer_clock = system_time;
}

bool Greater_or_Equal(float cur, float tar) {
  if (cur >= tar) {
    return true;
  } else {
    return false;
  }
}

bool Less_or_Equal(float cur, float tar) {
  if (cur <= tar) {
    return true;
  } else {
    return false;
  }
}

bool Error_less_1(float cur, float tar) {
  if (ABS(cur - tar) < 1.0f) {
    return true;
  } else {
    return false;
  }
}

bool Error_less_10(float cur, float tar) {
  if (ABS(cur - tar) < 10.0f) {
    return true;
  } else {
    return false;
  }
}

bool Error_less_100(float cur, float tar) {
  if (ABS(cur - tar) < 100.0f) {
    return true;
  } else {
    return false;
  }
}

bool Error_less_1000(float cur, float tar) {
  if (ABS(cur - tar) < 1000.0f) {
    return true;
  } else {
    return false;
  }
}

bool All_false(float cur, float tar) {
  return false;
}

uint32_t add_defer_with_time_line(TASK_DEFER* ctrl, void (*task)(void* para), void* para, uint32_t later) {
  add_defer(ctrl, task, para, All_false, NULL, 0, UINT8, later);
  return later;
}
