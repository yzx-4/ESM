#pragma once

#include <stdbool.h>

#include "esp_err.h"

#define ESM_MONITOR_TEST_MODE_NONE             0
#define ESM_MONITOR_TEST_MODE_PHASE_DUTY       1
#define ESM_MONITOR_TEST_MODE_OPEN_LOOP_VECTOR 2

/* Switch test mode here: NONE / PHASE_DUTY / OPEN_LOOP_VECTOR. */
#ifndef ESM_MONITOR_TEST_MODE
#define ESM_MONITOR_TEST_MODE                  ESM_MONITOR_TEST_MODE_NONE 
#endif
esp_err_t esm_task_monitor_start(void);
bool esm_task_monitor_is_test_mode_enabled(void);//提供给foc_ctrl任务查询监视器是否处于测试模式的函数，FOC任务可以根据这个函数的返回值决定是否跳过某些操作或者调整行为，以适应测试环境

