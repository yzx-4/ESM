#pragma once

#include <stdbool.h>

#include "esp_err.h"

#define ESM_MONITOR_TEST_MODE_NONE             0
#define ESM_MONITOR_TEST_MODE_PHASE_DUTY       1
#define ESM_MONITOR_TEST_MODE_OPEN_LOOP_VECTOR 2

/* Switch test mode here: NONE / PHASE_DUTY / OPEN_LOOP_VECTOR. */
#define ESM_MONITOR_TEST_MODE                  ESM_MONITOR_TEST_MODE_NONE 

esp_err_t esm_task_monitor_start(void);
bool esm_task_monitor_is_test_mode_enabled(void);

