#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"

#include "esp_err.h"

static inline TickType_t esm_task_foc_ms_to_ticks(uint32_t ms)
{
	return pdMS_TO_TICKS(ms);
}

esp_err_t esm_task_foc_ctrl_start(void);

