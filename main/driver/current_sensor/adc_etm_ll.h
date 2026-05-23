#pragma once

#include "esp_err.h"
#include "esp_etm.h"

esp_err_t esm_drv_adc_etm_new_start_task(esp_etm_task_handle_t *out_task);