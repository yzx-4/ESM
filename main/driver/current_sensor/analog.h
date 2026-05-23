/* Analog driver: phase-current calibration and raw current output. */
#pragma once

#include "esp_err.h"

esp_err_t esm_drv_analog_init(void);

/* Optional: setup ETM-based trigger for ADC sampling. If the platform/IDF
	provides esp_etm or equivalent, implement this to connect MCPWM events to
	ADC tasks. Returns ESP_OK if set (or no-op), ESP_ERR_NOT_SUPPORTED if not
	available. */
esp_err_t esm_drv_analog_setup_etm_trigger(void);
