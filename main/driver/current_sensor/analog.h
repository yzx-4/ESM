/* Analog driver: phase-current calibration and raw current output. */
#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "bsp/bsp_require_drv_hal.h"

/* Driver API (implementation of current-sensing): */
esp_err_t esm_drv_analog_init(const esm_drv_current_cfg_t *cfg);
esp_err_t esm_drv_analog_read_latest_sample(esm_drv_phase_current_t *current,
										   uint32_t *conv_done_delta,
										   uint32_t *conv_done_total);
esp_err_t esm_drv_analog_read_slow_raw(esm_drv_analog_slow_channel_id_t slow_channel_id, uint16_t *raw);
