#pragma once

#include "esp_err.h"

#include "bsp/bsp_hal.h"

esp_err_t esm_bsp_current_sense_init(void);
esp_err_t esm_bsp_current_sense_read_latest_sample(esm_bsp_phase_current_t *current,
												   uint32_t *conv_done_delta,
												   uint32_t *conv_done_total);
esp_err_t esm_bsp_current_sense_read_slow_raw(esm_bsp_analog_slow_channel_id_t slow_channel_id, uint16_t *raw);