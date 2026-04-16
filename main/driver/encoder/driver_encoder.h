/* Encoder driver interface: absolute angle read and zero-offset setup. */
#pragma once

#include "esp_err.h"

esp_err_t esm_drv_encoder_init(void);
esp_err_t esm_drv_encoder_read_angle_rad(float *angle_rad);
esp_err_t esm_drv_encoder_set_zero_offset_rad(float zero_offset_rad);

