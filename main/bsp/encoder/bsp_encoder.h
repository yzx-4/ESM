#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t esm_bsp_encoder_init(void);
esp_err_t esm_bsp_encoder_read_angle_rad(float *angle_rad);
esp_err_t esm_bsp_encoder_set_zero_offset_rad(float zero_offset_rad);
void esm_bsp_encoder_set_verbose(bool v);
