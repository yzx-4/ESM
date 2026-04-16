/* Current sensor driver: phase-current calibration and converted current output. */
#pragma once

#include "esp_err.h"

typedef struct {
	float ia;
	float ib;
	float ic;
} esm_phase_current_t;

esp_err_t esm_drv_current_sensor_init(void);
esp_err_t esm_drv_current_sensor_calibrate_zero(void);
esp_err_t esm_drv_current_sensor_get_phase_current(esm_phase_current_t *current);

