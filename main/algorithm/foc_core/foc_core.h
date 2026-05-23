#pragma once

#include "esp_err.h"

typedef struct {
	uint16_t raw_ia;
	uint16_t raw_ib;
	uint16_t raw_ic;
	float bus_v;
	float elec_angle_rad;
} esm_foc_sample_t;

typedef struct {
	float duty_a;
	float duty_b;
	float duty_c;
} esm_foc_pwm_cmd_t;

esp_err_t esm_algo_foc_core_init(void);
esp_err_t esm_algo_foc_set_current_ref(float id_ref_a, float iq_ref_a);
esp_err_t esm_algo_foc_run_10khz(const esm_foc_sample_t *sample, esm_foc_pwm_cmd_t *cmd);

