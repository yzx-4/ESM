#pragma once

#include "esp_err.h"

typedef struct {
	float v_alpha;
	float v_beta;
	float bus_v;
	float max_modulation;
} esm_svpwm_input_t;

typedef struct {
	float duty_a;
	float duty_b;
	float duty_c;
} esm_svpwm_output_t;

esp_err_t esm_algo_svpwm_init(void);
esp_err_t esm_algo_svpwm_generate(const esm_svpwm_input_t *in, esm_svpwm_output_t *out);

