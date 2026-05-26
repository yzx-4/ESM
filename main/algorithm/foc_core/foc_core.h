#pragma once

#include <stdint.h>

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

typedef esp_err_t (*esm_algo_foc_pwm_apply_fn_t)(void *user_ctx, const esm_foc_pwm_cmd_t *cmd);
typedef esp_err_t (*esm_algo_foc_encoder_read_fn_t)(void *user_ctx, float *angle_rad);
typedef esp_err_t (*esm_algo_foc_encoder_set_zero_fn_t)(void *user_ctx, float zero_offset_rad);
typedef void (*esm_algo_foc_delay_ms_fn_t)(void *user_ctx, uint32_t ms);

typedef struct {
	float align_duty_a;
	float align_duty_b;
	float align_duty_c;
	float zero_duty_a;
	float zero_duty_b;
	float zero_duty_c;
	uint32_t hold_ms;
	uint32_t settle_ms;
	uint32_t retry_count;
	uint32_t retry_delay_ms;
} esm_algo_foc_encoder_align_cfg_t;

esp_err_t esm_algo_foc_core_init(void);
esp_err_t esm_algo_foc_set_current_ref(float id_ref_a, float iq_ref_a);
esp_err_t esm_algo_foc_mech_to_elec_angle(float mech_angle_rad, uint8_t pole_pairs, float *elec_angle_rad);
esp_err_t esm_algo_foc_apply_pwm_cmd(const esm_foc_pwm_cmd_t *cmd, esm_algo_foc_pwm_apply_fn_t apply_fn, void *user_ctx);
esp_err_t esm_algo_foc_align_encoder_zero(const esm_algo_foc_encoder_align_cfg_t *cfg,
	esm_algo_foc_pwm_apply_fn_t pwm_apply_fn,
	void *pwm_apply_ctx,
	esm_algo_foc_encoder_read_fn_t encoder_read_fn,
	void *encoder_read_ctx,
	esm_algo_foc_encoder_set_zero_fn_t encoder_set_zero_fn,
	void *encoder_set_zero_ctx,
	esm_algo_foc_delay_ms_fn_t delay_fn,
	void *delay_ctx);
esp_err_t esm_algo_foc_run_10khz(const esm_foc_sample_t *sample, esm_foc_pwm_cmd_t *cmd);

