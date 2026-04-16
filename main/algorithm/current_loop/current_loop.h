#pragma once

#include "esp_err.h"

typedef struct {//PI控制器结构体，包含比例系数、积分系数、积分值和输出限制等
	float kp;
	float ki;
	float integrator;
	float out_min;
	float out_max;
} esm_pi_ctrl_t;

typedef struct {//当前环配置结构体，包含ID和IQ轴的PI控制器参数和电流限制等
	float id_kp;
	float id_ki;
	float iq_kp;
	float iq_ki;
	float current_limit_a;
} esm_current_loop_cfg_t;

typedef struct {//当前环输入结构体，包含ID和IQ轴的参考值和测量值、采样周期和电压限制等

	float id_ref;
	float iq_ref;
	float id_meas;
	float iq_meas;
	float dt_s;
	float v_limit;
} esm_current_loop_input_t;

typedef struct {//当前环输出结构体，包含D轴和Q轴的电压值
	float vd;
	float vq;
} esm_current_loop_output_t;

esp_err_t esm_algo_current_loop_init(void);
esp_err_t esm_algo_current_loop_run(const esm_current_loop_input_t *in, esm_current_loop_output_t *out);

