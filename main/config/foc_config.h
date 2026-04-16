#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef struct {//电机参数包含极对数、定子电阻和d轴、q轴电感等，用于FOC算法的坐标变换和控制计算
    uint8_t pole_pairs;
    float rs_ohm;
    float ld_h;
    float lq_h;
} esm_motor_params_t;

typedef struct {//电流传感器参数包含增益、偏置和总线电压分压比等，用于将ADC读数转换成实际的电流和电压值
    float phase_gain_v_per_a;
    float phase_offset_v[3];
    float bus_div_ratio;
} esm_current_sense_params_t;

typedef struct {//FOC算法的配置参数，包含电机参数、电流传感器参数、当前环参数和控制参数
    float id_kp;
    float id_ki;
    float iq_kp;
    float iq_ki;
    float current_limit_a;
    float id_ref_a;
    float iq_ref_a;
} esm_current_loop_params_t;

typedef struct {//FOC控制参数，包含PWM频率、控制环频率、死区时间和最大调制比等，用于控制PWM输出和算法的运行
    uint32_t pwm_hz;
    uint32_t current_loop_hz;
    uint32_t speed_loop_hz;
    uint32_t position_loop_hz;
    uint32_t external_deadtime_ns;
    uint32_t software_deadtime_ns;
    float max_modulation;
} esm_control_params_t;

typedef struct {
    esm_motor_params_t motor;
    esm_current_sense_params_t current_sense;
    esm_current_loop_params_t current_loop;
    esm_control_params_t control;
} esm_foc_config_t;

const esm_foc_config_t *esm_cfg_foc_get(void);
esp_err_t esm_cfg_foc_set_motor_params(const esm_motor_params_t *params);
esp_err_t esm_cfg_foc_set_phase_offset_v(const float offset_v[3]);
esp_err_t esm_cfg_foc_set_current_loop_params(const esm_current_loop_params_t *params);
