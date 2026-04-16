/* BSP abstraction: cfg/ops registration and instance lookup for hardware-facing modules. */
#pragma once

#include <stdint.h>

#include "bsp/bsp_status.h"

#define ESM_BSP_PWM_MAX_INSTANCE            2
#define ESM_BSP_CURRENT_SENSE_MAX_INSTANCE  2
#define ESM_BSP_ENCODER_MAX_INSTANCE        2

typedef void (*esm_bsp_isr_callback_t)(void *user_ctx);

typedef struct {//drv层pwm使用的硬件
    uint32_t timer_num;
    uint32_t freq_hz;
    int8_t phase_u_high_pin;
    int8_t phase_u_low_pin;
    int8_t phase_v_high_pin;
    int8_t phase_v_low_pin;
    int8_t phase_w_high_pin;
    int8_t phase_w_low_pin;
    uint32_t deadtime_ns;
} esm_bsp_pwm_cfg_t;

typedef struct {//drv层pwm使用的操作
    bsp_status_t (*init)(const esm_bsp_pwm_cfg_t *cfg);
    bsp_status_t (*set_duty)(uint8_t phase, float duty);
    bsp_status_t (*enable)(void);
    bsp_status_t (*disable)(void);
    bsp_status_t (*set_period_callback)(esm_bsp_isr_callback_t cb, void *user_ctx);
} esm_bsp_pwm_ops_t;

typedef struct {//drv层读电流使用的硬件
    uint8_t adc_unit[3];
    uint8_t adc_channel[3];
    uint8_t bus_voltage_adc_unit;
    uint8_t bus_voltage_adc_channel;
    uint8_t ntc_adc_unit;
    uint8_t ntc_adc_channel;
    float gain_v_per_a;
    float bus_div_ratio;
} esm_bsp_current_sense_cfg_t;

typedef struct {//三相电流数组
    float ia;
    float ib;
    float ic;
} esm_bsp_phase_current_t;

typedef struct {//drv层读电流使用的操作
    bsp_status_t (*init)(const esm_bsp_current_sense_cfg_t *cfg);
    bsp_status_t (*read_phase_voltage)(float phase_v[3]);
    bsp_status_t (*calibrate_zero)(void);
    bsp_status_t (*read_phase_current)(esm_bsp_phase_current_t *current);
    bsp_status_t (*read_bus_voltage)(float *bus_v);
} esm_bsp_current_sense_ops_t;

typedef struct {//drv层读编码器使用的硬件
    uint8_t spi_host;
    int8_t mosi_pin;
    int8_t miso_pin;
    int8_t sclk_pin;
    int8_t cs_pin;
    uint32_t clock_hz;
    uint8_t spi_mode;
} esm_bsp_encoder_cfg_t;

typedef struct {//drv层读编码器使用的操作
    bsp_status_t (*init)(const esm_bsp_encoder_cfg_t *cfg);//初始化函数，参数是配置结构体指针
    bsp_status_t (*read_angle_rad)(float *angle_rad);  //读取角度函数，参数是输出的角度值指针，单位是弧度，范围是0~2PI
    bsp_status_t (*set_zero_offset_rad)(float zero_offset_rad);//设置零点偏移函数，参数是零点偏移值，单位是弧度，范围是0~2PI
} esm_bsp_encoder_ops_t;
// BSP接口：提供PWM、读电流、读编码器等硬件相关模块的配置和操作函数的注册和查询。
bsp_status_t esm_bsp_pwm_register(uint8_t instance_id, const esm_bsp_pwm_ops_t *ops, const esm_bsp_pwm_cfg_t *cfg);
const esm_bsp_pwm_ops_t *esm_bsp_pwm_get_ops(uint8_t instance_id);
const esm_bsp_pwm_cfg_t *esm_bsp_pwm_get_cfg(uint8_t instance_id);

bsp_status_t esm_bsp_current_sense_register(uint8_t instance_id, const esm_bsp_current_sense_ops_t *ops, const esm_bsp_current_sense_cfg_t *cfg);
const esm_bsp_current_sense_ops_t *esm_bsp_current_sense_get_ops(uint8_t instance_id);
const esm_bsp_current_sense_cfg_t *esm_bsp_current_sense_get_cfg(uint8_t instance_id);

bsp_status_t esm_bsp_encoder_register(uint8_t instance_id, const esm_bsp_encoder_ops_t *ops, const esm_bsp_encoder_cfg_t *cfg);
const esm_bsp_encoder_ops_t *esm_bsp_encoder_get_ops(uint8_t instance_id);
const esm_bsp_encoder_cfg_t *esm_bsp_encoder_get_cfg(uint8_t instance_id);
