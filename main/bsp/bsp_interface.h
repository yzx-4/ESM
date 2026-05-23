/* BSP abstraction: cfg/ops registration and instance lookup for hardware-facing modules. */
#pragma once

#include <stdint.h>

#include "bsp/bsp_status.h"

#define ESM_BSP_PWM_MAX_INSTANCE            2

#define ESM_BSP_CURRENT_SENSE_MAX_INSTANCE  2
#define ESM_BSP_ENCODER_MAX_INSTANCE        2
#define ESM_BSP_ANALOG_FAST_PHASE_COUNT     3
#define ESM_BSP_ANALOG_SLOW_MAX_CHANNEL     4

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

typedef struct {//drv层读电流使用的硬件，这里的fast指的是电流环，slow测一些慢变化的量
    uint8_t fast_phase_index[ESM_BSP_ANALOG_FAST_PHASE_COUNT];
    uint8_t fast_adc_unit[ESM_BSP_ANALOG_FAST_PHASE_COUNT];
    uint8_t fast_adc_channel[ESM_BSP_ANALOG_FAST_PHASE_COUNT];
    uint8_t etm_trigger_phase;
    uint8_t slow_adc_unit[ESM_BSP_ANALOG_SLOW_MAX_CHANNEL];
    uint8_t slow_adc_channel[ESM_BSP_ANALOG_SLOW_MAX_CHANNEL];
    uint8_t slow_channel_count;
    float bus_v_valid_min;
    float bus_v_valid_max;
    float bus_v_default;
} esm_bsp_current_sense_cfg_t;

typedef enum {
    ESM_BSP_ANALOG_SLOW_CH_BUS_V = 0,
    ESM_BSP_ANALOG_SLOW_CH_NTC = 1,
    ESM_BSP_ANALOG_SLOW_CH_RESERVED0 = 2,
    ESM_BSP_ANALOG_SLOW_CH_RESERVED1 = 3,
} esm_bsp_analog_slow_channel_id_t;

typedef struct {//三相电流原始采样值，留给上层统一换算成电流
    uint16_t raw_ia;
    uint16_t raw_ib;
    uint16_t raw_ic;
} esm_bsp_phase_current_t;

typedef struct {//drv层读电流使用的操作
    bsp_status_t (*init)(const esm_bsp_current_sense_cfg_t *cfg);
    bsp_status_t (*trigger_fast_sample)(esm_bsp_phase_current_t *current);//触发电流环非阻塞读取
    bsp_status_t (*read_slow_raw)(esm_bsp_analog_slow_channel_id_t slow_channel_id, uint16_t *raw);//频率不高的电压读取(可以是阻塞的)
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
