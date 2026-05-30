#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define ESM_BSP_PWM_MAX_INSTANCE            1
#define ESM_BSP_CURRENT_SENSE_MAX_INSTANCE  1
#define ESM_BSP_ENCODER_MAX_INSTANCE        1
#define ESM_BSP_ANALOG_FAST_PHASE_COUNT     3
#define ESM_BSP_ANALOG_SLOW_MAX_CHANNEL     4

typedef struct {
    uint8_t fast_phase_index[3];
    uint8_t fast_adc_unit[3];
    uint8_t fast_adc_channel[3];
    uint8_t etm_trigger_phase;
    uint8_t slow_adc_unit[4];
    uint8_t slow_adc_channel[4];
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

typedef struct {
    uint16_t raw_ia;
    uint16_t raw_ib;
    uint16_t raw_ic;
} esm_bsp_phase_current_t;//闭环计算采用原始数据，不转成电流（float）

esp_err_t esm_bsp_current_sense_init(void);
esp_err_t esm_bsp_current_sense_read_latest_sample(esm_bsp_phase_current_t *current,
                                                   uint32_t *conv_done_delta,
                                                   uint32_t *conv_done_total);//只是读取，采样转换由硬件完成
esp_err_t esm_bsp_current_sense_read_slow_raw(esm_bsp_analog_slow_channel_id_t slow_channel_id, uint16_t *raw);

typedef struct {
    uint8_t spi_host;
    int8_t mosi_pin;
    int8_t miso_pin;
    int8_t sclk_pin;
    int8_t cs_pin;
    uint32_t clock_hz;
    uint8_t spi_mode;
} esm_bsp_encoder_cfg_t;

esp_err_t esm_bsp_encoder_init(void);
esp_err_t esm_bsp_encoder_read_angle_rad(float *angle_rad);
esp_err_t esm_bsp_encoder_set_zero_offset_rad(float zero_offset_rad);
void esm_bsp_encoder_set_verbose(bool v);

typedef void (*esm_bsp_isr_callback_t)(void *user_ctx);

typedef struct {
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

typedef struct {
    uint32_t isr_count;
    uint32_t period_cb_count;
    uint32_t period_cb_drop_count;
} esm_bsp_pwm_isr_stats_t;

esp_err_t esm_bsp_pwm_init(void);
esp_err_t esm_bsp_pwm_set_duty(uint8_t phase, float duty);
esp_err_t esm_bsp_pwm_enable(void);
esp_err_t esm_bsp_pwm_disable(void);
esp_err_t esm_bsp_pwm_set_period_callback(esm_bsp_isr_callback_t cb, void *user_ctx);
void esm_bsp_pwm_get_isr_stats(esm_bsp_pwm_isr_stats_t *stats);
void esm_bsp_pwm_reset_isr_stats(void);
