/* Board-level static hardware mapping for PWM, ADC sensing, encoder SPI, and board peripherals. */
/* 引脚配置为esp32s3，工程链接： */

#include "bsp/board/board.h"

#include <stddef.h>

static const esm_bsp_pwm_cfg_t s_pwm_cfg_table[ESM_BSP_PWM_MAX_INSTANCE] = {
    {
        .timer_num = 0,
        .freq_hz = 10000,
        .phase_u_high_pin = 8,
        .phase_u_low_pin = 7,
        .phase_v_high_pin = 9,
        .phase_v_low_pin = 15,
        .phase_w_high_pin = 16,
        .phase_w_low_pin = 6,
        .deadtime_ns = 250,
    },
};

static const esm_bsp_current_sense_cfg_t s_current_sense_cfg_table[ESM_BSP_CURRENT_SENSE_MAX_INSTANCE] = {
    {
        .adc_unit = {1, 1, 1},
        .adc_channel = {4, 1, 0},
        .bus_voltage_adc_unit = 1,
        .bus_voltage_adc_channel = 3,
        .ntc_adc_unit = 1,
        .ntc_adc_channel = 2,
        .gain_v_per_a = 0.05f,
        .bus_div_ratio = 0.08f,
    },
};

static const esm_bsp_encoder_cfg_t s_encoder_cfg_table[ESM_BSP_ENCODER_MAX_INSTANCE] = {
    {
        .spi_host = 2,
        .mosi_pin = 10,
        .miso_pin = 12,
        .sclk_pin = 11,
        .cs_pin = 13,
        .clock_hz = 1000000,
        .spi_mode = 1,
    },
};

static const struct {
    int8_t tx_pin;
    int8_t rx_pin;
} s_twai_cfg = {
    .tx_pin = 47,
    .rx_pin = 48,
};

static const struct {
    int8_t gpio_pin;
} s_ws2812_cfg = {
    .gpio_pin = 35,
};

bsp_status_t esm_bsp_board_init(void)
{
    (void)s_twai_cfg;
    (void)s_ws2812_cfg;
    return BSP_OK;
}

const esm_bsp_pwm_cfg_t *esm_bsp_board_get_pwm_cfg(uint8_t instance_id)
{
    if (instance_id >= ESM_BSP_PWM_MAX_INSTANCE) {
        return NULL;
    }
    return &s_pwm_cfg_table[instance_id];
}

const esm_bsp_current_sense_cfg_t *esm_bsp_board_get_current_sense_cfg(uint8_t instance_id)
{
    if (instance_id >= ESM_BSP_CURRENT_SENSE_MAX_INSTANCE) {
        return NULL;
    }
    return &s_current_sense_cfg_table[instance_id];
}

const esm_bsp_encoder_cfg_t *esm_bsp_board_get_encoder_cfg(uint8_t instance_id)
{
    if (instance_id >= ESM_BSP_ENCODER_MAX_INSTANCE) {
        return NULL;
    }
    return &s_encoder_cfg_table[instance_id];
}