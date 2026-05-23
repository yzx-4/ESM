/* Board-level static hardware mapping for PWM, ADC sensing, encoder SPI, and board peripherals. */
/* 引脚配置为ESP32C5。 */

#include "bsp/board/board.h"

#include <stddef.h>
/*pwm参数，引脚*/
static const esm_bsp_pwm_cfg_t s_pwm_cfg_table[ESM_BSP_PWM_MAX_INSTANCE] = {
    {
        .timer_num = 0,
        .freq_hz = 10000,
        .phase_u_high_pin = 27,
        .phase_u_low_pin = 23,
        .phase_v_high_pin = 4,
        .phase_v_low_pin = 24,
        .phase_w_high_pin = 5,
        .phase_w_low_pin = 12,
        .deadtime_ns = 250,
    },
};
/**/
static const esm_bsp_current_sense_cfg_t s_current_sense_cfg_table[ESM_BSP_CURRENT_SENSE_MAX_INSTANCE] = {
    {
        .fast_phase_index = {0, 1, 0},
        .fast_adc_unit = {1, 1, 0},
        .fast_adc_channel = {5, 0, 0},
        .etm_trigger_phase = 0,
        .slow_channel_count = 0,
    },
};

static const esm_bsp_encoder_cfg_t s_encoder_cfg_table[ESM_BSP_ENCODER_MAX_INSTANCE] = {
    {
        .spi_host = 2,
        .mosi_pin = 7,
        .miso_pin = 8,
        .sclk_pin = 9,
        .cs_pin = 10,
        .clock_hz = 1000000,
        .spi_mode = 3,//工程实测as5047用mode3，可以读到10M，就是唯一正确的模式！！！
    },
};

static const struct {
    int8_t tx_pin;
    int8_t rx_pin;
} s_twai_cfg = {
    .tx_pin = 11,
    .rx_pin = 25,
};

static const struct {
    int8_t gpio_pin;
} s_ws2812_cfg = {
    .gpio_pin = 0,
};

bsp_status_t esm_bsp_board_init(void)
{
    (void)s_twai_cfg;
    (void)s_ws2812_cfg;
    return BSP_OK;
}

const esm_bsp_pwm_cfg_t *esm_bsp_board_get_pwm_cfg(uint8_t instance_id)//根据实例id返回对应的pwm配置结构体指针，如果id非法返回NULL
{
    if (instance_id >= ESM_BSP_PWM_MAX_INSTANCE) {
        return NULL;
    }
    return &s_pwm_cfg_table[instance_id];
}

const esm_bsp_current_sense_cfg_t *esm_bsp_board_get_current_sense_cfg(uint8_t instance_id)//根据实例id返回对应的电流传感配置结构体指针，如果id非法返回NULL
{
    if (instance_id >= ESM_BSP_CURRENT_SENSE_MAX_INSTANCE) {
        return NULL;
    }
    return &s_current_sense_cfg_table[instance_id];
}

const esm_bsp_encoder_cfg_t *esm_bsp_board_get_encoder_cfg(uint8_t instance_id)//根据实例id返回对应的编码器配置结构体指针，如果id非法返回NULL
{
    if (instance_id >= ESM_BSP_ENCODER_MAX_INSTANCE) {
        return NULL;
    }
    return &s_encoder_cfg_table[instance_id];
}