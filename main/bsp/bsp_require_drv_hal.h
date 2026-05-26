#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Driver contract header: types and esm_drv_* prototypes that drivers must implement.
 * This header is included by driver implementations and by BSP implementation files
 * that call into drivers. It is NOT intended for application-level includes.
 */

/* Use ESP-IDF MCPWM types for comparator handle etc. */
#include "driver/mcpwm_types.h"
/* Expose BSP-visible types used by driver contracts */
#include "bsp/bsp_hal.h"

/* Driver-visible MCPWM config (driver contract). BSP provides its own
 * `esm_bsp_pwm_cfg_t` in bsp_hal.h; here we define the driver-side config
 * that the driver implementation expects. BSP converts its config to this
 * before calling into the driver.
 */
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
} esm_drv_mcpwm_cfg_t;

typedef struct {
    uint32_t isr_count;
    uint32_t period_cb_count;
    uint32_t period_cb_drop_count;
} esm_drv_mcpwm_isr_stats_t;

typedef void (*esm_drv_mcpwm_period_cb_t)(void *user_ctx);
typedef void (*esm_drv_mcpwm_adc_trigger_cb_t)(void *user_ctx);

/* Driver API: MCPWM */
esp_err_t esm_drv_mcpwm_init(const esm_drv_mcpwm_cfg_t *cfg);
esp_err_t esm_drv_mcpwm_set_duty(uint8_t phase, float duty);
esp_err_t esm_drv_mcpwm_apply_duty(float duty_a, float duty_b, float duty_c);
esp_err_t esm_drv_mcpwm_enable(void);
esp_err_t esm_drv_mcpwm_disable(void);
esp_err_t esm_drv_mcpwm_set_period_callback(esm_drv_mcpwm_period_cb_t cb, void *user_ctx);
esp_err_t esm_drv_mcpwm_register_adc_center_trigger_callback(esm_drv_mcpwm_adc_trigger_cb_t cb, void *user_ctx);
void esm_drv_mcpwm_get_isr_stats(esm_drv_mcpwm_isr_stats_t *stats);
void esm_drv_mcpwm_reset_isr_stats(void);
esp_err_t esm_drv_mcpwm_get_comparator_handle(uint8_t phase, mcpwm_cmpr_handle_t *out_cmpr);

/* Current sensor (analog) driver contract */
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
} esm_drv_current_cfg_t;

typedef void (*esm_drv_analog_conv_cb_t)(void *user_ctx);

/* Driver-visible current sample type (driver-side copy to avoid circular includes) */
typedef struct {
    uint16_t raw_ia;
    uint16_t raw_ib;
    uint16_t raw_ic;
} esm_drv_phase_current_t;

typedef enum {
    ESM_DRV_ANALOG_SLOW_CH_BUS_V = 0,
    ESM_DRV_ANALOG_SLOW_CH_NTC = 1,
    ESM_DRV_ANALOG_SLOW_CH_RESERVED0 = 2,
    ESM_DRV_ANALOG_SLOW_CH_RESERVED1 = 3,
} esm_drv_analog_slow_channel_id_t;

esp_err_t esm_drv_analog_init(const esm_drv_current_cfg_t *cfg);
esp_err_t esm_drv_analog_read_latest_sample(esm_drv_phase_current_t *current);
esp_err_t esm_drv_analog_read_slow_raw(esm_drv_analog_slow_channel_id_t slow_channel_id, uint16_t *raw);
