#pragma once

#include "bsp/bsp_hal.h"

// BSP-level API for MCPWM (thin wrapper over driver)
// These functions are implemented in bsp/mcpwm/bsp_mcpwm.c

esp_err_t esm_bsp_pwm_init(void);
esp_err_t esm_bsp_pwm_set_duty(uint8_t phase, float duty);
esp_err_t esm_bsp_pwm_enable(void);
esp_err_t esm_bsp_pwm_disable(void);
esp_err_t esm_bsp_pwm_set_period_callback(esm_bsp_isr_callback_t cb, void *user_ctx);
void esm_bsp_pwm_get_isr_stats(esm_bsp_pwm_isr_stats_t *stats);
void esm_bsp_pwm_reset_isr_stats(void);
