#include "bsp/bsp_hal.h"

#include "bsp/board/board.h"
#include "bsp/bsp_require_drv_hal.h"

esp_err_t esm_bsp_pwm_init(void)
{
    const esm_bsp_pwm_cfg_t *cfg = esm_bsp_board_get_pwm_cfg();
    esm_drv_mcpwm_cfg_t drv_cfg;

    if (cfg == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    drv_cfg.timer_num = cfg->timer_num;
    drv_cfg.freq_hz = cfg->freq_hz;
    drv_cfg.phase_u_high_pin = cfg->phase_u_high_pin;
    drv_cfg.phase_u_low_pin = cfg->phase_u_low_pin;
    drv_cfg.phase_v_high_pin = cfg->phase_v_high_pin;
    drv_cfg.phase_v_low_pin = cfg->phase_v_low_pin;
    drv_cfg.phase_w_high_pin = cfg->phase_w_high_pin;
    drv_cfg.phase_w_low_pin = cfg->phase_w_low_pin;
    drv_cfg.deadtime_ns = cfg->deadtime_ns;

    return esm_drv_mcpwm_init(&drv_cfg);
}

esp_err_t esm_bsp_pwm_set_duty(uint8_t phase, float duty)
{
    return esm_drv_mcpwm_set_duty(phase, duty);
}

esp_err_t esm_bsp_pwm_enable(void)
{
    return esm_drv_mcpwm_enable();
}

esp_err_t esm_bsp_pwm_disable(void)
{
    return esm_drv_mcpwm_disable();
}

esp_err_t esm_bsp_pwm_set_period_callback(esm_bsp_isr_callback_t cb, void *user_ctx)
{
    return esm_drv_mcpwm_set_period_callback(cb, user_ctx);
}

void esm_bsp_pwm_get_isr_stats(esm_bsp_pwm_isr_stats_t *stats)
{
    esm_drv_mcpwm_isr_stats_t drv_stats;

    if (stats == NULL) {
        return;
    }
    esm_drv_mcpwm_get_isr_stats(&drv_stats);
    stats->isr_count = drv_stats.isr_count;
    stats->period_cb_count = drv_stats.period_cb_count;
    stats->period_cb_drop_count = drv_stats.period_cb_drop_count;
}

void esm_bsp_pwm_reset_isr_stats(void)
{
    esm_drv_mcpwm_reset_isr_stats();
}
