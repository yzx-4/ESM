#include "config/foc_config.h"

#include <string.h>

static esm_foc_config_t s_foc_cfg = {
    .motor = {
        .pole_pairs = 7,
        .rs_ohm = 0.0f,
        .ld_h = 0.0f,
        .lq_h = 0.0f,
    },
    .current_sense = {
        .phase_gain_v_per_a = 0.15f,
        .phase_offset_raw = 2048U,
        .enable_etm_trigger = true,
        .etm_trigger_phase = 0U,
        .use_adc_continuous = true,
        .use_adc_etm_ll = false,
    },
    .current_loop = {
        .id_kp = 0.2f,
        .id_ki = 40.0f,
        .iq_kp = 0.2f,
        .iq_ki = 40.0f,
        .current_limit_a = 0.01f,
        .id_ref_a = 0.00f,
        .iq_ref_a = 0.01f,
    },
    .control = {
        .pwm_hz = 40000,
        .current_loop_hz = 10000,
        .current_loop_notify_divider = 4,
        .speed_loop_hz = 1000,
        .position_loop_hz = 100,
        .external_deadtime_ns = 250,
        .software_deadtime_ns = 0,     //本项目由外部硬件死区250ns，软件死区设置为0
        .max_modulation = 0.9f,
    },
};

const esm_foc_config_t *esm_cfg_foc_get(void)
{
    return &s_foc_cfg;
}

esp_err_t esm_cfg_foc_set_motor_params(const esm_motor_params_t *params)
{
    if (params == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_foc_cfg.motor = *params;
    return ESP_OK;
}

esp_err_t esm_cfg_foc_set_phase_offset_raw(uint16_t offset_raw)
{
    s_foc_cfg.current_sense.phase_offset_raw = offset_raw;
    return ESP_OK;
}

esp_err_t esm_cfg_foc_set_etm_trigger(bool enable, uint8_t phase)
{
    if (phase >= 3U) {
        return ESP_ERR_INVALID_ARG;
    }
    s_foc_cfg.current_sense.enable_etm_trigger = enable;
    s_foc_cfg.current_sense.etm_trigger_phase = phase;
    return ESP_OK;
}

esp_err_t esm_cfg_foc_set_adc_options(bool use_continuous, bool use_etm_ll)
{
    s_foc_cfg.current_sense.use_adc_continuous = use_continuous;
    s_foc_cfg.current_sense.use_adc_etm_ll = use_etm_ll;
    return ESP_OK;
}

esp_err_t esm_cfg_foc_set_current_loop_params(const esm_current_loop_params_t *params)
{
    if (params == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_foc_cfg.current_loop = *params;
    return ESP_OK;
}
