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
        .phase_gain_v_per_a = 0.05f,
        .phase_offset_v = {1.65f, 1.65f, 1.65f},
        .bus_div_ratio = 0.08f,
    },
    .current_loop = {
        .id_kp = 0.2f,
        .id_ki = 40.0f,
        .iq_kp = 0.2f,
        .iq_ki = 40.0f,
        .current_limit_a = 5.0f,
        .id_ref_a = 0.0f,
        .iq_ref_a = 0.5f,
    },
    .control = {
        .pwm_hz = 10000,
        .current_loop_hz = 10000,
        .speed_loop_hz = 1000,
        .position_loop_hz = 100,
        .external_deadtime_ns = 250,
        .software_deadtime_ns = 0,
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

esp_err_t esm_cfg_foc_set_phase_offset_v(const float offset_v[3])
{
    if (offset_v == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(s_foc_cfg.current_sense.phase_offset_v, offset_v, sizeof(s_foc_cfg.current_sense.phase_offset_v));
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
