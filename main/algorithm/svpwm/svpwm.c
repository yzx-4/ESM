#include "algorithm/svpwm/svpwm.h"

#define ESM_SQRT3_BY_2  0.86602540378f

static float esm_clampf(float x, float x_min, float x_max)
{
    if (x < x_min) {
        return x_min;
    }
    if (x > x_max) {
        return x_max;
    }
    return x;
}

esp_err_t esm_algo_svpwm_init(void)
{
    return ESP_OK;
}

esp_err_t esm_algo_svpwm_generate(const esm_svpwm_input_t *in, esm_svpwm_output_t *out)
{
    float bus_v;
    float max_mod;
    float phase_v_scale;
    float va;
    float vb;
    float vc;
    float duty_min;
    float duty_max;

    if (in == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bus_v = in->bus_v;
    if (bus_v < 1.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    max_mod = esm_clampf(in->max_modulation, 0.1f, 1.0f);
    phase_v_scale = 2.0f / bus_v;

    va = in->v_alpha;
    vb = -0.5f * in->v_alpha + ESM_SQRT3_BY_2 * in->v_beta;
    vc = -0.5f * in->v_alpha - ESM_SQRT3_BY_2 * in->v_beta;

    out->duty_a = 0.5f + va * phase_v_scale;
    out->duty_b = 0.5f + vb * phase_v_scale;
    out->duty_c = 0.5f + vc * phase_v_scale;

    duty_min = 0.5f - 0.5f * max_mod;
    duty_max = 0.5f + 0.5f * max_mod;
    out->duty_a = esm_clampf(out->duty_a, duty_min, duty_max);
    out->duty_b = esm_clampf(out->duty_b, duty_min, duty_max);
    out->duty_c = esm_clampf(out->duty_c, duty_min, duty_max);
    return ESP_OK;
}

