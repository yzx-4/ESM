#include "algorithm/current_loop/current_loop.h"

#include <math.h>

#include "config/foc_config.h"

static esm_pi_ctrl_t s_id_pi = {//D轴PI控制器的初始参数，比例系数0.2，积分系数40，积分值0，输出限制-6到6伏特
    .kp = 0.2f,
    .ki = 40.0f,
    .integrator = 0.0f,
    .out_min = -6.0f,
    .out_max = 6.0f,
};

static esm_pi_ctrl_t s_iq_pi = {//Q轴PI控制器的初始参数，比例系数0.2，积分系数40，积分值0，输出限制-6到6伏特
    .kp = 0.2f,
    .ki = 40.0f,
    .integrator = 0.0f,
    .out_min = -6.0f,
    .out_max = 6.0f,
};

static float esm_clampf(float x, float x_min, float x_max)//将x限制在x_min和x_max之间
{
    if (x < x_min) {
        return x_min;
    }
    if (x > x_max) {
        return x_max;
    }
    return x;
}

static float esm_pi_run(esm_pi_ctrl_t *pi, float ref, float meas, float dt_s)//运行PI控制器，输入参考值、测量值和采样周期，输出饱和后的控制信号
{
    float err;
    float out_unsat;
    float out_sat;

    if (pi == NULL || dt_s <= 0.0f) {
        return 0.0f;
    }

    err = ref - meas;
    pi->integrator += pi->ki * err * dt_s;
    pi->integrator = esm_clampf(pi->integrator, pi->out_min, pi->out_max);
    out_unsat = pi->kp * err + pi->integrator;
    out_sat = esm_clampf(out_unsat, pi->out_min, pi->out_max);

    if (out_unsat != out_sat) {
        pi->integrator = out_sat - pi->kp * err;
        pi->integrator = esm_clampf(pi->integrator, pi->out_min, pi->out_max);
    }
    return out_sat;
}

esp_err_t esm_algo_current_loop_init(void)
{
    const esm_foc_config_t *cfg = esm_cfg_foc_get();

    if (cfg == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_id_pi.kp = cfg->current_loop.id_kp;
    s_id_pi.ki = cfg->current_loop.id_ki;
    s_iq_pi.kp = cfg->current_loop.iq_kp;
    s_iq_pi.ki = cfg->current_loop.iq_ki;
    s_id_pi.integrator = 0.0f;
    s_iq_pi.integrator = 0.0f;
    return ESP_OK;
}

esp_err_t esm_algo_current_loop_run(const esm_current_loop_input_t *in, esm_current_loop_output_t *out)//运行当前环算法，输入ID和IQ轴的参考值和测量值、采样周期和电压限制等，输出D轴和Q轴的电压值
{
    float v_limit;

    if (in == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    v_limit = fabsf(in->v_limit);
    if (v_limit < 1.0f) {
        v_limit = 1.0f;
    }

    s_id_pi.out_min = -v_limit;
    s_id_pi.out_max = v_limit;
    s_iq_pi.out_min = -v_limit;
    s_iq_pi.out_max = v_limit;

    out->vd = esm_pi_run(&s_id_pi, in->id_ref, in->id_meas, in->dt_s);
    out->vq = esm_pi_run(&s_iq_pi, in->iq_ref, in->iq_meas, in->dt_s);
    return ESP_OK;
}

