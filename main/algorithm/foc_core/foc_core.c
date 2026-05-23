#include "algorithm/foc_core/foc_core.h"

#include <math.h>

#include "algorithm/current_loop/current_loop.h"
#include "algorithm/svpwm/svpwm.h"
#include "config/foc_config.h"

#define ESM_ONE_BY_SQRT3  0.57735026919f    // 1/sqrt(3)，用于坐标变换的常数

esp_err_t esm_algo_foc_core_init(void)
{
    return esm_algo_current_loop_init();
}

static void esm_clarke_transform(float ia, float ib, float *i_alpha, float *i_beta)
{
    if (i_alpha == NULL || i_beta == NULL) {
        return;
    }
    *i_alpha = ia;
    *i_beta = (ia + 2.0f * ib) * ESM_ONE_BY_SQRT3;
}

static float esm_sample_raw_to_amp(uint16_t raw, const esm_foc_config_t *cfg)
{
    float gain = 0.15f;
    float delta = (float)raw;

    if (cfg != NULL) {
        if (cfg->current_sense.phase_gain_v_per_a > 1e-6f) {
            gain = cfg->current_sense.phase_gain_v_per_a;
        }
        delta -= (float)cfg->current_sense.phase_offset_raw;
    }
    return (delta * 3.3f / 4095.0f) / gain;
}

static void esm_park_transform(float i_alpha, float i_beta, float theta, float *id, float *iq)
{
    float c = cosf(theta);
    float s = sinf(theta);

    if (id == NULL || iq == NULL) {
        return;
    }
    *id = i_alpha * c + i_beta * s;
    *iq = -i_alpha * s + i_beta * c;
}

static void esm_inv_park_transform(float vd, float vq, float theta, float *v_alpha, float *v_beta)
{
    float c = cosf(theta);
    float s = sinf(theta);

    if (v_alpha == NULL || v_beta == NULL) {
        return;
    }
    *v_alpha = vd * c - vq * s;
    *v_beta = vd * s + vq * c;
}

static float esm_normalize_angle(float angle_rad)  //将角度归一化到0~2PI范围
{
    const float two_pi = 6.28318530718f;
    while (angle_rad >= two_pi) {
        angle_rad -= two_pi;
    }
    while (angle_rad < 0.0f) {
        angle_rad += two_pi;
    }
    return angle_rad;
}

esp_err_t esm_algo_foc_set_current_ref(float id_ref_a, float iq_ref_a)//设置FOC算法的电流参考值，单位是安培
{
    esm_current_loop_params_t params = {0};
    const esm_foc_config_t *cfg = esm_cfg_foc_get();

    if (cfg == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    params = cfg->current_loop;
    params.id_ref_a = id_ref_a;
    params.iq_ref_a = iq_ref_a;
    (void)esm_cfg_foc_set_current_loop_params(&params);
    return ESP_OK;
}

esp_err_t esm_algo_foc_run_10khz(const esm_foc_sample_t *sample, esm_foc_pwm_cmd_t *cmd)//运行FOC算法，输入采样数据，输出PWM命令
{
    const esm_foc_config_t *cfg = esm_cfg_foc_get();
    float i_alpha = 0.0f;
    float i_beta = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;
    float v_alpha = 0.0f;
    float v_beta = 0.0f;
    float bus_v = 24.0f;
    float dt_s;        //采样周期，单位是秒，根据配置的当前环频率计算得到
    float v_limit;     //电压限制，单位是伏特，根据总线电压和最大调制比计算得到
    float theta;       //电角度，单位是弧度，根据机械角和极对数计算得到，结果范围是0~2PI
    esm_current_loop_input_t loop_in;
    esm_current_loop_output_t loop_out;
    esm_svpwm_input_t svpwm_in;
    esm_svpwm_output_t svpwm_out;

    if (sample == NULL || cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    dt_s = 1.0f / (float)cfg->control.current_loop_hz;
    if (cfg->control.current_loop_hz == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    theta = esm_normalize_angle(sample->elec_angle_rad);
    bus_v = sample->bus_v > 1.0f ? sample->bus_v : bus_v;
    v_limit = bus_v * ESM_ONE_BY_SQRT3 * cfg->control.max_modulation;

    const float ia = esm_sample_raw_to_amp(sample->raw_ia, cfg);
    const float ib = esm_sample_raw_to_amp(sample->raw_ib, cfg);
    esm_clarke_transform(ia, ib, &i_alpha, &i_beta);
    esm_park_transform(i_alpha, i_beta, theta, &id, &iq);

    loop_in.id_ref = cfg->current_loop.id_ref_a;
    loop_in.iq_ref = cfg->current_loop.iq_ref_a;
    if (cfg->current_loop.current_limit_a > 0.0f) {//如果配置了电流限制，就对参考值进行限制，确保不会超过设定的电流限制
        if (loop_in.id_ref > cfg->current_loop.current_limit_a) {
            loop_in.id_ref = cfg->current_loop.current_limit_a;
        }
        if (loop_in.id_ref < -cfg->current_loop.current_limit_a) {
            loop_in.id_ref = -cfg->current_loop.current_limit_a;
        }
        if (loop_in.iq_ref > cfg->current_loop.current_limit_a) {
            loop_in.iq_ref = cfg->current_loop.current_limit_a;
        }
        if (loop_in.iq_ref < -cfg->current_loop.current_limit_a) {
            loop_in.iq_ref = -cfg->current_loop.current_limit_a;
        }
    }
    loop_in.id_meas = id;
    loop_in.iq_meas = iq;
    loop_in.dt_s = dt_s;
    loop_in.v_limit = v_limit;
    if (esm_algo_current_loop_run(&loop_in, &loop_out) != ESP_OK) {
        return ESP_FAIL;
    }

    esm_inv_park_transform(loop_out.vd, loop_out.vq, theta, &v_alpha, &v_beta);

    svpwm_in.v_alpha = v_alpha;
    svpwm_in.v_beta = v_beta;
    svpwm_in.bus_v = bus_v;
    svpwm_in.max_modulation = cfg->control.max_modulation;
    if (esm_algo_svpwm_generate(&svpwm_in, &svpwm_out) != ESP_OK) {
        return ESP_FAIL;
    }

    cmd->duty_a = svpwm_out.duty_a;//将SVPWM算法生成的占空比返回到PWM命令结构体中，供FOC控制任务使用
    cmd->duty_b = svpwm_out.duty_b;
    cmd->duty_c = svpwm_out.duty_c;
    return ESP_OK;
}

