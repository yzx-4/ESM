#include "algorithm/svpwm/svpwm.h"

#define ESM_SQRT3_BY_2  0.86602540378f                     //sqrt(3)/2，用于坐标变换的常数

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

esp_err_t esm_algo_svpwm_init(void)                       //SVPWM算法的初始化函数，目前没有需要初始化的状态变量，所以直接返回成功
{
    return ESP_OK;
}

esp_err_t esm_algo_svpwm_generate(const esm_svpwm_input_t *in, esm_svpwm_output_t *out)//根据输入的电压矢量和总线电压，计算出三相的PWM占空比，输出到out结构体中
{
    float bus_v;         //总线电压，单位是伏特，从输入结构体中获取，如果输入的总线电压小于1伏特，则认为输入无效，返回错误
    float max_mod;       //最大调制比，单位是无量纲，从输入结构体中获取，并限制在0.1到1.0之间，如果输入的最大调制比不在这个范围内，则返回错误
    float phase_v_scale; //电压到占空比的转换比例，单位是无量纲，根据总线电压计算得到，公式是2除以总线电压，因为SVPWM的占空比范围是0.5±0.5*调制比，而电压范围是±总线电压的1/sqrt(3)倍，所以需要乘以sqrt(3)来得到正确的比例
    float va;            //A相的电压矢量分量，单位是伏特，从输入结构体中获取
    float vb;
    float vc;
    float duty_min;      //占空比的最小值，单位是无量纲，根据最大调制比计算得到，公式是0.5减去0.5乘以最大调制比，因为SVPWM的占空比范围是0.5±0.5*调制比，所以最小值就是0.5-0.5*调制比
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

