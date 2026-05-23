/* MCPWM implementation: timer/operator setup, duty update, ISR callback and stats. */
#include "driver/mcpwm/driver_mcpwm.h"

#include <stdbool.h>

#include "bsp/board/board.h"
#include "bsp/bsp_interface.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "esp_check.h"

#define ESM_PWM_INSTANCE  0

typedef struct {//每相的PWM设备结构体，包含一个操作器、一个比较器和两个发生器（高电平和低电平），用于控制每相的PWM输出
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmpr;
    mcpwm_gen_handle_t gen_high;
    mcpwm_gen_handle_t gen_low;
} esm_pwm_phase_dev_t;

static esm_drv_mcpwm_period_cb_t s_period_cb = NULL;  //FOC控制任务在定时器周期事件回调中被通知运行一次，s_period_cb就是保存这个回调函数的指针，初始值为NULL表示没有注册回调  
static void *s_period_cb_ctx = NULL;                  //回调函数的用户上下文指针，FOC控制任务在注册回调时可以传入一个指针，这个指针会在回调被调用时传回给FOC控制任务，供其使用
static esm_drv_mcpwm_adc_trigger_cb_t s_adc_trigger_cb = NULL;//ADC触发回调函数指针，FOC控制任务在PWM周期事件中被通知需要进行一次ADC采样时，s_adc_trigger_cb就是保存这个回调函数的指针，初始值为NULL表示没有注册回调
static void *s_adc_trigger_ctx = NULL;                //
static bool s_pwm_ready = false;                      //标志位，表示PWM已经初始化完成，可以开始接受占空比更新，如果FOC控制任务在PWM准备好之前就调用了更新占空比的函数，就会返回错误
static uint32_t s_peak_ticks = 0;                     //半周期计数值，初始化后固定，用于占空比映射和比较器初值
static volatile esm_drv_mcpwm_isr_stats_t s_isr_stats;//ISR统计数据结构体，包含定时器周期事件的总调用次数、成功调用回调的次数和丢弃回调的次数等，用于监控ISR的执行情况和性能指标

static mcpwm_timer_handle_t s_timer = NULL;           //MCPWM定时器句柄，代表一个定时器实例，用于生成PWM周期事件和触发ISR回调
static esm_pwm_phase_dev_t s_phase_dev[3];            //三相PWM设备结构体数组，每个元素包含一个操作器、一个比较器和两个发生器（高电平和低电平），用于控制每相的PWM输出
/*将输入的占空比限制在0~1范围内*/
static float esm_clamp01(float x)
{
    if (x < 0.0f) {
        return 0.0f;
    }
    if (x > 1.0f) {
        return 1.0f;
    }
    return x;
}

static void esm_mcpwm_on_timer_common(bool trigger_adc)//定时器事件的公共处理函数，参数trigger_adc表示是否需要触发ADC采样，FOC控制任务在这个函数被调用时会知道一个新的PWM周期开始了，可以进行一次FOC计算和占空比更新了，同时如果trigger_adc为true还会通知FOC控制任务进行一次ADC采样
{
    s_isr_stats.isr_count++;
    if (s_period_cb != NULL) {
        s_isr_stats.period_cb_count++;
        s_period_cb(s_period_cb_ctx);
    } else {
        s_isr_stats.period_cb_drop_count++;
    }

    if (trigger_adc && s_adc_trigger_cb != NULL) {
        s_adc_trigger_cb(s_adc_trigger_ctx);
    }
}

static bsp_status_t esm_mcpwm_apply_phase_duty(uint8_t phase, float duty)//更新影子占空比
{
    float duty_clamped = esm_clamp01(duty);
    uint32_t cmp_ticks;//比较器计数值，用duty算一下

    if (!s_pwm_ready) {
        return BSP_ERR_FAIL;
    }
    if (phase >= 3U) {
        return BSP_ERR_INVALID_ARG;
    }

    cmp_ticks = (uint32_t)(duty_clamped * (float)s_peak_ticks);
    if (cmp_ticks > s_peak_ticks) {
        cmp_ticks = s_peak_ticks;
    }
    if (mcpwm_comparator_set_compare_value(s_phase_dev[phase].cmpr, cmp_ticks) != ESP_OK) {
        return BSP_ERR_FAIL;
    }
    return BSP_OK;
}

static bool esm_mcpwm_on_timer_empty(mcpwm_timer_handle_t timer,
                                     const mcpwm_timer_event_data_t *edata,
                                     void *user_data)//定时器空事件回调函数，FOC控制任务在这个回调函数被调用时会知道一个新的PWM周期开始了，可以进行一次FOC计算和占空比更新了
{
    (void)timer;
    (void)edata;
    (void)user_data;
    esm_mcpwm_on_timer_common(false);
    return false;
}

static bool esm_mcpwm_on_timer_full(mcpwm_timer_handle_t timer,
                                    const mcpwm_timer_event_data_t *edata,
                                    void *user_data)//定时器满事件回调函数，FOC控制任务在这个回调函数被调用时会知道PWM周期结束了，可以进行一些周期性统计和监控了
{
    (void)timer;
    (void)edata;
    (void)user_data;
    esm_mcpwm_on_timer_common(true);
    return false;
}
/*设置每相的PWM设备，包括创建操作器、比较器和发生器，并配置它们的行为，idx表示相的索引（0~2），high_gpio_num和low_gpio_num分别是高电平和低电平的GPIO引脚号，group_id是MCPWM组号*/
static esp_err_t esm_mcpwm_setup_phase_device(int idx, int high_gpio_num, int low_gpio_num, uint32_t group_id)
{
    mcpwm_operator_config_t oper_cfg = {
        .group_id = (int)group_id,
    };
    mcpwm_comparator_config_t cmpr_cfg = {
        .flags.update_cmp_on_tez = 1,
        .flags.update_cmp_on_tep = 1,
    };
    mcpwm_generator_config_t high_gen_cfg = {
        .gen_gpio_num = high_gpio_num,
    };
    mcpwm_generator_config_t low_gen_cfg = {
        .gen_gpio_num = low_gpio_num,
    };

    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &s_phase_dev[idx].oper), "mcpwm", "new operator failed");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_phase_dev[idx].oper, s_timer), "mcpwm", "connect timer failed");
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_phase_dev[idx].oper, &cmpr_cfg, &s_phase_dev[idx].cmpr), "mcpwm", "new comparator failed");
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_phase_dev[idx].oper, &high_gen_cfg, &s_phase_dev[idx].gen_high), "mcpwm", "new high generator failed");

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            s_phase_dev[idx].gen_high,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)),
        "mcpwm", "set high timer action failed");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            s_phase_dev[idx].gen_high,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_phase_dev[idx].cmpr, MCPWM_GEN_ACTION_HIGH)),
        "mcpwm", "set high compare up action failed");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            s_phase_dev[idx].gen_high,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, s_phase_dev[idx].cmpr, MCPWM_GEN_ACTION_LOW)),
        "mcpwm", "set high compare down action failed");
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_phase_dev[idx].oper, &low_gen_cfg, &s_phase_dev[idx].gen_low), "mcpwm", "new low generator failed");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            s_phase_dev[idx].gen_low,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)),
        "mcpwm", "set low timer action failed");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            s_phase_dev[idx].gen_low,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_phase_dev[idx].cmpr, MCPWM_GEN_ACTION_LOW)),
        "mcpwm", "set low compare up action failed");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            s_phase_dev[idx].gen_low,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, s_phase_dev[idx].cmpr, MCPWM_GEN_ACTION_HIGH)),
        "mcpwm", "set low compare down action failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_phase_dev[idx].cmpr, s_peak_ticks / 2U), "mcpwm", "set initial cmp failed");
    return ESP_OK;
}
/*初始化MCPWM驱动*/
static bsp_status_t esm_drv_mcpwm_ops_init(const esm_bsp_pwm_cfg_t *cfg)
{
    mcpwm_timer_config_t timer_cfg = {0};
    mcpwm_timer_event_callbacks_t cbs = {
        .on_empty = esm_mcpwm_on_timer_empty,//注册定时器空硬件回调
        .on_full = esm_mcpwm_on_timer_full,
    };

    if (s_pwm_ready) {
        return BSP_OK;
    }

    if (cfg == NULL || cfg->freq_hz == 0U) {
        return BSP_ERR_INVALID_ARG;
    }

    {
        uint32_t period_ticks = 10000000U / cfg->freq_hz;
        if (period_ticks == 0U) {
            return BSP_ERR_INVALID_ARG;
        }
        s_peak_ticks = period_ticks / 2U;
        if (s_peak_ticks == 0U) {
            return BSP_ERR_INVALID_ARG;
        }
        timer_cfg.period_ticks = period_ticks;
    }

    timer_cfg.group_id = (int)cfg->timer_num;
    timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = 10000000;
    timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    
    timer_cfg.intr_priority = 0;
    timer_cfg.flags.update_period_on_empty = 1;

    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &s_timer), "mcpwm", "new timer failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_register_event_callbacks(s_timer, &cbs, NULL), "mcpwm", "register timer cb failed");

    ESP_RETURN_ON_ERROR(esm_mcpwm_setup_phase_device(0, cfg->phase_u_high_pin, cfg->phase_u_low_pin, cfg->timer_num), "mcpwm", "setup phase U failed");
    ESP_RETURN_ON_ERROR(esm_mcpwm_setup_phase_device(1, cfg->phase_v_high_pin, cfg->phase_v_low_pin, cfg->timer_num), "mcpwm", "setup phase V failed");
    ESP_RETURN_ON_ERROR(esm_mcpwm_setup_phase_device(2, cfg->phase_w_high_pin, cfg->phase_w_low_pin, cfg->timer_num), "mcpwm", "setup phase W failed");

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_timer), "mcpwm", "enable timer failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP), "mcpwm", "start timer failed");

    s_isr_stats.isr_count = 0;
    s_isr_stats.period_cb_count = 0;
    s_isr_stats.period_cb_drop_count = 0;
    s_pwm_ready = true;
    return BSP_OK;
}
/*设置指定相位的占空比，phase表示相的索引（0~2），duty是新的占空比值，范围是0~1，如果PWM还没有准备好或者phase无效，就返回错误*/
static bsp_status_t esm_drv_mcpwm_ops_set_duty(uint8_t phase, float duty)
{
    return esm_mcpwm_apply_phase_duty(phase, duty);
}

static bsp_status_t esm_drv_mcpwm_ops_enable(void)
{
    return BSP_OK;
}

static bsp_status_t esm_drv_mcpwm_ops_disable(void)
{
    return BSP_OK;
}

static bsp_status_t esm_drv_mcpwm_ops_set_period_callback(esm_bsp_isr_callback_t cb, void *user_ctx)//FOC控制任务调用这个函数注册一个回调函数，这个回调函数会在定时器周期事件回调中被调用，通知FOC控制任务运行一次
{
    s_period_cb = (esm_drv_mcpwm_period_cb_t)cb;
    s_period_cb_ctx = user_ctx;
    return BSP_OK;
}

esp_err_t esm_drv_mcpwm_init(void)//调用函数初始化结构体，之后注册到bsp
{
    static const esm_bsp_pwm_ops_t ops = {
        .init = esm_drv_mcpwm_ops_init,
        .set_duty = esm_drv_mcpwm_ops_set_duty,
        .enable = esm_drv_mcpwm_ops_enable,
        .disable = esm_drv_mcpwm_ops_disable,
        .set_period_callback = esm_drv_mcpwm_ops_set_period_callback,
    };
    const esm_bsp_pwm_cfg_t *cfg = esm_bsp_board_get_pwm_cfg(ESM_PWM_INSTANCE);

    if (cfg == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    if (esm_bsp_pwm_register(ESM_PWM_INSTANCE, &ops, cfg) != BSP_OK) {//注册一个PWM设备实例，供上层调用，ESM_PWM_INSTANCE是这个设备实例的ID，ops是这个设备实例的操作函数集合，cfg是这个设备实例的硬件配置参数
        return ESP_FAIL;
    }
    if (ops.init != NULL && ops.init(cfg) != BSP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t esm_drv_mcpwm_apply_duty(float duty_a, float duty_b, float duty_c)//更新三相的占空比值，输入的占空比值可以是任意实数，函数会将它们限制在0~1范围内，并保存到全局变量中，供定时器周期事件回调函数使用
{
    if (esm_mcpwm_apply_phase_duty(0, duty_a) != BSP_OK) {
        return ESP_FAIL;
    }
    if (esm_mcpwm_apply_phase_duty(1, duty_b) != BSP_OK) {
        return ESP_FAIL;
    }
    if (esm_mcpwm_apply_phase_duty(2, duty_c) != BSP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t esm_drv_mcpwm_register_period_callback(esm_drv_mcpwm_period_cb_t cb, void *user_ctx)//FOC控制任务调用这个函数注册一个回调函数，这个回调函数会在定时器周期事件回调中被调用，通知FOC控制任务运行一次
{
    const esm_bsp_pwm_ops_t *ops = esm_bsp_pwm_get_ops(ESM_PWM_INSTANCE);

    if (ops == NULL || ops->set_period_callback == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return ops->set_period_callback((esm_bsp_isr_callback_t)cb, user_ctx) == BSP_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t esm_drv_mcpwm_register_adc_center_trigger_callback(esm_drv_mcpwm_adc_trigger_cb_t cb, void *user_ctx)
{
    s_adc_trigger_cb = cb;
    s_adc_trigger_ctx = user_ctx;
    return ESP_OK;
}

void esm_drv_mcpwm_get_isr_stats(esm_drv_mcpwm_isr_stats_t *stats)// 获取ISR统计数据，FOC控制任务可以调用这个函数获取定时器周期事件的总调用次数、成功调用回调的次数和丢弃回调的次数等，用于监控ISR的执行情况和性能指标
{
    if (stats == NULL) {
        return;
    }
    *stats = s_isr_stats;
}

void esm_drv_mcpwm_reset_isr_stats(void)// 重置ISR统计数据
{
    s_isr_stats.isr_count = 0;
    s_isr_stats.period_cb_count = 0;
    s_isr_stats.period_cb_drop_count = 0;
}

esp_err_t esm_drv_mcpwm_get_comparator_handle(uint8_t phase, mcpwm_cmpr_handle_t *out_cmpr)
{
    if (out_cmpr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (phase >= 3U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_pwm_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    *out_cmpr = s_phase_dev[phase].cmpr;
    return ESP_OK;
}

