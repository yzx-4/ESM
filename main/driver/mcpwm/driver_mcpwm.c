/* MCPWM implementation: timer/operator setup, duty update, ISR callback and stats. */
#include "driver/mcpwm/driver_mcpwm.h"

#include <stdbool.h>

#include "bsp/board/board.h"
#include "bsp/bsp_interface.h"
#include "driver/mcpwm_prelude.h"
#include "esp_check.h"

#define ESM_PWM_INSTANCE  0

typedef struct {//每相的PWM设备结构体，包含一个操作器、一个比较器和两个发生器（高电平和低电平），用于控制每相的PWM输出
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmpr;
    mcpwm_gen_handle_t gen_high;
    mcpwm_gen_handle_t gen_low;
} esm_pwm_phase_dev_t;

static float s_last_duty_a = 0.5f;                    //保存每相的最后占空比值，初始值为0.5，表示默认输出50%占空比的PWM信号
static float s_last_duty_b = 0.5f;                      
static float s_last_duty_c = 0.5f;
static esm_drv_mcpwm_period_cb_t s_period_cb = NULL;  //FOC控制任务在定时器周期事件回调中被通知运行一次，s_period_cb就是保存这个回调函数的指针，初始值为NULL表示没有注册回调  
static void *s_period_cb_ctx = NULL;                  //回调函数的用户上下文指针，FOC控制任务在注册回调时可以传入一个指针，这个指针会在回调被调用时传回给FOC控制任务，供其使用
static bool s_pwm_ready = false;                      //标志位，表示PWM已经初始化完成，可以开始接受占空比更新，如果FOC控制任务在PWM准备好之前就调用了更新占空比的函数，就会返回错误
static uint32_t s_period_ticks = 0;                   //定时器周期对应的计数值，根据配置的PWM频率计算得到，用于设置定时器的周期和比较器的初始值
static volatile esm_drv_mcpwm_isr_stats_t s_isr_stats;//ISR统计数据结构体，包含定时器周期事件的总调用次数、成功调用回调的次数和丢弃回调的次数等，用于监控ISR的执行情况和性能指标

static mcpwm_timer_handle_t s_timer = NULL;           //MCPWM定时器句柄，代表一个定时器实例，用于生成PWM周期事件和触发ISR回调
static esm_pwm_phase_dev_t s_phase_dev[3];            //三相PWM设备结构体数组，每个元素包含一个操作器、一个比较器和两个发生器（高电平和低电平），用于控制每相的PWM输出

static float esm_clamp01(float x)//将输入的占空比限制在0~1范围内
{
    if (x < 0.0f) {
        return 0.0f;
    }
    if (x > 1.0f) {
        return 1.0f;
    }
    return x;
}

static bool esm_mcpwm_on_timer_empty(mcpwm_timer_handle_t timer,
                                     const mcpwm_timer_event_data_t *edata,
                                     void *user_data)//定时器空事件回调函数，每当定时器计数到达周期值时触发，通知FOC控制任务运行一次
{
    (void)timer;
    (void)edata;
    (void)user_data;
    s_isr_stats.isr_count++;
    if (s_period_cb != NULL) {
        s_isr_stats.period_cb_count++;
        s_period_cb(s_period_cb_ctx);
    } else {
        s_isr_stats.period_cb_drop_count++;
    }
    return false;
}

static esp_err_t esm_mcpwm_setup_phase_device(int idx, int high_gpio_num, int low_gpio_num, uint32_t group_id)//设置每相的PWM设备，包括创建操作器、比较器和发生器，并配置它们的行为，idx表示相的索引（0~2），high_gpio_num和low_gpio_num分别是高电平和低电平的GPIO引脚号，group_id是MCPWM组号
{
    mcpwm_operator_config_t oper_cfg = {
        .group_id = (int)group_id,
    };
    mcpwm_comparator_config_t cmpr_cfg = {
        .flags.update_cmp_on_tez = 1,
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
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)),
        "mcpwm", "set timer action failed");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            s_phase_dev[idx].gen_high,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_phase_dev[idx].cmpr, MCPWM_GEN_ACTION_LOW)),
        "mcpwm", "set compare action failed");
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_phase_dev[idx].oper, &low_gen_cfg, &s_phase_dev[idx].gen_low), "mcpwm", "new low generator failed");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(
            s_phase_dev[idx].gen_low,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)),
        "mcpwm", "set low timer action failed");
    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(
            s_phase_dev[idx].gen_low,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_phase_dev[idx].cmpr, MCPWM_GEN_ACTION_HIGH)),
        "mcpwm", "set low compare action failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_phase_dev[idx].cmpr, s_period_ticks / 2U), "mcpwm", "set initial cmp failed");
    return ESP_OK;
}

static bsp_status_t esm_drv_mcpwm_ops_init(const esm_bsp_pwm_cfg_t *cfg)
{
    mcpwm_timer_config_t timer_cfg = {0};
    mcpwm_timer_event_callbacks_t cbs = {
        .on_empty = esm_mcpwm_on_timer_empty,
    };

    if (cfg == NULL || cfg->freq_hz == 0U) {
        return BSP_ERR_INVALID_ARG;
    }

    s_period_ticks = 10000000U / cfg->freq_hz;
    if (s_period_ticks == 0U) {
        return BSP_ERR_INVALID_ARG;
    }

    timer_cfg.group_id = (int)cfg->timer_num;
    timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = 10000000;
    timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_cfg.period_ticks = s_period_ticks;
    timer_cfg.intr_priority = 0;
    timer_cfg.flags.update_period_on_empty = 1;

    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &s_timer), "mcpwm", "new timer failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_register_event_callbacks(s_timer, &cbs, NULL), "mcpwm", "register timer cb failed");

    ESP_RETURN_ON_ERROR(esm_mcpwm_setup_phase_device(0, cfg->phase_u_high_pin, cfg->phase_u_low_pin, cfg->timer_num), "mcpwm", "setup phase U failed");
    ESP_RETURN_ON_ERROR(esm_mcpwm_setup_phase_device(1, cfg->phase_v_high_pin, cfg->phase_v_low_pin, cfg->timer_num), "mcpwm", "setup phase V failed");
    ESP_RETURN_ON_ERROR(esm_mcpwm_setup_phase_device(2, cfg->phase_w_high_pin, cfg->phase_w_low_pin, cfg->timer_num), "mcpwm", "setup phase W failed");

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_timer), "mcpwm", "enable timer failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP), "mcpwm", "start timer failed");

    s_pwm_ready = true;
    return BSP_OK;
}

static bsp_status_t esm_drv_mcpwm_ops_set_duty(uint8_t phase, float duty)//012分别对应UVW相,只设置占空比,不更新值
{
    float duty_clamped = esm_clamp01(duty);//将输入的占空比限制在0~1范围内
    if (phase == 0) {
        s_last_duty_a = duty_clamped;
    } else if (phase == 1) {
        s_last_duty_b = duty_clamped;
    } else if (phase == 2) {
        s_last_duty_c = duty_clamped;
    } else {
        return BSP_ERR_INVALID_ARG;
    }
    return BSP_OK;
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
    if (esm_bsp_pwm_register(ESM_PWM_INSTANCE, &ops, cfg) != BSP_OK) {
        return ESP_FAIL;
    }
    if (ops.init != NULL && ops.init(cfg) != BSP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t esm_drv_mcpwm_apply_duty(float duty_a, float duty_b, float duty_c)//更新三相的占空比值，输入的占空比值可以是任意实数，函数会将它们限制在0~1范围内，并保存到全局变量中，供定时器周期事件回调函数使用
{
    if (!s_pwm_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    s_last_duty_a = esm_clamp01(duty_a);
    s_last_duty_b = esm_clamp01(duty_b);
    s_last_duty_c = esm_clamp01(duty_c);

    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_phase_dev[0].cmpr, (uint32_t)(s_last_duty_a * (float)s_period_ticks)), "mcpwm", "set cmp U failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_phase_dev[1].cmpr, (uint32_t)(s_last_duty_b * (float)s_period_ticks)), "mcpwm", "set cmp V failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_phase_dev[2].cmpr, (uint32_t)(s_last_duty_c * (float)s_period_ticks)), "mcpwm", "set cmp W failed");
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

