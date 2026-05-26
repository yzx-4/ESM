/* Low-level MCPWM implementation: timer/operator setup, duty update, callbacks and stats. */
#include "driver/mcpwm/driver_mcpwm.h"

#include <stdbool.h>

#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "esp_check.h"

typedef struct {
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmpr;
    mcpwm_gen_handle_t gen_high;
    mcpwm_gen_handle_t gen_low;
} esm_pwm_phase_dev_t;

static esm_drv_mcpwm_period_cb_t s_period_cb = NULL;
static void *s_period_cb_ctx = NULL;
static esm_drv_mcpwm_adc_trigger_cb_t s_adc_trigger_cb = NULL;
static void *s_adc_trigger_ctx = NULL;
static bool s_pwm_ready = false;
static uint32_t s_peak_ticks = 0;
static volatile esm_drv_mcpwm_isr_stats_t s_isr_stats;
static mcpwm_timer_handle_t s_timer = NULL;
static esm_pwm_phase_dev_t s_phase_dev[3];

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

static void esm_mcpwm_on_timer_common(bool trigger_adc)
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

static bool esm_mcpwm_on_timer_empty(mcpwm_timer_handle_t timer,
                                     const mcpwm_timer_event_data_t *edata,
                                     void *user_data)
{
    (void)timer;
    (void)edata;
    (void)user_data;
    esm_mcpwm_on_timer_common(false);
    return false;
}

static bool esm_mcpwm_on_timer_full(mcpwm_timer_handle_t timer,
                                    const mcpwm_timer_event_data_t *edata,
                                    void *user_data)
{
    (void)timer;
    (void)edata;
    (void)user_data;
    esm_mcpwm_on_timer_common(true);
    return false;
}

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

static esp_err_t esm_mcpwm_apply_phase_duty(uint8_t phase, float duty)
{
    float duty_clamped = esm_clamp01(duty);
    uint32_t cmp_ticks;

    if (!s_pwm_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (phase >= 3U) {
        return ESP_ERR_INVALID_ARG;
    }

    cmp_ticks = (uint32_t)(duty_clamped * (float)s_peak_ticks);
    if (cmp_ticks > s_peak_ticks) {
        cmp_ticks = s_peak_ticks;
    }
    if (mcpwm_comparator_set_compare_value(s_phase_dev[phase].cmpr, cmp_ticks) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t esm_drv_mcpwm_init(const esm_drv_mcpwm_cfg_t *cfg)
{
    mcpwm_timer_config_t timer_cfg = {0};
    mcpwm_timer_event_callbacks_t cbs = {
        .on_empty = esm_mcpwm_on_timer_empty,
        .on_full = esm_mcpwm_on_timer_full,
    };

    if (s_pwm_ready) {
        return ESP_OK;
    }
    if (cfg == NULL || cfg->freq_hz == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    {
        uint32_t period_ticks = 10000000U / cfg->freq_hz;
        if (period_ticks == 0U) {
            return ESP_ERR_INVALID_ARG;
        }
        s_peak_ticks = period_ticks / 2U;
        if (s_peak_ticks == 0U) {
            return ESP_ERR_INVALID_ARG;
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
    return ESP_OK;
}

esp_err_t esm_drv_mcpwm_set_duty(uint8_t phase, float duty)
{
    return esm_mcpwm_apply_phase_duty(phase, duty);
}

esp_err_t esm_drv_mcpwm_apply_duty(float duty_a, float duty_b, float duty_c)
{
    if (esm_mcpwm_apply_phase_duty(0, duty_a) != ESP_OK) {
        return ESP_FAIL;
    }
    if (esm_mcpwm_apply_phase_duty(1, duty_b) != ESP_OK) {
        return ESP_FAIL;
    }
    if (esm_mcpwm_apply_phase_duty(2, duty_c) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t esm_drv_mcpwm_enable(void)
{
    return ESP_OK;
}

esp_err_t esm_drv_mcpwm_disable(void)
{
    return ESP_OK;
}

esp_err_t esm_drv_mcpwm_set_period_callback(esm_drv_mcpwm_period_cb_t cb, void *user_ctx)
{
    s_period_cb = cb;
    s_period_cb_ctx = user_ctx;
    return ESP_OK;
}

esp_err_t esm_drv_mcpwm_register_adc_center_trigger_callback(esm_drv_mcpwm_adc_trigger_cb_t cb, void *user_ctx)
{
    s_adc_trigger_cb = cb;
    s_adc_trigger_ctx = user_ctx;
    return ESP_OK;
}

void esm_drv_mcpwm_get_isr_stats(esm_drv_mcpwm_isr_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    *stats = s_isr_stats;
}

void esm_drv_mcpwm_reset_isr_stats(void)
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
