/* FOC control task: wait for PWM period trigger and run one fast-loop iteration. */
#include "task/foc_ctrl/task_foc_ctrl.h"

#include "algorithm/foc_core/foc_core.h"
#include "esp_err.h"
#include "bsp/bsp_hal.h"
#include "bsp/encoder/bsp_encoder.h"
#include "config/foc_config.h"
#include "task/monitor/task_monitor.h"

#include "esp_log.h"
#include "freertos/task.h"

#define ESM_FOC_TASK_PWM_INSTANCE             0        //使用的PWM实例ID
#define ESM_FOC_BUS_V_DEFAULT                 12.0f    //C5不读取母线电压，先固定默认值
#define ESM_FOC_TASK_CORE_ID                  0        //ESP32-C5单核，FOC任务固定到CPU0
#define ESM_FOC_ZERO_REF_EPS_A                0.01f    //FOC算法中判断电流参考是否为零的阈值，单位安培，过大可能导致不输出PWM，过小可能导致死区问题
/*注入矢量对齐编码器，相关的参数*/
#define ESM_FOC_ENCODER_ALIGN_RETRY           5        //编码器校零时的最大重试次数
#define ESM_FOC_ENCODER_ALIGN_DELAY_MS        20       //编码器校零时每次重试的延迟时间，单位毫秒
#define ESM_FOC_ENCODER_ALIGN_HOLD_MS         300      //上电对齐磁场保持时间
#define ESM_FOC_ENCODER_ALIGN_SETTLE_MS       50       //撤销对齐磁场后的稳定时间
/*对齐编码器用的矢量，一定不能大，电流纹波会出事*/
#define ESM_FOC_ALIGN_DUTY_A                  0.51f    
#define ESM_FOC_ALIGN_DUTY_B                  0.50f
#define ESM_FOC_ALIGN_DUTY_C                  0.49f

/*调试日志的开关，和日志输出频率参数，1表示启用日志*/ 
#define ESM_FOC_DEBUG_ENABLE                  1        
#define ESM_FOC_DEBUG_LOG_PERIOD_MS           1000U
/*用于屏蔽adc读取。1:bapassADC采样用于性能定位，0:恢复真实采样*/
#define ESM_FOC_DEBUG_BYPASS_ADC              0        

static const char *TAG = "task_foc_ctrl";

#if ESM_FOC_DEBUG_ENABLE                               //是否启用调试日志
static volatile uint32_t s_foc_isr_notify_count = 0;
#endif

static volatile uint32_t s_foc_isr_divider_count = 0;
static uint32_t s_foc_isr_divider = 1U;

static TaskHandle_t s_foc_task_handle = NULL;
/*=================================================================
*下面这些_cb函数被用在向algo传递bsp/rtos能力用，避免算法层引入硬件，项目耦合。
现在只有上电对齐编码器用到。
=================================================================*/
static esp_err_t esm_task_foc_apply_pwm_cmd_cb(void *user_ctx, const esm_foc_pwm_cmd_t *pwm_cmd)
{
    (void)user_ctx;

    if (pwm_cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esm_bsp_pwm_set_duty(0, pwm_cmd->duty_a) != ESP_OK) {
        return ESP_FAIL;
    }
    if (esm_bsp_pwm_set_duty(1, pwm_cmd->duty_b) != ESP_OK) {
        return ESP_FAIL;
    }
    if (esm_bsp_pwm_set_duty(2, pwm_cmd->duty_c) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t esm_task_foc_encoder_read_angle_cb(void *user_ctx, float *angle_rad)
{
    (void)user_ctx;
    return esm_bsp_encoder_read_angle_rad(angle_rad);
}

static esp_err_t esm_task_foc_encoder_set_zero_cb(void *user_ctx, float zero_offset_rad)
{
    (void)user_ctx;
    return esm_bsp_encoder_set_zero_offset_rad(zero_offset_rad);
}

static void esm_task_foc_delay_ms_cb(void *user_ctx, uint32_t ms)
{
    (void)user_ctx;
    vTaskDelay(esm_task_foc_ms_to_ticks(ms));
}
/*foc_ctrl对齐编码器的函数，本体逻辑在algo层*/
static esp_err_t esm_task_foc_align_encoder_zero(void)
{
    const esm_algo_foc_encoder_align_cfg_t align_cfg = {
        .align_duty_a = ESM_FOC_ALIGN_DUTY_A,
        .align_duty_b = ESM_FOC_ALIGN_DUTY_B,
        .align_duty_c = ESM_FOC_ALIGN_DUTY_C,
        .zero_duty_a = 0.5f,
        .zero_duty_b = 0.5f,
        .zero_duty_c = 0.5f,
        .hold_ms = ESM_FOC_ENCODER_ALIGN_HOLD_MS,
        .settle_ms = ESM_FOC_ENCODER_ALIGN_SETTLE_MS,
        .retry_count = ESM_FOC_ENCODER_ALIGN_RETRY,
        .retry_delay_ms = ESM_FOC_ENCODER_ALIGN_DELAY_MS,
    };

    return esm_algo_foc_align_encoder_zero(&align_cfg,
                                           esm_task_foc_apply_pwm_cmd_cb,
                                           NULL,
                                           esm_task_foc_encoder_read_angle_cb,
                                           NULL,
                                           esm_task_foc_encoder_set_zero_cb,
                                           NULL,
                                           esm_task_foc_delay_ms_cb,
                                           NULL);
}
/*PWM周期中断回调函数，通知FOC控制任务运行一次*/ 
static void esm_task_foc_period_isr_cb(void *user_ctx)
{
    BaseType_t task_woken = pdFALSE;
    TaskHandle_t task = (TaskHandle_t)user_ctx;

#if ESM_FOC_DEBUG_ENABLE
    s_foc_isr_notify_count++;
#endif

    if (s_foc_isr_divider == 0U) {
        s_foc_isr_divider = 1U;
    }

    s_foc_isr_divider_count++;
    if ((s_foc_isr_divider_count % s_foc_isr_divider) != 0U) {
        return;
    }

    if (task != NULL) {
        vTaskNotifyGiveFromISR(task, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }
}
 //FOC控制任务入口函数，等待周期触发，读取传感器数据，运行FOC算法，并输出PWM命令
static void esm_task_foc_ctrl_entry(void *arg) 
{
    ESP_LOGI(TAG, "foc_ctrl task entry start");
    esm_foc_sample_t sample = {0};
    esm_foc_pwm_cmd_t pwm_cmd = {0};
#if !ESM_FOC_DEBUG_BYPASS_ADC                   
    esm_bsp_phase_current_t phase_current = {0};
    uint32_t adc_conv_done_delta = 0;
    uint32_t adc_conv_done_total = 0;
#endif
    const esm_foc_config_t *cfg = esm_cfg_foc_get();
#if ESM_FOC_DEBUG_ENABLE
    TickType_t dbg_last_tick = xTaskGetTickCount();
    uint32_t dbg_loop_count = 0;
    uint32_t dbg_encoder_ok_count = 0;
    uint32_t dbg_encoder_fail_count = 0;
#endif
    float mech_angle_rad = 0.0f;  //机械角，范围是0~2PI

    (void)arg;
    if (cfg == NULL) {
        ESP_LOGE(TAG, "foc config is null");
        return;
    }
    while (1) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#if ESM_FOC_DEBUG_ENABLE
        dbg_loop_count++;
#endif

#if ESM_FOC_DEBUG_BYPASS_ADC
    sample.raw_ia = 0U;
    sample.raw_ib = 0U;
    sample.raw_ic = 0U;
        sample.bus_v = ESM_FOC_BUS_V_DEFAULT;
#else
        sample.bus_v = ESM_FOC_BUS_V_DEFAULT;
        if (esm_bsp_current_sense_read_latest_sample(&phase_current,
                                 &adc_conv_done_delta,
                                 &adc_conv_done_total) == ESP_OK) {
        sample.raw_ia = phase_current.raw_ia;
        sample.raw_ib = phase_current.raw_ib;
        sample.raw_ic = phase_current.raw_ic;
        }
#endif
        if (esm_bsp_encoder_read_angle_rad(&mech_angle_rad) == ESP_OK) {      //读机械角，如果读角度失败就用默认值0，结果范围是0~2PI
            if (esm_algo_foc_mech_to_elec_angle(mech_angle_rad, cfg->motor.pole_pairs, &sample.elec_angle_rad) != ESP_OK) {
                sample.elec_angle_rad = 0.0f;
            }
#if ESM_FOC_DEBUG_ENABLE
            dbg_encoder_ok_count++;
#endif
        } else {
#if ESM_FOC_DEBUG_ENABLE
            dbg_encoder_fail_count++;
#endif
        }

        if (cfg != NULL
            && cfg->current_loop.id_ref_a < ESM_FOC_ZERO_REF_EPS_A
            && cfg->current_loop.id_ref_a > -ESM_FOC_ZERO_REF_EPS_A
            && cfg->current_loop.iq_ref_a < ESM_FOC_ZERO_REF_EPS_A
            && cfg->current_loop.iq_ref_a > -ESM_FOC_ZERO_REF_EPS_A) {
            pwm_cmd.duty_a = 0.5f;
            pwm_cmd.duty_b = 0.5f;
            pwm_cmd.duty_c = 0.5f;
            (void)esm_algo_foc_apply_pwm_cmd(&pwm_cmd, esm_task_foc_apply_pwm_cmd_cb, NULL);
            continue;
        }

        if (esm_algo_foc_run_10khz(&sample, &pwm_cmd) == ESP_OK) {//运行FOC算法，输入采样数据，输出PWM命令，如果算法运行失败就跳过这次输出
            (void)esm_algo_foc_apply_pwm_cmd(&pwm_cmd, esm_task_foc_apply_pwm_cmd_cb, NULL);
        }

#if ESM_FOC_DEBUG_ENABLE
        TickType_t now_tick = xTaskGetTickCount();
        if ((now_tick - dbg_last_tick) >= esm_task_foc_ms_to_ticks(ESM_FOC_DEBUG_LOG_PERIOD_MS)) {
            uint32_t isr_cnt = s_foc_isr_notify_count;
            ESP_LOGI(TAG,
                     "dbg 1s: isr=%u loop=%u enc_ok=%u enc_fail=%u adc_conv_delta=%u adc_conv_total=%u",
                     (unsigned)isr_cnt,
                     (unsigned)dbg_loop_count,
                     (unsigned)dbg_encoder_ok_count,
                     (unsigned)dbg_encoder_fail_count,
                     (unsigned)adc_conv_done_delta,
                     (unsigned)adc_conv_done_total);
            s_foc_isr_notify_count = 0;
            dbg_loop_count = 0;
            dbg_encoder_ok_count = 0;
            dbg_encoder_fail_count = 0;
            dbg_last_tick = now_tick;
        }
#endif
    }
}

esp_err_t esm_task_foc_ctrl_start(void)// FOC控制任务的启动函数，进行必要的初始化和资源准备，然后创建任务
{
    ESP_LOGI(TAG, "foc_ctrl start called");
    BaseType_t ok;                     //下面一段初始化一个电机的实例,并将其注册到FOC算法中
    const esm_foc_config_t *cfg = esm_cfg_foc_get();

    ESP_LOGI(TAG, "startup deps: current_sensor=%p encoder_init=%p",
             (void *)esm_bsp_current_sense_init,
             (void *)esm_bsp_encoder_init);

    if (cfg == NULL) {
        ESP_LOGE(TAG, "foc config is null");
        return ESP_FAIL;
    }

    if (esm_task_monitor_is_test_mode_enabled()) {
        ESP_LOGW(TAG, "monitor test mode enabled, foc task startup skipped");
        return ESP_OK;
    }

    s_foc_isr_divider_count = 0U;
    if (cfg->control.current_loop_notify_divider == 0U) {
        ESP_LOGE(TAG, "current loop notify divider is zero");
        return ESP_FAIL;
    }
    s_foc_isr_divider = cfg->control.current_loop_notify_divider;

    ESP_LOGI(TAG, "starting encoder align phase");
    esm_bsp_encoder_set_verbose(true);
    if (esm_task_foc_align_encoder_zero() != ESP_OK) {
        esm_bsp_encoder_set_verbose(false);
        ESP_LOGE(TAG, "encoder align abnormal");
        return ESP_FAIL;
    }
    esm_bsp_encoder_set_verbose(false);
    ESP_LOGI(TAG, "encoder align done, creating foc task");
 
    ok = xTaskCreatePinnedToCore(esm_task_foc_ctrl_entry,
                                 "foc_ctrl",
                                 4096,
                                 NULL,
                                 10,
                                 &s_foc_task_handle,
                                 ESM_FOC_TASK_CORE_ID);//创建foc任务
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "foc task create failed");
        return ESP_FAIL;  
    }
    ESP_LOGI(TAG, "foc task created handle=%p", (void *)s_foc_task_handle);

    if (esm_bsp_pwm_set_period_callback(esm_task_foc_period_isr_cb, s_foc_task_handle) != ESP_OK) {
        ESP_LOGE(TAG, "pwm set_period_callback failed");
        vTaskDelete(s_foc_task_handle);
        s_foc_task_handle = NULL;
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "pwm period callback registered");
    }

    return ESP_OK;
}

