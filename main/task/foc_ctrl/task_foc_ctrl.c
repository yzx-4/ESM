/* FOC control task: wait for PWM period trigger and run one fast-loop iteration. */
#include "task/foc_ctrl/task_foc_ctrl.h"

#include "algorithm/foc_core/foc_core.h"
#include "bsp/encoder/bsp_encoder.h"
#include "bsp/bsp_interface.h"
#include "config/foc_config.h"
#include "task/monitor/task_monitor.h"

#include "esp_log.h"
#include "freertos/task.h"

#define ESM_FOC_TASK_CURRENT_SENSOR_INSTANCE  0        //使用的电流传感器实例ID
#define ESM_FOC_TASK_PWM_INSTANCE             0        //使用的PWM实例ID
#define ESM_FOC_TASK_ENCODER_INSTANCE         0        //使用的编码器实例ID
#define ESM_FOC_ENCODER_ALIGN_RETRY           5        //编码器校零时的最大重试次数
#define ESM_FOC_ENCODER_ALIGN_DELAY_MS        20       //编码器校零时每次重试的延迟时间，单位毫秒
#define ESM_FOC_ENCODER_ALIGN_HOLD_MS         300      //上电对齐磁场保持时间
#define ESM_FOC_ENCODER_ALIGN_SETTLE_MS       50       //撤销对齐磁场后的稳定时间
#define ESM_FOC_BUS_V_DEFAULT                 12.0f    //C5不读取母线电压，先固定默认值
#define ESM_FOC_TASK_CORE_ID                  0        //ESP32-C5单核，FOC任务固定到CPU0
#define ESM_FOC_ZERO_REF_EPS_A                0.01f

#define ESM_FOC_ALIGN_DUTY_A                  0.54f
#define ESM_FOC_ALIGN_DUTY_B                  0.46f
#define ESM_FOC_ALIGN_DUTY_C                  0.46f

/* FOC debug switches: set to 0 to fully silence runtime debug logs. */
#define ESM_FOC_DEBUG_ENABLE                  1
#define ESM_FOC_DEBUG_LOG_PERIOD_MS           1000U
#define ESM_FOC_DEBUG_BYPASS_ADC              0        //1:旁路ADC采样用于性能定位，0:恢复真实采样

static const char *TAG = "task_foc_ctrl";

#if ESM_FOC_DEBUG_ENABLE                               //是否启用调试日志
static volatile uint32_t s_foc_isr_notify_count = 0;
#endif

static float esm_task_foc_mech_to_elec_angle(float mech_angle_rad, uint8_t pole_pairs)//机械角转电角，结果范围是0~2PI
{
    const float two_pi = 6.28318530718f;
    float elec = mech_angle_rad * (float)pole_pairs;

    while (elec >= two_pi) {
        elec -= two_pi;
    }
    while (elec < 0.0f) {
        elec += two_pi;
    }
    return elec;
}

static TaskHandle_t s_foc_task_handle = NULL;

static esp_err_t esm_task_foc_apply_pwm_cmd(const esm_foc_pwm_cmd_t *pwm_cmd)//将FOC算法输出的PWM命令传给硬件
{
    const esm_bsp_pwm_ops_t *pwm_ops = esm_bsp_pwm_get_ops(ESM_FOC_TASK_PWM_INSTANCE);

    if (pwm_cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pwm_ops == NULL || pwm_ops->set_duty == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pwm_ops->set_duty(0, pwm_cmd->duty_a) != BSP_OK) {
        return ESP_FAIL;
    }
    if (pwm_ops->set_duty(1, pwm_cmd->duty_b) != BSP_OK) {
        return ESP_FAIL;
    }
    if (pwm_ops->set_duty(2, pwm_cmd->duty_c) != BSP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t esm_task_foc_align_encoder_zero(const esm_bsp_encoder_ops_t *encoder_ops,
                                                 const esm_bsp_pwm_ops_t *pwm_ops)//FOC控制前对编码器进行校零，先注入固定磁场锁定转子，再记录零点
{
    float angle_rad = 0.0f;
    int retry = 0;

    if (encoder_ops == NULL || encoder_ops->read_angle_rad == NULL || encoder_ops->set_zero_offset_rad == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pwm_ops == NULL || pwm_ops->set_duty == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 注入固定电压矢量对齐转子. */
    if (pwm_ops->set_duty(0, ESM_FOC_ALIGN_DUTY_A) != BSP_OK
        || pwm_ops->set_duty(1, ESM_FOC_ALIGN_DUTY_B) != BSP_OK
        || pwm_ops->set_duty(2, ESM_FOC_ALIGN_DUTY_C) != BSP_OK) {
        return ESP_FAIL;
    }
    vTaskDelay(esm_task_foc_ms_to_ticks(ESM_FOC_ENCODER_ALIGN_HOLD_MS));

    vTaskDelay(esm_task_foc_ms_to_ticks(100));
/* 读取当前角度并设置为零点，重试多次以提高成功率. */
    for (retry = 0; retry < ESM_FOC_ENCODER_ALIGN_RETRY; retry++) {
            ESP_LOGI(TAG, "align retry %d/%d", retry + 1, ESM_FOC_ENCODER_ALIGN_RETRY);
        if (encoder_ops->read_angle_rad(&angle_rad) == BSP_OK) {
            if (encoder_ops->set_zero_offset_rad(angle_rad) != BSP_OK) {
                    ESP_LOGE(TAG, "set_zero_offset_rad failed");
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "encoder aligned, zero=%.4f rad", (double)angle_rad);
            (void)pwm_ops->set_duty(0, 0.5f);
            (void)pwm_ops->set_duty(1, 0.5f);
            (void)pwm_ops->set_duty(2, 0.5f);
            vTaskDelay(pdMS_TO_TICKS(ESM_FOC_ENCODER_ALIGN_SETTLE_MS));
            return ESP_OK;
        }
        vTaskDelay(esm_task_foc_ms_to_ticks(ESM_FOC_ENCODER_ALIGN_DELAY_MS));
    }

    (void)pwm_ops->set_duty(0, 0.5f);
    (void)pwm_ops->set_duty(1, 0.5f);
    (void)pwm_ops->set_duty(2, 0.5f);
    ESP_LOGE(TAG, "encoder align abnormal after %d retries", ESM_FOC_ENCODER_ALIGN_RETRY);
    return ESP_FAIL;
}

static void esm_task_foc_period_isr_cb(void *user_ctx)// PWM周期中断回调函数，通知FOC控制任务运行一次
{
    BaseType_t task_woken = pdFALSE;
    TaskHandle_t task = (TaskHandle_t)user_ctx;

#if ESM_FOC_DEBUG_ENABLE
    s_foc_isr_notify_count++;
#endif

    if (task != NULL) {
        vTaskNotifyGiveFromISR(task, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }
}

static void esm_task_foc_ctrl_entry(void *arg)  // FOC控制任务入口函数，等待周期触发，读取传感器数据，运行FOC算法，并输出PWM命令
{
    ESP_LOGI(TAG, "foc_ctrl task entry start");
    esm_foc_sample_t sample = {0};
    esm_foc_pwm_cmd_t pwm_cmd = {0};
#if !ESM_FOC_DEBUG_BYPASS_ADC
    esm_bsp_phase_current_t phase_current = {0};
    const esm_bsp_current_sense_ops_t *current_ops = esm_bsp_current_sense_get_ops(ESM_FOC_TASK_CURRENT_SENSOR_INSTANCE);
#endif
    const esm_bsp_encoder_ops_t *encoder_ops = esm_bsp_encoder_get_ops(ESM_FOC_TASK_ENCODER_INSTANCE);
    const esm_foc_config_t *cfg = esm_cfg_foc_get();
#if ESM_FOC_DEBUG_ENABLE
    TickType_t dbg_last_tick = xTaskGetTickCount();
    uint32_t dbg_loop_count = 0;
    uint32_t dbg_encoder_ok_count = 0;
    uint32_t dbg_encoder_fail_count = 0;
#endif
    float mech_angle_rad = 0.0f;  //机械角，范围是0~2PI

    (void)arg;
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
        if (current_ops != NULL && current_ops->trigger_fast_sample != NULL   //触发并读取两相电流，失败就用默认值0A
            && current_ops->trigger_fast_sample(&phase_current) == BSP_OK) {
    sample.raw_ia = phase_current.raw_ia;
    sample.raw_ib = phase_current.raw_ib;
    sample.raw_ic = phase_current.raw_ic;
        }
#endif
        if (encoder_ops != NULL && encoder_ops->read_angle_rad != NULL      //读机械角，如果读角度失败就用默认值0，结果范围是0~2PI
            && encoder_ops->read_angle_rad(&mech_angle_rad) == BSP_OK) {
            sample.elec_angle_rad = esm_task_foc_mech_to_elec_angle(mech_angle_rad, cfg->motor.pole_pairs);
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
            (void)esm_task_foc_apply_pwm_cmd(&pwm_cmd);
            continue;
        }

        if (esm_algo_foc_run_10khz(&sample, &pwm_cmd) == ESP_OK) {//运行FOC算法，输入采样数据，输出PWM命令，如果算法运行失败就跳过这次输出
            (void)esm_task_foc_apply_pwm_cmd(&pwm_cmd);
        }

#if ESM_FOC_DEBUG_ENABLE
        TickType_t now_tick = xTaskGetTickCount();
        if ((now_tick - dbg_last_tick) >= esm_task_foc_ms_to_ticks(ESM_FOC_DEBUG_LOG_PERIOD_MS)) {
            uint32_t isr_cnt = s_foc_isr_notify_count;
            ESP_LOGI(TAG,
                     "dbg 1s: isr=%u loop=%u enc_ok=%u enc_fail=%u",
                     (unsigned)isr_cnt,
                     (unsigned)dbg_loop_count,
                     (unsigned)dbg_encoder_ok_count,
                     (unsigned)dbg_encoder_fail_count);
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
    const esm_bsp_current_sense_ops_t *current_ops = esm_bsp_current_sense_get_ops(ESM_FOC_TASK_CURRENT_SENSOR_INSTANCE);
    const esm_bsp_pwm_ops_t *pwm_ops = esm_bsp_pwm_get_ops(ESM_FOC_TASK_PWM_INSTANCE);
    const esm_bsp_encoder_ops_t *encoder_ops = esm_bsp_encoder_get_ops(ESM_FOC_TASK_ENCODER_INSTANCE);

    ESP_LOGI(TAG, "startup deps: current_ops=%p pwm_ops=%p encoder_ops=%p cfg=%p",
             (void *)current_ops,
             (void *)pwm_ops,
             (void *)encoder_ops,
             (void *)esm_bsp_current_sense_get_cfg(ESM_FOC_TASK_CURRENT_SENSOR_INSTANCE));

    if (current_ops == NULL || esm_bsp_current_sense_get_cfg(ESM_FOC_TASK_CURRENT_SENSOR_INSTANCE) == NULL) {
        ESP_LOGE(TAG, "current sensor ops or cfg missing");
        return ESP_ERR_INVALID_STATE;
    }
    if (pwm_ops == NULL || pwm_ops->set_period_callback == NULL) {
        ESP_LOGE(TAG, "pwm ops or set_period_callback missing");
        return ESP_ERR_INVALID_STATE;
    }
    if (encoder_ops == NULL || encoder_ops->read_angle_rad == NULL) {
        ESP_LOGE(TAG, "encoder ops or read_angle_rad missing");
        return ESP_ERR_INVALID_STATE;
    }

    if (esm_task_monitor_is_test_mode_enabled()) {
        ESP_LOGW(TAG, "monitor test mode enabled, foc task startup skipped");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "starting encoder align phase");
    esm_bsp_encoder_set_verbose(true);
    if (esm_task_foc_align_encoder_zero(encoder_ops, pwm_ops) != ESP_OK) {
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

    if (pwm_ops->set_period_callback(esm_task_foc_period_isr_cb, s_foc_task_handle) != BSP_OK) {
        ESP_LOGE(TAG, "pwm set_period_callback failed");
        vTaskDelete(s_foc_task_handle);
        s_foc_task_handle = NULL;
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "pwm period callback registered");
    }

    return ESP_OK;
}

