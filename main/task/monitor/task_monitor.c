#include "task/monitor/task_monitor.h"

#include <math.h>

#include "bsp/bsp_interface.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ESM_MONITOR_PWM_INSTANCE        0
#define ESM_MONITOR_ENCODER_INSTANCE    0
#define ESM_MONITOR_TWO_PI              6.28318530718f

#define ESM_MONITOR_PHASE_DUTY_U        0.30f
#define ESM_MONITOR_PHASE_DUTY_V        0.10f
#define ESM_MONITOR_PHASE_DUTY_W        0.50f

#define ESM_MONITOR_VECTOR_MODULATION   0.01f
#define ESM_MONITOR_VECTOR_ELEC_HZ      1.0f
#define ESM_MONITOR_VECTOR_STEP_MS      2U
#define ESM_MONITOR_VECTOR_LOG_MS       1000U

static const char *TAG = "task_monitor";

bool esm_task_monitor_is_test_mode_enabled(void)
{
    return (ESM_MONITOR_TEST_MODE != ESM_MONITOR_TEST_MODE_NONE);
}

#if ESM_MONITOR_TEST_MODE == ESM_MONITOR_TEST_MODE_OPEN_LOOP_VECTOR
static float esm_monitor_wrap_0_2pi(float a)
{
    while (a >= ESM_MONITOR_TWO_PI) {
        a -= ESM_MONITOR_TWO_PI;
    }
    while (a < 0.0f) {
        a += ESM_MONITOR_TWO_PI;
    }
    return a;
}

static float esm_monitor_wrap_pm_pi(float a)
{
    while (a > 3.14159265359f) {
        a -= ESM_MONITOR_TWO_PI;
    }
    while (a < -3.14159265359f) {
        a += ESM_MONITOR_TWO_PI;
    }
    return a;
}

static float esm_monitor_clamp01(float x)
{
    if (x < 0.0f) {
        return 0.0f;
    }
    if (x > 1.0f) {
        return 1.0f;
    }
    return x;
}

static void esm_monitor_set_neutral(const esm_bsp_pwm_ops_t *pwm_ops)
{
    if (pwm_ops == NULL || pwm_ops->set_duty == NULL) {
        return;
    }
    (void)pwm_ops->set_duty(0, 0.5f);
    (void)pwm_ops->set_duty(1, 0.5f);
    (void)pwm_ops->set_duty(2, 0.5f);
}

#endif

#if ESM_MONITOR_TEST_MODE == ESM_MONITOR_TEST_MODE_PHASE_DUTY
static void esm_monitor_run_phase_duty_test(const esm_bsp_pwm_ops_t *pwm_ops)
{
    ESP_LOGW(TAG,
             "phase duty test active: U=%.2f V=%.2f W=%.2f",
             (double)ESM_MONITOR_PHASE_DUTY_U,
             (double)ESM_MONITOR_PHASE_DUTY_V,
             (double)ESM_MONITOR_PHASE_DUTY_W);
    (void)pwm_ops->set_duty(0, ESM_MONITOR_PHASE_DUTY_U);
    (void)pwm_ops->set_duty(1, ESM_MONITOR_PHASE_DUTY_V);
    (void)pwm_ops->set_duty(2, ESM_MONITOR_PHASE_DUTY_W);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

#if ESM_MONITOR_TEST_MODE == ESM_MONITOR_TEST_MODE_OPEN_LOOP_VECTOR
static void esm_monitor_run_open_loop_vector_continuous(const esm_bsp_pwm_ops_t *pwm_ops,
                                                        const esm_bsp_encoder_ops_t *enc_ops)
{
    const float dt_s = (float)ESM_MONITOR_VECTOR_STEP_MS / 1000.0f;
    const float omega_e = ESM_MONITOR_TWO_PI * ESM_MONITOR_VECTOR_ELEC_HZ;
    float theta_e = 0.0f;
    float prev_mech = 0.0f;
    float accum_mech = 0.0f;
    bool have_prev = false;
    TickType_t last_log_tick = xTaskGetTickCount();

    ESP_LOGW(TAG,
             "open-loop vector test: continuous m=%.3f elec_hz=%.2f",
             (double)ESM_MONITOR_VECTOR_MODULATION,
             (double)ESM_MONITOR_VECTOR_ELEC_HZ);

    esm_monitor_set_neutral(pwm_ops);
    while (1) {
        float du;
        float dv;
        float dw;
        float mech = 0.0f;

        theta_e = esm_monitor_wrap_0_2pi(theta_e + omega_e * dt_s);

        du = 0.5f + ESM_MONITOR_VECTOR_MODULATION * sinf(theta_e);
        dv = 0.5f + ESM_MONITOR_VECTOR_MODULATION * sinf(theta_e - 2.09439510239f);
        dw = 0.5f + ESM_MONITOR_VECTOR_MODULATION * sinf(theta_e + 2.09439510239f);

        (void)pwm_ops->set_duty(0, esm_monitor_clamp01(du));
        (void)pwm_ops->set_duty(1, esm_monitor_clamp01(dv));
        (void)pwm_ops->set_duty(2, esm_monitor_clamp01(dw));

        if (enc_ops != NULL && enc_ops->read_angle_rad != NULL && enc_ops->read_angle_rad(&mech) == BSP_OK) {
            if (!have_prev) {
                prev_mech = mech;
                have_prev = true;
            } else {
                float delta = esm_monitor_wrap_pm_pi(mech - prev_mech);
                accum_mech += fabsf(delta);
                prev_mech = mech;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(ESM_MONITOR_VECTOR_STEP_MS));

        if ((xTaskGetTickCount() - last_log_tick) >= pdMS_TO_TICKS(ESM_MONITOR_VECTOR_LOG_MS)) {
            ESP_LOGI(TAG, "open-loop vector running: accum_mech=%.3f rad", (double)accum_mech);
            accum_mech = 0.0f;
            last_log_tick = xTaskGetTickCount();
        }
    }
}
#endif

static void esm_task_monitor_entry(void *arg)
{
    const esm_bsp_pwm_ops_t *pwm_ops = esm_bsp_pwm_get_ops(ESM_MONITOR_PWM_INSTANCE);
#if ESM_MONITOR_TEST_MODE == ESM_MONITOR_TEST_MODE_OPEN_LOOP_VECTOR
    const esm_bsp_encoder_ops_t *enc_ops = esm_bsp_encoder_get_ops(ESM_MONITOR_ENCODER_INSTANCE);
#endif

    (void)arg;

    if (ESM_MONITOR_TEST_MODE != ESM_MONITOR_TEST_MODE_NONE) {
        if (pwm_ops == NULL || pwm_ops->set_duty == NULL) {
            ESP_LOGE(TAG, "test mode enabled but pwm ops unavailable");
            vTaskDelete(NULL);
            return;
        }

#if ESM_MONITOR_TEST_MODE == ESM_MONITOR_TEST_MODE_PHASE_DUTY
        esm_monitor_run_phase_duty_test(pwm_ops);
#elif ESM_MONITOR_TEST_MODE == ESM_MONITOR_TEST_MODE_OPEN_LOOP_VECTOR
    esm_monitor_run_open_loop_vector_continuous(pwm_ops, enc_ops);
#else
        ESP_LOGW(TAG, "unknown monitor test mode=%d", ESM_MONITOR_TEST_MODE);
#endif
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t esm_task_monitor_start(void)
{
    BaseType_t ok = xTaskCreate(esm_task_monitor_entry, "monitor", 3072, NULL, 4, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

