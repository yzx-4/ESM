#include "task/motor_state/task_motor_state.h"

#include <math.h>
#include <stdbool.h>

#include "algorithm/foc_core/foc_core.h"
#include "bsp/encoder/bsp_encoder.h"
#include "config/foc_config.h"
#include "esp_log.h"
#include "task/comm/task_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "task_motor_state";

#define ESM_MOTOR_STATE_LOOP_MS          1U
#define ESM_MOTOR_STATE_SPEED_KP         0.06f
#define ESM_MOTOR_STATE_SPEED_KI         0.40f
#define ESM_MOTOR_STATE_SPEED_INTEGRAL_CLAMP  5.0f

static float s_speed_integrator = 0.0f;
static float s_last_mech_angle_rad = 0.0f;
static bool s_last_mech_angle_valid = false;

static float esm_motor_state_wrap_pm_pi(float angle_rad)
{
    const float two_pi = 6.28318530718f;
    while (angle_rad > 3.14159265359f) {
        angle_rad -= two_pi;
    }
    while (angle_rad < -3.14159265359f) {
        angle_rad += two_pi;
    }
    return angle_rad;
}

static float esm_motor_state_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float esm_motor_state_clamp_ref(float iq_ref_a, float torque_limit_a)
{
    const esm_foc_config_t *cfg = esm_cfg_foc_get();
    float limit = torque_limit_a;

    if (cfg != NULL && cfg->current_loop.current_limit_a > 0.0f) {
        if (limit <= 0.0f || cfg->current_loop.current_limit_a < limit) {
            limit = cfg->current_loop.current_limit_a;
        }
    }
    if (limit <= 0.0f) {
        return iq_ref_a;
    }
    return esm_motor_state_clamp(iq_ref_a, -limit, limit);
}

static void esm_task_motor_state_entry(void *arg)
{
    control_command_t command = {0};
    motor_feedback_t feedback = {0};
    float mech_angle_rad = 0.0f;
    float mech_speed_rad_s = 0.0f;
    float iq_ref_a = 0.0f;
    float speed_error = 0.0f;
    float dt_s = (float)ESM_MOTOR_STATE_LOOP_MS / 1000.0f;

    (void)arg;
    ESP_LOGI(TAG, "motor state task ready");
    while (1) {
        if (esm_task_comm_get_control_command(&command) != ESP_OK) {
            command.mode = ESM_CONTROL_MODE_NONE;
        }

        if (esm_bsp_encoder_read_angle_rad(&mech_angle_rad) == ESP_OK) {
            if (s_last_mech_angle_valid) {
                float delta = esm_motor_state_wrap_pm_pi(mech_angle_rad - s_last_mech_angle_rad);
                mech_speed_rad_s = delta / dt_s;
            }
            s_last_mech_angle_rad = mech_angle_rad;
            s_last_mech_angle_valid = true;
        } else {
            mech_speed_rad_s = 0.0f;
        }

        switch (command.mode) {
        case ESM_CONTROL_MODE_TORQUE:
            iq_ref_a = esm_motor_state_clamp_ref(command.target_torque, command.torque_limit);
            s_speed_integrator = 0.0f;
            break;
        case ESM_CONTROL_MODE_VELOCITY:
            speed_error = command.target_velocity - mech_speed_rad_s;
            s_speed_integrator += ESM_MOTOR_STATE_SPEED_KI * speed_error * dt_s;
            s_speed_integrator = esm_motor_state_clamp(s_speed_integrator,
                                                      -ESM_MOTOR_STATE_SPEED_INTEGRAL_CLAMP,
                                                      ESM_MOTOR_STATE_SPEED_INTEGRAL_CLAMP);
            iq_ref_a = ESM_MOTOR_STATE_SPEED_KP * speed_error + s_speed_integrator;
            iq_ref_a = esm_motor_state_clamp_ref(iq_ref_a, command.torque_limit);
            break;
        case ESM_CONTROL_MODE_POSITION:
            iq_ref_a = 0.0f;
            s_speed_integrator = 0.0f;
            break;
        case ESM_CONTROL_MODE_NONE:
        default:
            iq_ref_a = 0.0f;
            s_speed_integrator = 0.0f;
            break;
        }

        (void)esm_algo_foc_set_current_ref(0.0f, iq_ref_a);

        feedback.actual_position = mech_angle_rad;
        feedback.actual_velocity = mech_speed_rad_s;
        feedback.actual_torque = iq_ref_a;
        feedback.iq_measured = 0.0f;
        feedback.vbus_voltage = (float)command.u_bus;
        feedback.fault_code = 0U;
        (void)esm_task_comm_publish_motor_feedback(&feedback);

        vTaskDelay(pdMS_TO_TICKS(ESM_MOTOR_STATE_LOOP_MS));
    }
}

esp_err_t esm_task_motor_state_start(void)
{
    BaseType_t ok = xTaskCreate(esm_task_motor_state_entry, "motor_state", 3072, NULL, 8, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

