#include "task/comm/task_comm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "task_comm";
static QueueHandle_t s_control_cmd_queue = NULL;
static QueueHandle_t s_motor_feedback_queue = NULL;

static const control_command_t s_default_command = {
    .mode = ESM_CONTROL_MODE_NONE,
    .u_bus = 0U,
    .target_torque = 0.0f,
    .target_velocity = 0.0f,
    .target_position = 0.0f,
    .torque_limit = 0.0f,
};

static const motor_feedback_t s_default_feedback = {
    .actual_torque = 0.0f,
    .actual_velocity = 0.0f,
    .actual_position = 0.0f,
    .iq_measured = 0.0f,
    .vbus_voltage = 0.0f,
    .fault_code = 0U,
};

esp_err_t esm_task_comm_set_control_command(const control_command_t *cmd)
{
    if (cmd == NULL || s_control_cmd_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueueOverwrite(s_control_cmd_queue, cmd) == pdPASS ? ESP_OK : ESP_FAIL;
}

esp_err_t esm_task_comm_get_control_command(control_command_t *cmd)
{
    if (cmd == NULL || s_control_cmd_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueuePeek(s_control_cmd_queue, cmd, 0) == pdPASS ? ESP_OK : ESP_FAIL;
}

esp_err_t esm_task_comm_publish_motor_feedback(const motor_feedback_t *fb)
{
    if (fb == NULL || s_motor_feedback_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueueOverwrite(s_motor_feedback_queue, fb) == pdPASS ? ESP_OK : ESP_FAIL;
}

esp_err_t esm_task_comm_get_latest_motor_feedback(motor_feedback_t *fb)
{
    if (fb == NULL || s_motor_feedback_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueuePeek(s_motor_feedback_queue, fb, 0) == pdPASS ? ESP_OK : ESP_FAIL;
}

static void esm_task_comm_entry(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "comm task ready, queue framework initialized");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t esm_task_comm_start(void)//启用通信任务，目前空实现，后续可以添加CAN通信等功能
{
    if (s_control_cmd_queue == NULL) {
        s_control_cmd_queue = xQueueCreate(1, sizeof(control_command_t));
    }
    if (s_motor_feedback_queue == NULL) {
        s_motor_feedback_queue = xQueueCreate(1, sizeof(motor_feedback_t));
    }
    if (s_control_cmd_queue == NULL || s_motor_feedback_queue == NULL) {
        return ESP_FAIL;
    }

    (void)xQueueOverwrite(s_control_cmd_queue, &s_default_command);
    (void)xQueueOverwrite(s_motor_feedback_queue, &s_default_feedback);

    BaseType_t ok = xTaskCreate(esm_task_comm_entry, "comm", 3072, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

