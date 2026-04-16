#include "task/motor_state/task_motor_state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void esm_task_motor_state_entry(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t esm_task_motor_state_start(void)
{
    BaseType_t ok = xTaskCreate(esm_task_motor_state_entry, "motor_state", 3072, NULL, 8, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

