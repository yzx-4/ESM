#include "task/safety/task_safety.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void esm_task_safety_entry(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t esm_task_safety_start(void)
{
    BaseType_t ok = xTaskCreate(esm_task_safety_entry, "safety", 3072, NULL, 9, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

