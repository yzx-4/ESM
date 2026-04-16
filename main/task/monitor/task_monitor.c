#include "task/monitor/task_monitor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void esm_task_monitor_entry(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t esm_task_monitor_start(void)
{
    BaseType_t ok = xTaskCreate(esm_task_monitor_entry, "monitor", 3072, NULL, 4, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

