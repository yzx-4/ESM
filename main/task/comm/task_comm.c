#include "task/comm/task_comm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void esm_task_comm_entry(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t esm_task_comm_start(void)//启用通信任务，目前空实现，后续可以添加CAN通信等功能
{
    BaseType_t ok = xTaskCreate(esm_task_comm_entry, "comm", 3072, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

