// Project-local ADC ETM task wrapper
#include "driver/current_sensor/adc_etm_ll.h"
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_private/etm_interface.h"

#include "esp_heap_caps.h"

static const char *TAG = "adc_etm_ll";

static esp_err_t esm_drv_adc_etm_del_task(esp_etm_task_t *task)
{
    free(task);
    return ESP_OK;
}

esp_err_t esm_drv_adc_etm_new_start_task(esp_etm_task_handle_t *out_task)
{
#if CONFIG_IDF_TARGET_ESP32C5
    if (out_task == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

/* If IDF headers don't expose ADC_TASK_START0 for this target, provide
 * a conservative fallback value known from SoC headers. */
#ifndef ADC_TASK_START0
#define ADC_TASK_START0 125u
#endif

#ifdef ADC_TASK_START0
    esp_etm_task_t *task = heap_caps_calloc(1, sizeof(esp_etm_task_t), MALLOC_CAP_DEFAULT);
    if (task == NULL) {
        ESP_LOGE(TAG, "malloc failed");
        return ESP_ERR_NO_MEM;
    }
    task->task_id = (uint32_t)ADC_TASK_START0;
    task->trig_periph = ETM_TRIG_PERIPH_ANA_CMPR;
    task->del = esm_drv_adc_etm_del_task;

    *out_task = (esp_etm_task_handle_t)task;
    return ESP_OK;
#else
    ESP_LOGE(TAG, "ADC ETM start task ID macro not available");
    return ESP_ERR_NOT_SUPPORTED;
#endif
#else
    (void)out_task;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
