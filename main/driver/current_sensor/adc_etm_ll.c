// Project-local ADC ETM task wrapper
#include "driver/current_sensor/adc_etm_ll.h"
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "adc_etm_ll";

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
    /*
     * The public IDF does not provide a constructor for an ADC ETM task.
     * Create a tiny heap allocation that matches the initial layout
     * expected by esp_etm_channel_connect(): the task id (uint32_t)
     * and the trigger peripheral selector. Downstream code will treat
     * the pointer opaquely as an `esp_etm_task_handle_t`.
     */
    uint32_t *buf = (uint32_t *)malloc(sizeof(uint32_t) * 2);
    if (buf == NULL) {
        ESP_LOGE(TAG, "malloc failed");
        return ESP_ERR_NO_MEM;
    }
    memset(buf, 0, sizeof(uint32_t) * 2);

    buf[0] = (uint32_t)ADC_TASK_START0;
#ifdef ETM_TRIG_PERIPH_ANA_CMPR
    buf[1] = (uint32_t)ETM_TRIG_PERIPH_ANA_CMPR;
#else
    /* fall back to zero if the symbolic name is not present */
    buf[1] = 0;
#endif

    *out_task = (esp_etm_task_handle_t)buf;
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
