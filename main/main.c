#include "esp_err.h"
#include "esp_log.h"

#include "algorithm/algorithm.h"
#include "bsp/bsp.h"
#include "driver/driver.h"
#include "task/task_layer.h"

static const char *TAG = "app_main";

void app_main(void)
{
	bsp_status_t bsp_ret = esm_bsp_init();
    if (bsp_ret != BSP_OK) {
        ESP_LOGE(TAG, "bsp init failed: %d", (int)bsp_ret);
        return;
    }

	if (esm_drv_init() != ESP_OK) {
		ESP_LOGE(TAG, "driver init failed");
		return;
	}
	if (esm_algo_init() != ESP_OK) {
		ESP_LOGE(TAG, "algorithm init failed");
		return;
	}
	if (esm_task_startup() != ESP_OK) {
		ESP_LOGE(TAG, "task startup failed");
		return;
	}

	ESP_LOGI(TAG, "FOC framework startup complete");
}
