#include "algorithm/algorithm.h"

#include "esp_check.h"

#include "algorithm/current_loop/current_loop.h"
#include "algorithm/foc_core/foc_core.h"
#include "algorithm/observer/observer.h"
#include "algorithm/speed_loop/speed_loop.h"
#include "algorithm/svpwm/svpwm.h"

esp_err_t esm_algo_init(void)
{
    ESP_RETURN_ON_ERROR(esm_algo_foc_core_init(), "algorithm", "foc core init failed");
    ESP_RETURN_ON_ERROR(esm_algo_current_loop_init(), "algorithm", "current loop init failed");
    ESP_RETURN_ON_ERROR(esm_algo_speed_loop_init(), "algorithm", "speed loop init failed");
    ESP_RETURN_ON_ERROR(esm_algo_svpwm_init(), "algorithm", "svpwm init failed");
    ESP_RETURN_ON_ERROR(esm_algo_observer_init(), "algorithm", "observer init failed");
    return ESP_OK;
}

