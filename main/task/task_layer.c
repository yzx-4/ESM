#include "task/task_layer.h"

#include "esp_check.h"

#include "task/comm/task_comm.h"
#include "task/foc_ctrl/task_foc_ctrl.h"
#include "task/monitor/task_monitor.h"
#include "task/motor_state/task_motor_state.h"


esp_err_t esm_task_startup(void)
{
    ESP_RETURN_ON_ERROR(esm_task_comm_start(), "task", "comm task create failed");
    ESP_RETURN_ON_ERROR(esm_task_motor_state_start(), "task", "motor state task create failed");
    ESP_RETURN_ON_ERROR(esm_task_foc_ctrl_start(), "task", "foc ctrl task create failed");
    ESP_RETURN_ON_ERROR(esm_task_monitor_start(), "task", "monitor task create failed");
    
    return ESP_OK;
}

