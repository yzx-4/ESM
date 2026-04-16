/* Driver layer unified init entry: initialize control-path hardware modules. */
#include "driver/driver.h"

#include "esp_check.h"

#include "driver/can/driver_can.h"
#include "driver/current_sensor/driver_current_sensor.h"
#include "driver/encoder/driver_encoder.h"
#include "driver/mcpwm/driver_mcpwm.h"

esp_err_t esm_drv_init(void)
{
    /* Key control-path modules must be ready before closing current loop. */
    ESP_RETURN_ON_ERROR(esm_drv_mcpwm_init(), "driver", "mcpwm init failed");
    ESP_RETURN_ON_ERROR(esm_drv_current_sensor_init(), "driver", "current sensor init failed");
    ESP_RETURN_ON_ERROR(esm_drv_encoder_init(), "driver", "encoder init failed");

    /* Non-critical modules are initialized best-effort in this stage. */
    (void)esm_drv_can_init();
    return ESP_OK;
}

