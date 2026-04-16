/* Three-shunt current sensing: read phase voltages, calibrate offsets, output currents. */
#include "driver/current_sensor/driver_current_sensor.h"

#include <stdbool.h>

#include "bsp/board/board.h"
#include "bsp/bsp_interface.h"
#include "config/foc_config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"

#define ESM_CURRENT_SENSOR_INSTANCE  0

static float s_phase_offset_v[3] = {1.65f, 1.65f, 1.65f};
static bool s_is_calibrated = false;
static const esm_bsp_current_sense_cfg_t *s_cfg_hw = NULL;
static adc_oneshot_unit_handle_t s_adc1 = NULL;
static adc_oneshot_unit_handle_t s_adc2 = NULL;
static bool s_adc1_ch_cfg[10] = {0};
static bool s_adc2_ch_cfg[10] = {0};

static adc_oneshot_unit_handle_t esm_current_sensor_get_adc_handle(uint8_t adc_unit)
{
    if (adc_unit == 1U) {
        return s_adc1;
    }
    if (adc_unit == 2U) {
        return s_adc2;
    }
    return NULL;
}

static bool *esm_current_sensor_get_cfg_bitmap(uint8_t adc_unit)
{
    if (adc_unit == 1U) {
        return s_adc1_ch_cfg;
    }
    if (adc_unit == 2U) {
        return s_adc2_ch_cfg;
    }
    return NULL;
}

static esp_err_t esm_current_sensor_adc_read_voltage(uint8_t adc_unit, uint8_t adc_channel, float *voltage_v)
{
    adc_oneshot_unit_handle_t handle = esm_current_sensor_get_adc_handle(adc_unit);
    bool *cfg_bitmap = esm_current_sensor_get_cfg_bitmap(adc_unit);
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    int raw = 0;

    if (voltage_v == NULL || handle == NULL || cfg_bitmap == NULL || adc_channel >= 10U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!cfg_bitmap[adc_channel]) {
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(handle, (adc_channel_t)adc_channel, &chan_cfg),
                            "current_sensor",
                            "adc config channel failed");
        cfg_bitmap[adc_channel] = true;
    }

    ESP_RETURN_ON_ERROR(adc_oneshot_read(handle, (adc_channel_t)adc_channel, &raw),
                        "current_sensor",
                        "adc oneshot read failed");
    *voltage_v = (3.3f * (float)raw) / 4095.0f;
    return ESP_OK;
}

static esp_err_t esm_current_sensor_read_phase_voltage_raw(float phase_v[3])
{
    if (phase_v == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_cfg_hw == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(esm_current_sensor_adc_read_voltage(s_cfg_hw->adc_unit[0], s_cfg_hw->adc_channel[0], &phase_v[0]),
                        "current_sensor",
                        "read phase U voltage failed");
    ESP_RETURN_ON_ERROR(esm_current_sensor_adc_read_voltage(s_cfg_hw->adc_unit[1], s_cfg_hw->adc_channel[1], &phase_v[1]),
                        "current_sensor",
                        "read phase V voltage failed");
    ESP_RETURN_ON_ERROR(esm_current_sensor_adc_read_voltage(s_cfg_hw->adc_unit[2], s_cfg_hw->adc_channel[2], &phase_v[2]),
                        "current_sensor",
                        "read phase W voltage failed");
    return ESP_OK;
}

static bsp_status_t esm_drv_current_sensor_ops_init(const esm_bsp_current_sense_cfg_t *cfg)
{
    adc_oneshot_unit_init_cfg_t cfg1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_unit_init_cfg_t cfg2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    s_cfg_hw = cfg;

    if (cfg == NULL) {
        return BSP_ERR_INVALID_ARG;
    }
    if (s_adc1 == NULL && adc_oneshot_new_unit(&cfg1, &s_adc1) != ESP_OK) {
        return BSP_ERR_FAIL;
    }
    if (s_adc2 == NULL && adc_oneshot_new_unit(&cfg2, &s_adc2) != ESP_OK) {
        return BSP_ERR_FAIL;
    }
    return BSP_OK;
}

static bsp_status_t esm_drv_current_sensor_ops_read_phase_voltage(float phase_v[3])
{
    return esm_current_sensor_read_phase_voltage_raw(phase_v) == ESP_OK ? BSP_OK : BSP_ERR_FAIL;
}

static bsp_status_t esm_drv_current_sensor_ops_calibrate_zero(void)
{
    return esm_drv_current_sensor_calibrate_zero() == ESP_OK ? BSP_OK : BSP_ERR_FAIL;
}

static bsp_status_t esm_drv_current_sensor_ops_read_phase_current(esm_bsp_phase_current_t *current)
{
    esm_phase_current_t drv_current;

    if (current == NULL) {
        return BSP_ERR_INVALID_ARG;
    }
    if (esm_drv_current_sensor_get_phase_current(&drv_current) != ESP_OK) {
        return BSP_ERR_FAIL;
    }
    current->ia = drv_current.ia;
    current->ib = drv_current.ib;
    current->ic = drv_current.ic;
    return BSP_OK;
}

static bsp_status_t esm_drv_current_sensor_ops_read_bus_voltage(float *bus_v)
{
    float adc_v = 0.0f;

    if (bus_v == NULL) {
        return BSP_ERR_INVALID_ARG;
    }
    if (s_cfg_hw == NULL || s_cfg_hw->bus_div_ratio <= 0.0f) {
        return BSP_ERR_FAIL;
    }
    if (esm_current_sensor_adc_read_voltage(s_cfg_hw->bus_voltage_adc_unit, s_cfg_hw->bus_voltage_adc_channel, &adc_v) != ESP_OK) {
        return BSP_ERR_FAIL;
    }

    *bus_v = adc_v / s_cfg_hw->bus_div_ratio;
    return BSP_OK;
}

esp_err_t esm_drv_current_sensor_init(void)
{
    static const esm_bsp_current_sense_ops_t ops = {
        .init = esm_drv_current_sensor_ops_init,
        .read_phase_voltage = esm_drv_current_sensor_ops_read_phase_voltage,
        .calibrate_zero = esm_drv_current_sensor_ops_calibrate_zero,
        .read_phase_current = esm_drv_current_sensor_ops_read_phase_current,
        .read_bus_voltage = esm_drv_current_sensor_ops_read_bus_voltage,
    };
    const esm_bsp_current_sense_cfg_t *cfg_hw = esm_bsp_board_get_current_sense_cfg(ESM_CURRENT_SENSOR_INSTANCE);

    if (cfg_hw == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    if (esm_bsp_current_sense_register(ESM_CURRENT_SENSOR_INSTANCE, &ops, cfg_hw) != BSP_OK) {
        return ESP_FAIL;
    }
    if (ops.init != NULL && ops.init(cfg_hw) != BSP_OK) {
        return ESP_FAIL;
    }

    const esm_foc_config_t *cfg = esm_cfg_foc_get();
    s_phase_offset_v[0] = cfg->current_sense.phase_offset_v[0];
    s_phase_offset_v[1] = cfg->current_sense.phase_offset_v[1];
    s_phase_offset_v[2] = cfg->current_sense.phase_offset_v[2];
    return ESP_OK;
}

esp_err_t esm_drv_current_sensor_calibrate_zero(void)
{
    float phase_v[3] = {0.0f, 0.0f, 0.0f};
    ESP_RETURN_ON_ERROR(esm_current_sensor_read_phase_voltage_raw(phase_v), "current_sensor", "read phase voltage failed");

    s_phase_offset_v[0] = phase_v[0];
    s_phase_offset_v[1] = phase_v[1];
    s_phase_offset_v[2] = phase_v[2];
    s_is_calibrated = true;
    return esm_cfg_foc_set_phase_offset_v(s_phase_offset_v);
}

esp_err_t esm_drv_current_sensor_get_phase_current(esm_phase_current_t *current)
{
    float phase_v[3] = {0.0f, 0.0f, 0.0f};
    const esm_foc_config_t *cfg = esm_cfg_foc_get();

    if (current == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(esm_current_sensor_read_phase_voltage_raw(phase_v), "current_sensor", "read phase voltage failed");

    if (!s_is_calibrated) {
        return ESP_ERR_INVALID_STATE;
    }

    current->ia = (phase_v[0] - s_phase_offset_v[0]) / cfg->current_sense.phase_gain_v_per_a;
    current->ib = (phase_v[1] - s_phase_offset_v[1]) / cfg->current_sense.phase_gain_v_per_a;
    current->ic = (phase_v[2] - s_phase_offset_v[2]) / cfg->current_sense.phase_gain_v_per_a;
    return ESP_OK;
}

