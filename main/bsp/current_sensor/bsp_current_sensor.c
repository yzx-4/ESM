#include "bsp/current_sensor/bsp_current_sensor.h"

#include "bsp/board/board.h"
#include "bsp/bsp_require_drv_hal.h"
#include "driver/current_sensor/analog.h"

#include "esp_err.h"

esp_err_t esm_bsp_current_sense_init(void)
{
    const esm_bsp_current_sense_cfg_t *bcfg = esm_bsp_board_get_current_sense_cfg();
    if (bcfg == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    esm_drv_current_cfg_t dcfg = {0};
    for (int i = 0; i < 3; i++) {
        dcfg.fast_phase_index[i] = bcfg->fast_phase_index[i];
        dcfg.fast_adc_unit[i] = bcfg->fast_adc_unit[i];
        dcfg.fast_adc_channel[i] = bcfg->fast_adc_channel[i];
    }
    dcfg.etm_trigger_phase = bcfg->etm_trigger_phase;
    for (int i = 0; i < 4; i++) {
        dcfg.slow_adc_unit[i] = bcfg->slow_adc_unit[i];
        dcfg.slow_adc_channel[i] = bcfg->slow_adc_channel[i];
    }
    dcfg.slow_channel_count = bcfg->slow_channel_count;
    dcfg.bus_v_valid_min = bcfg->bus_v_valid_min;
    dcfg.bus_v_valid_max = bcfg->bus_v_valid_max;
    dcfg.bus_v_default = bcfg->bus_v_default;

    return esm_drv_analog_init(&dcfg);
}

esp_err_t esm_bsp_current_sense_read_latest_sample(esm_bsp_phase_current_t *current)
{
    if (current == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esm_drv_phase_current_t drv_cur;
    esp_err_t err = esm_drv_analog_read_latest_sample(&drv_cur);
    if (err != ESP_OK) {
        return err;
    }
    current->raw_ia = drv_cur.raw_ia;
    current->raw_ib = drv_cur.raw_ib;
    current->raw_ic = drv_cur.raw_ic;
    return ESP_OK;
}

esp_err_t esm_bsp_current_sense_read_slow_raw(esm_bsp_analog_slow_channel_id_t slow_channel_id, uint16_t *raw)
{
    return esm_drv_analog_read_slow_raw(slow_channel_id, raw);
}
