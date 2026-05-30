#include "bsp/current_sensor/bsp_current_sensor.h"

#include <stdbool.h>
#include <stddef.h>

#include "bsp/board/board.h"
#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

#define ESM_BSP_CURRENT_SENSOR_INSTANCE     0
#define ESM_BSP_CURRENT_SENSOR_FAST_PHASES  2U
#define ESM_BSP_CURRENT_SENSOR_AVG_FRAMES   4U

static const char *TAG = "bsp_current_sensor";

static adc_continuous_handle_t s_adc_cont = NULL;
static uint8_t s_fast_phase_index[ESM_BSP_CURRENT_SENSOR_FAST_PHASES] = {0};
static uint8_t s_fast_adc_channel[ESM_BSP_CURRENT_SENSOR_FAST_PHASES] = {0};
static volatile uint16_t s_latest_raw_ia = 0U;
static volatile uint16_t s_latest_raw_ib = 0U;
static volatile bool s_latest_sample_valid = false;
static volatile uint32_t s_frame_total = 0U;
static volatile uint32_t s_frame_last_read = 0U;
static uint32_t s_avg_raw_ia_sum = 0U;
static uint32_t s_avg_raw_ib_sum = 0U;
static uint32_t s_avg_frame_count = 0U;
static bool s_initialized = false;

static bool esm_bsp_current_sense_on_conv_done(adc_continuous_handle_t handle,
                                               const adc_continuous_evt_data_t *edata,
                                               void *user_data)
{
    const adc_digi_output_data_t *samples;
    uint16_t raw_by_phase[ESM_BSP_CURRENT_SENSOR_FAST_PHASES] = {0};
    bool got_by_phase[ESM_BSP_CURRENT_SENSOR_FAST_PHASES] = {false};
    size_t sample_count;

    (void)handle;
    (void)user_data;
    if (edata == NULL || edata->conv_frame_buffer == NULL || edata->size < SOC_ADC_DIGI_DATA_BYTES_PER_CONV) {
        return false;
    }

    samples = (const adc_digi_output_data_t *)edata->conv_frame_buffer;
    sample_count = edata->size / SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
    for (size_t i = 0; i < sample_count; i++) {
        uint32_t raw = samples[i].val;
        uint8_t channel = (raw >> 13) & 0x0F;
        uint16_t data = raw & 0x0FFF;
        for (size_t phase = 0; phase < ESM_BSP_CURRENT_SENSOR_FAST_PHASES; phase++) {
            if (channel == s_fast_adc_channel[phase]) {
                raw_by_phase[s_fast_phase_index[phase]] = data;
                got_by_phase[s_fast_phase_index[phase]] = true;
                break;
            }
        }
    }

    if (got_by_phase[0] && got_by_phase[1]) {
        s_avg_raw_ia_sum += raw_by_phase[0];
        s_avg_raw_ib_sum += raw_by_phase[1];
        s_avg_frame_count++;

        if (s_avg_frame_count >= ESM_BSP_CURRENT_SENSOR_AVG_FRAMES) {
            s_latest_raw_ia = (uint16_t)(s_avg_raw_ia_sum / ESM_BSP_CURRENT_SENSOR_AVG_FRAMES);
            s_latest_raw_ib = (uint16_t)(s_avg_raw_ib_sum / ESM_BSP_CURRENT_SENSOR_AVG_FRAMES);
            s_latest_sample_valid = true;
            s_avg_raw_ia_sum = 0U;
            s_avg_raw_ib_sum = 0U;
            s_avg_frame_count = 0U;
        }
    }

    s_frame_total++;
    return false;
}

esp_err_t esm_bsp_current_sense_init(void)
{
    const esm_bsp_current_sense_cfg_t *bcfg = esm_bsp_board_get_current_sense_cfg();
    adc_continuous_handle_cfg_t handle_cfg;
    adc_continuous_evt_cbs_t evt_cbs;
    adc_digi_pattern_config_t adc_pattern[ESM_BSP_CURRENT_SENSOR_FAST_PHASES] = {0};
    adc_continuous_config_t cont_cfg;

    if (bcfg == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    if (s_initialized) {
        return ESP_OK;
    }

    for (size_t i = 0; i < ESM_BSP_CURRENT_SENSOR_FAST_PHASES; i++) {
        if (bcfg->fast_adc_unit[i] != 1U) {
            ESP_LOGE(TAG, "fast ADC must use ADC1 only");
            return ESP_ERR_INVALID_ARG;
        }
        if (bcfg->fast_adc_channel[i] >= 10U) {
            ESP_LOGE(TAG, "fast ADC channel out of range: ch%u=%u", (unsigned)i, (unsigned)bcfg->fast_adc_channel[i]);
            return ESP_ERR_INVALID_ARG;
        }
        if (bcfg->fast_phase_index[i] >= ESM_BSP_CURRENT_SENSOR_FAST_PHASES) {
            ESP_LOGE(TAG, "fast phase index out of range: %u", (unsigned)bcfg->fast_phase_index[i]);
            return ESP_ERR_INVALID_ARG;
        }
        s_fast_phase_index[i] = bcfg->fast_phase_index[i];
        s_fast_adc_channel[i] = bcfg->fast_adc_channel[i];
    }

    if (bcfg->slow_channel_count != 0U) {
        ESP_LOGW(TAG, "slow ADC disabled on C5, ignoring configured slow channels");
    }

    handle_cfg.max_store_buf_size = 2048;
    handle_cfg.conv_frame_size = ESM_BSP_CURRENT_SENSOR_FAST_PHASES * SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
    handle_cfg.flags.flush_pool = 1;
    if (adc_continuous_new_handle(&handle_cfg, &s_adc_cont) != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_new_handle failed");
        return ESP_FAIL;
    }

    for (size_t i = 0; i < ESM_BSP_CURRENT_SENSOR_FAST_PHASES; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = (adc_channel_t)s_fast_adc_channel[i];
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }

    cont_cfg.pattern_num = ESM_BSP_CURRENT_SENSOR_FAST_PHASES;
    cont_cfg.adc_pattern = adc_pattern;
    cont_cfg.sample_freq_hz = 40000;
    cont_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    cont_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    if (adc_continuous_config(s_adc_cont, &cont_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_config failed");
        return ESP_FAIL;
    }

    evt_cbs.on_conv_done = esm_bsp_current_sense_on_conv_done;
    evt_cbs.on_pool_ovf = NULL;
    if (adc_continuous_register_event_callbacks(s_adc_cont, &evt_cbs, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_register_event_callbacks failed");
        return ESP_FAIL;
    }

    if (adc_continuous_start(s_adc_cont) != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_start failed");
        return ESP_FAIL;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t esm_bsp_current_sense_read_latest_sample(esm_bsp_phase_current_t *current,
                                                   uint32_t *conv_done_delta,
                                                   uint32_t *conv_done_total)
{
    if (current == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized || !s_latest_sample_valid) {
        return ESP_FAIL;
    }

    current->raw_ia = s_latest_raw_ia;
    current->raw_ib = s_latest_raw_ib;
    current->raw_ic = 0U;

    if (conv_done_delta != NULL || conv_done_total != NULL) {
        uint32_t total = s_frame_total;
        if (conv_done_delta != NULL) {
            *conv_done_delta = total - s_frame_last_read;
        }
        if (conv_done_total != NULL) {
            *conv_done_total = total;
        }
        s_frame_last_read = total;
    }
    return ESP_OK;
}

esp_err_t esm_bsp_current_sense_read_slow_raw(esm_bsp_analog_slow_channel_id_t slow_channel_id, uint16_t *raw)
{
    (void)slow_channel_id;
    (void)raw;
    return ESP_OK;
}
