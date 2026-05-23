/* Two-phase analog sensing: read V/W phase currents and derive the third phase by KCL. */
#include "driver/current_sensor/analog.h"

#include <stdbool.h>
#include <stddef.h>

#include "bsp/board/board.h"
#include "bsp/bsp_interface.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_etm.h"
#include "config/foc_config.h"
#include "driver/mcpwm/driver_mcpwm.h"
#include "driver/mcpwm_etm.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"
#include "driver/current_sensor/adc_etm_ll.h"

#define ESM_CURRENT_SENSOR_INSTANCE  0
#define ESM_ANALOG_FAST_PHASE_COUNT   2U

static const char *TAG = "esm_analog";

static adc_continuous_handle_t s_adc_cont = NULL;
static uint8_t s_fast_phase_index[ESM_ANALOG_FAST_PHASE_COUNT] = {0};//快采样通道索引
static uint8_t s_fast_adc_channel[ESM_ANALOG_FAST_PHASE_COUNT] = {0};//快采样ADC通道号
static bool s_etm_trigger_ready = false;                             //ETM触发配置完成并且FOC控制任务注册了ADC采样回调后才会被置为true，表示可以通过ETM触发ADC采样了
static esp_etm_channel_handle_t s_etm_chan = NULL;                   //ETM通道句柄
static esp_etm_event_handle_t s_mcpwm_etm_event = NULL;              //MCPWM事件句柄
static esp_etm_task_handle_t s_adc_etm_task = NULL;                  //ADC ETM任务句柄
static volatile uint16_t s_latest_raw_ia = 0U;                       //最新电流12位数据
static volatile uint16_t s_latest_raw_ib = 0U;
static volatile bool s_latest_sample_valid = false;                  //最新采样数据是否有效，只有当s_latest_raw_ia和s_latest_raw_ib都被更新过了才会被置为true
/*连续ADC转换完成事件回调函数，FOC控制任务注册这个回调后，每当连续ADC转换完成一次，
FOC控制任务就会被通知一次，回调函数会从事件数据中解析出电流采样的原始值，并更新全局
变量s_latest_raw_ia和s_latest_raw_ib，以及标志位s_latest_sample_valid*/
static bool esm_drv_analog_on_continuous_conv_done(adc_continuous_handle_t handle,
                                                   const adc_continuous_evt_data_t *edata,
                                                   void *user_data)
{
    const adc_digi_output_data_t *samples;
    uint16_t raw_by_phase[ESM_BSP_ANALOG_FAST_PHASE_COUNT] = {0};
    bool got_by_phase[ESM_BSP_ANALOG_FAST_PHASE_COUNT] = {false};
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
        for (size_t phase = 0; phase < ESM_ANALOG_FAST_PHASE_COUNT; phase++) {
            if (channel == s_fast_adc_channel[phase]) {
                raw_by_phase[s_fast_phase_index[phase]] = data;
                got_by_phase[s_fast_phase_index[phase]] = true;
                break;
            }
        }
    }

    if (got_by_phase[0] && got_by_phase[1]) {
        s_latest_raw_ia = raw_by_phase[0];
        s_latest_raw_ib = raw_by_phase[1];
        s_latest_sample_valid = true;
    }

    return false;
}
/* 初始化函数，配置连续ADC和ETM触发，成功返回BSP_OK */
static bsp_status_t esm_drv_analog_ops_init(const esm_bsp_current_sense_cfg_t *cfg)
{
    esp_err_t err;
    const esm_foc_config_t *foc_cfg;
    adc_continuous_handle_cfg_t handle_cfg;
    adc_continuous_evt_cbs_t evt_cbs = {
        .on_conv_done = esm_drv_analog_on_continuous_conv_done,//注册到硬件，当连续ADC转换完成时会调用这个回调函数
        .on_pool_ovf = NULL,
    };

    if (cfg == NULL) {
        ESP_LOGE(TAG, "ops_init invalid arg: cfg is NULL");
        return BSP_ERR_INVALID_ARG;
    }
    foc_cfg = esm_cfg_foc_get();
    if (foc_cfg == NULL || !foc_cfg->current_sense.use_adc_continuous) {
        ESP_LOGE(TAG, "continuous ADC mode must be enabled by config");
        return BSP_ERR_FAIL;
    }
    if (cfg->fast_adc_unit[0] != 1U || cfg->fast_adc_unit[1] != 1U) {
        ESP_LOGE(TAG, "fast ADC must use ADC1 only");
        return BSP_ERR_INVALID_ARG;
    }
    if (cfg->fast_adc_channel[0] >= 10U || cfg->fast_adc_channel[1] >= 10U) {
        ESP_LOGE(TAG, "fast ADC channel out of range: ch0=%u ch1=%u",
                 (unsigned)cfg->fast_adc_channel[0],
                 (unsigned)cfg->fast_adc_channel[1]);
        return BSP_ERR_INVALID_ARG;
    }
    if (cfg->slow_channel_count != 0U) {
        ESP_LOGW(TAG, "slow ADC disabled on C5, ignoring configured slow channels");
    }
    if (cfg->etm_trigger_phase >= ESM_BSP_ANALOG_FAST_PHASE_COUNT) {
        ESP_LOGE(TAG, "ETM trigger phase out of range: %u", (unsigned)cfg->etm_trigger_phase);
        return BSP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < ESM_BSP_ANALOG_FAST_PHASE_COUNT; i++) {
        if (cfg->fast_phase_index[i] >= ESM_BSP_ANALOG_FAST_PHASE_COUNT) {
            ESP_LOGE(TAG, "fast phase index out of range: %u", (unsigned)cfg->fast_phase_index[i]);
            return BSP_ERR_INVALID_ARG;
        }
    }

    s_fast_phase_index[0] = cfg->fast_phase_index[0];
    s_fast_phase_index[1] = cfg->fast_phase_index[1];
    s_fast_adc_channel[0] = cfg->fast_adc_channel[0];
    s_fast_adc_channel[1] = cfg->fast_adc_channel[1];

    if (s_adc_cont == NULL) {
        handle_cfg.max_store_buf_size = 2048;
        handle_cfg.conv_frame_size = ESM_ANALOG_FAST_PHASE_COUNT * SOC_ADC_DIGI_DATA_BYTES_PER_CONV;
        err = adc_continuous_new_handle(&handle_cfg, &s_adc_cont);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_continuous_new_handle failed: %s", esp_err_to_name(err));
            return BSP_ERR_FAIL;
        }
    }

    {
        adc_digi_pattern_config_t adc_pattern[ESM_ANALOG_FAST_PHASE_COUNT] = {0};
        for (int i = 0; i < ESM_ANALOG_FAST_PHASE_COUNT; i++) {
            adc_pattern[i].atten = ADC_ATTEN_DB_12;
            adc_pattern[i].channel = (adc_channel_t)s_fast_adc_channel[i];
            adc_pattern[i].unit = ADC_UNIT_1;
            adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
        }

        adc_continuous_config_t cont_cfg = {
            .pattern_num = ESM_ANALOG_FAST_PHASE_COUNT,
            .adc_pattern = adc_pattern,
            .sample_freq_hz = foc_cfg->control.current_loop_hz,
            .conv_mode = ADC_CONV_SINGLE_UNIT_1,
            .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        };
        err = adc_continuous_config(s_adc_cont, &cont_cfg);//idf的连续触发初始化函数
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_continuous_config failed: %s", esp_err_to_name(err));
            return BSP_ERR_FAIL;
        }
    }

    err = adc_continuous_register_event_callbacks(s_adc_cont, &evt_cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_register_event_callbacks failed: %s", esp_err_to_name(err));
        return BSP_ERR_FAIL;
    }

    return BSP_OK;
}

static bsp_status_t esm_drv_analog_ops_trigger_fast_sample(esm_bsp_phase_current_t *current)//这只是检查最新的采样数据是否有效，并把它保存到current指向的结构体里，真正的触发是ETM触发ADC采样
{
    if (current == NULL) {
        return BSP_ERR_INVALID_ARG;
    }

    if (!s_latest_sample_valid) {
        return BSP_ERR_FAIL;
    }

    current->raw_ia = s_latest_raw_ia;
    current->raw_ib = s_latest_raw_ib;
    current->raw_ic = 0U;
    s_latest_sample_valid = false;
    return BSP_OK;
}

static bsp_status_t esm_drv_analog_ops_read_slow_raw(esm_bsp_analog_slow_channel_id_t slow_channel_id, uint16_t *raw)//阻塞式oneshot读取，C5不用,占位
{
    (void)slow_channel_id;
    (void)raw;
    return BSP_ERR_FAIL;
}

esp_err_t esm_drv_analog_init(void)//初始化读电流需要的资源，注册到上层
{
    bsp_status_t bsp_ret;
    static const esm_bsp_current_sense_ops_t ops = {
        .init = esm_drv_analog_ops_init,
        .trigger_fast_sample = esm_drv_analog_ops_trigger_fast_sample,
        .read_slow_raw = esm_drv_analog_ops_read_slow_raw,
    };
    const esm_bsp_current_sense_cfg_t *cfg_hw = esm_bsp_board_get_current_sense_cfg(ESM_CURRENT_SENSOR_INSTANCE);

    if (cfg_hw == NULL) {
        ESP_LOGE(TAG, "board current sense cfg not found");
        return ESP_ERR_NOT_FOUND;
    }
    bsp_ret = esm_bsp_current_sense_register(ESM_CURRENT_SENSOR_INSTANCE, &ops, cfg_hw);
    if (bsp_ret != BSP_OK) {
        ESP_LOGE(TAG, "bsp current sense register failed: %d", (int)bsp_ret);
        return ESP_FAIL;
    }
    if (ops.init != NULL) {
        bsp_ret = ops.init(cfg_hw);
        if (bsp_ret != BSP_OK) {
            ESP_LOGE(TAG, "current sense ops.init failed: %d", (int)bsp_ret);
            return ESP_FAIL;
        }
    }

    if (esm_cfg_foc_get() == NULL || !esm_cfg_foc_get()->current_sense.enable_etm_trigger) {
        ESP_LOGE(TAG, "ETM trigger must be enabled for continuous ADC path");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esm_drv_analog_setup_etm_trigger();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ETM trigger setup failed: %s", esp_err_to_name(err));
        return err;
    }

    err = adc_continuous_start(s_adc_cont);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_continuous_start failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "analog init ok");
    return ESP_OK;
}

esp_err_t esm_drv_analog_setup_etm_trigger(void)//配置ETM触发，成功返回ESP_OK
{
#if CONFIG_IDF_TARGET_ESP32C5
    esp_err_t err;
    mcpwm_cmpr_handle_t cmpr = NULL;

    if (s_etm_trigger_ready) {
        return ESP_OK;
    }

    /* 固定使用相位 U 比较器作为中心点触发源。 */
    const esm_bsp_current_sense_cfg_t *cfg_hw = esm_bsp_board_get_current_sense_cfg(ESM_CURRENT_SENSOR_INSTANCE);
    if (cfg_hw == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    err = esm_drv_mcpwm_get_comparator_handle(cfg_hw->etm_trigger_phase, &cmpr);
    if (err != ESP_OK || cmpr == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    mcpwm_cmpr_etm_event_config_t etm_event_cfg = {
        .event_type = MCPWM_CMPR_ETM_EVENT_EQUAL,
    };
    err = mcpwm_comparator_new_etm_event(cmpr, &etm_event_cfg, &s_mcpwm_etm_event);
    if (err != ESP_OK) {
        return err;
    }

    esp_etm_channel_config_t etm_chan_cfg = {
        .flags = {
            .allow_pd = 0,
        },
    };
    err = esp_etm_new_channel(&etm_chan_cfg, &s_etm_chan);
    if (err != ESP_OK) {
        return err;
    }
    err = esm_drv_adc_etm_new_start_task(&s_adc_etm_task);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_etm_channel_connect(s_etm_chan, s_mcpwm_etm_event, s_adc_etm_task);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_etm_channel_enable(s_etm_chan);
    if (err != ESP_OK) {
        return err;
    }

    s_etm_trigger_ready = true;
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
