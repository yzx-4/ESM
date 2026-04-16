/* Minimal AS5047P driver: SPI init and angle read (0~2pi). */
#include "driver/encoder/driver_encoder.h"

#include "bsp/board/board.h"
#include "bsp/bsp_interface.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#define ESM_ENCODER_INSTANCE  0
#define ESM_TWO_PI            6.28318530718f

static const char *TAG = "encoder_as5047p";

static spi_device_handle_t s_spi = NULL;
static spi_host_device_t s_host = SPI2_HOST;
static int s_cs_pin = -1;
static float s_zero_offset_rad = 0.0f;

static bsp_status_t s_as5047p_init(const esm_bsp_encoder_cfg_t *cfg)
{
    spi_bus_config_t bus_cfg = {0};
    spi_device_interface_config_t dev_cfg = {0};
    esp_err_t ret;

    if (cfg == NULL) {
        return BSP_ERR_INVALID_ARG;
    }
    if (s_spi != NULL) {
        return BSP_OK;
    }

    if (cfg->spi_host == 2U) {
        s_host = SPI2_HOST;
    } else if (cfg->spi_host == 3U) {
        s_host = SPI3_HOST;
    } else {
        ESP_LOGE(TAG, "unsupported spi host id=%u", (unsigned)cfg->spi_host);
        return BSP_ERR_INVALID_ARG;
    }

    bus_cfg.mosi_io_num = cfg->mosi_pin;
    bus_cfg.miso_io_num = cfg->miso_pin;
    bus_cfg.sclk_io_num = cfg->sclk_pin;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4;

    dev_cfg.clock_speed_hz = (int)cfg->clock_hz;
    dev_cfg.mode = (int)cfg->spi_mode;
    dev_cfg.spics_io_num = -1;
    dev_cfg.queue_size = 1;
    dev_cfg.flags = 0;

    ret = spi_bus_initialize(s_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return BSP_ERR_FAIL;
    }

    ret = spi_bus_add_device(s_host, &dev_cfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        (void)spi_bus_free(s_host);
        return BSP_ERR_FAIL;
    }

    s_cs_pin = cfg->cs_pin;
    gpio_reset_pin(s_cs_pin);
    gpio_set_direction(s_cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_cs_pin, 1);

    ESP_LOGI(TAG,
             "init host=%u mosi=%d miso=%d sclk=%d cs=%d mode=%u clk=%u",
             (unsigned)cfg->spi_host,
             (int)cfg->mosi_pin,
             (int)cfg->miso_pin,
             (int)cfg->sclk_pin,
             (int)cfg->cs_pin,
             (unsigned)cfg->spi_mode,
             (unsigned)cfg->clock_hz);
    return BSP_OK;
}

static bsp_status_t s_as5047p_read_angle_rad(float *angle_rad)
{
    uint8_t tx[2] = {0xFF, 0xFF};
    uint8_t rx[2] = {0, 0};
    spi_transaction_t t = {0};
    uint16_t raw;
    float angle;

    if (angle_rad == NULL) {
        return BSP_ERR_INVALID_ARG;
    }
    if (s_spi == NULL || s_cs_pin < 0) {
        return BSP_ERR_FAIL;
    }

    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    gpio_set_level(s_cs_pin, 0);
    if (spi_device_transmit(s_spi, &t) != ESP_OK) {
        gpio_set_level(s_cs_pin, 1);
        return BSP_ERR_FAIL;
    }
    gpio_set_level(s_cs_pin, 1);

    raw = (uint16_t)((((uint16_t)rx[0] << 8U) | (uint16_t)rx[1]) & 0x3FFFU);
    angle = ((float)raw / 16383.0f) * ESM_TWO_PI;
    angle -= s_zero_offset_rad;

    while (angle >= ESM_TWO_PI) {
        angle -= ESM_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += ESM_TWO_PI;
    }

    *angle_rad = angle;
    return BSP_OK;
}

static bsp_status_t s_as5047p_set_zero_offset_rad(float zero_offset_rad)
{
    float wrapped = zero_offset_rad;

    while (wrapped >= ESM_TWO_PI) {
        wrapped -= ESM_TWO_PI;
    }
    while (wrapped < 0.0f) {
        wrapped += ESM_TWO_PI;
    }
    s_zero_offset_rad = wrapped;
    return BSP_OK;
}

esp_err_t esm_drv_encoder_init(void)
{
    static const esm_bsp_encoder_ops_t ops = {
        .init = s_as5047p_init,
        .read_angle_rad = s_as5047p_read_angle_rad,
        .set_zero_offset_rad = s_as5047p_set_zero_offset_rad,
    };
    const esm_bsp_encoder_cfg_t *cfg = esm_bsp_board_get_encoder_cfg(ESM_ENCODER_INSTANCE);

    if (cfg == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    if (esm_bsp_encoder_register(ESM_ENCODER_INSTANCE, &ops, cfg) != BSP_OK) {
        return ESP_FAIL;
    }
    if (ops.init(cfg) != BSP_OK) {
        return ESP_FAIL;
    }

    s_zero_offset_rad = 0.0f;
    return ESP_OK;
}
//包装给上层，之后记得开编译优化-O2
esp_err_t esm_drv_encoder_read_angle_rad(float *angle_rad)
{
    return s_as5047p_read_angle_rad(angle_rad) == BSP_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t esm_drv_encoder_set_zero_offset_rad(float zero_offset_rad)
{
    return s_as5047p_set_zero_offset_rad(zero_offset_rad) == BSP_OK ? ESP_OK : ESP_FAIL;
}