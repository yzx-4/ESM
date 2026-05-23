#include "driver/spi/driver_spi.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "drv_spi";

static spi_device_handle_t s_spi = NULL;
static spi_host_device_t s_host = SPI2_HOST;
static int s_cs_pin = -1;

esp_err_t esm_drv_spi_init(uint8_t spi_host, int mosi_pin, int miso_pin, int sclk_pin, int cs_pin, uint32_t clock_hz, uint8_t spi_mode)
{
    spi_bus_config_t bus_cfg = {0};
    spi_device_interface_config_t dev_cfg = {0};
    esp_err_t ret;

    if (s_spi != NULL) {
        return ESP_OK;
    }

    if (spi_host == 2U) {
        s_host = SPI2_HOST;
    } else {
        ESP_LOGE(TAG, "unsupported spi host id=%u", (unsigned)spi_host);
        return ESP_ERR_INVALID_ARG;
    }

    bus_cfg.mosi_io_num = mosi_pin;
    bus_cfg.miso_io_num = miso_pin;
    bus_cfg.sclk_io_num = sclk_pin;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4;

    dev_cfg.clock_speed_hz = (int)clock_hz;
    dev_cfg.mode = (int)spi_mode;
    dev_cfg.spics_io_num = -1;
    dev_cfg.queue_size = 1;
    dev_cfg.flags = 0;

    ret = spi_bus_initialize(s_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = spi_bus_add_device(s_host, &dev_cfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        (void)spi_bus_free(s_host);
        return ret;
    }

    s_cs_pin = cs_pin;
    gpio_reset_pin(s_cs_pin);
    gpio_set_direction(s_cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_cs_pin, 1);

    ESP_LOGI(TAG, "init host=%u mosi=%d miso=%d sclk=%d cs=%d mode=%u clk=%u",
             (unsigned)spi_host, (int)mosi_pin, (int)miso_pin, (int)sclk_pin, (int)cs_pin, (unsigned)spi_mode, (unsigned)clock_hz);
    return ESP_OK;
}

esp_err_t esm_drv_spi_xfer16(uint16_t tx_word, uint16_t *rx_word)
{
    static spi_transaction_t t = {0};
    esp_err_t ret;

    if (rx_word == NULL || s_spi == NULL || s_cs_pin < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 16;
    t.rxlength = 16;
    t.tx_data[0] = (uint8_t)((tx_word >> 8U) & 0xFFU);
    t.tx_data[1] = (uint8_t)(tx_word & 0xFFU);

    gpio_set_level(s_cs_pin, 0);
    ret = spi_device_transmit(s_spi, &t);
    gpio_set_level(s_cs_pin, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    *rx_word = (uint16_t)(((uint16_t)t.rx_data[0] << 8U) | (uint16_t)t.rx_data[1]);
    return ESP_OK;
}
