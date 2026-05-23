#pragma once

#include <stdint.h>
#include "esp_err.h"

// 初始化 SPI 总线并添加设备（仅在第一次调用时初始化）
esp_err_t esm_drv_spi_init(uint8_t spi_host, int mosi_pin, int miso_pin, int sclk_pin, int cs_pin, uint32_t clock_hz, uint8_t spi_mode);

// 发送 16 位数据并接收 16 位响应（blocking）
esp_err_t esm_drv_spi_xfer16(uint16_t tx_word, uint16_t *rx_word);
