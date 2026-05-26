#include "bsp/encoder/bsp_encoder.h"

#include "bsp/board/board.h"
#include "driver/spi/driver_spi.h"
#include "esp_log.h"

#define ESM_BSP_ENCODER_TWO_PI    6.28318530718f
#define ESM_BSP_ENCODER_REG_ANGLE 0x3FFFU

static const char *TAG = "bsp_encoder";
static float s_zero_offset_rad = 0.0f;
static bool s_verbose = false;

static uint16_t esm_bsp_encoder_apply_even_parity(uint16_t word15)
{
    uint16_t w = (uint16_t)(word15 & 0x7FFFU);
    uint16_t x = w;
    uint8_t ones = 0U;

    while (x != 0U) {
        ones = (uint8_t)(ones + (x & 1U));
        x >>= 1U;
    }
    if ((ones & 1U) != 0U) {
        w |= 0x8000U;
    }
    return w;
}

static bool esm_bsp_encoder_even_parity_ok(uint16_t frame)
{
    uint16_t x = frame;
    uint8_t ones = 0U;

    while (x != 0U) {
        ones = (uint8_t)(ones + (x & 1U));
        x >>= 1U;
    }
    return ((ones & 1U) == 0U);
}

esp_err_t esm_bsp_encoder_init(void)
{
    const esm_bsp_encoder_cfg_t *cfg = esm_bsp_board_get_encoder_cfg();

    if (cfg == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    if (esm_drv_spi_init(cfg->spi_host,
                         cfg->mosi_pin,
                         cfg->miso_pin,
                         cfg->sclk_pin,
                         cfg->cs_pin,
                         cfg->clock_hz,
                         cfg->spi_mode) != ESP_OK) {
        return ESP_FAIL;
    }

    s_zero_offset_rad = 0.0f;
    return ESP_OK;
}

esp_err_t esm_bsp_encoder_read_angle_rad(float *angle_rad)
{
    uint16_t cmd_read_angle;
    uint16_t cmd_nop;
    uint16_t resp = 0U;
    uint16_t raw;
    float angle;

    if (angle_rad == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cmd_read_angle = esm_bsp_encoder_apply_even_parity((uint16_t)(0x4000U | ESM_BSP_ENCODER_REG_ANGLE));
    cmd_nop = esm_bsp_encoder_apply_even_parity(0x0000U);

    if (s_verbose) {
        ESP_LOGI(TAG, "read_angle begin");
    }

    if (esm_drv_spi_xfer16(cmd_read_angle, &resp) != ESP_OK) {
        if (s_verbose) {
            ESP_LOGE(TAG, "read_angle read-cmd xfer failed");
        }
        return ESP_FAIL;
    }
    if (esm_drv_spi_xfer16(cmd_nop, &resp) != ESP_OK) {
        if (s_verbose) {
            ESP_LOGE(TAG, "read_angle nop xfer failed");
        }
        return ESP_FAIL;
    }

    if (!esm_bsp_encoder_even_parity_ok(resp)) {
        if (s_verbose) {
            ESP_LOGE(TAG, "read_angle parity fail resp=0x%04x", (unsigned)resp);
        }
        return ESP_FAIL;
    }
    if ((resp & 0x4000U) != 0U) {
        if (s_verbose) {
            ESP_LOGE(TAG, "read_angle error flag set resp=0x%04x", (unsigned)resp);
        }
        return ESP_FAIL;
    }

    raw = (uint16_t)(resp & 0x3FFFU);
    angle = ((float)raw / 16384.0f) * ESM_BSP_ENCODER_TWO_PI;
    angle -= s_zero_offset_rad;

    while (angle >= ESM_BSP_ENCODER_TWO_PI) {
        angle -= ESM_BSP_ENCODER_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += ESM_BSP_ENCODER_TWO_PI;
    }

    *angle_rad = angle;
    return ESP_OK;
}

esp_err_t esm_bsp_encoder_set_zero_offset_rad(float zero_offset_rad)
{
    float wrapped = zero_offset_rad;

    while (wrapped >= ESM_BSP_ENCODER_TWO_PI) {
        wrapped -= ESM_BSP_ENCODER_TWO_PI;
    }
    while (wrapped < 0.0f) {
        wrapped += ESM_BSP_ENCODER_TWO_PI;
    }
    s_zero_offset_rad = wrapped;
    return ESP_OK;
}

void esm_bsp_encoder_set_verbose(bool v)
{
    s_verbose = v;
}
