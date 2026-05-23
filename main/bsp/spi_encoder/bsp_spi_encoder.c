// BSP thin wrapper: delegate SPI operations to driver layer
#include "bsp/spi_encoder/bsp_spi_encoder.h"
#include "driver/spi/driver_spi.h"

bsp_status_t esm_bsp_spi_encoder_init(const esm_bsp_encoder_cfg_t *cfg)
{
    if (cfg == NULL) {
        return BSP_ERR_INVALID_ARG;
    }
    return esm_drv_spi_init((uint8_t)cfg->spi_host, cfg->mosi_pin, cfg->miso_pin, cfg->sclk_pin, cfg->cs_pin, cfg->clock_hz, cfg->spi_mode) == ESP_OK ? BSP_OK : BSP_ERR_FAIL;
}

bsp_status_t esm_bsp_spi_encoder_xfer16(uint16_t tx_word, uint16_t *rx_word)
{
    return esm_drv_spi_xfer16(tx_word, rx_word) == ESP_OK ? BSP_OK : BSP_ERR_FAIL;
}
#include "bsp/spi_encoder/bsp_spi_encoder.h"

bsp_status_t esm_bsp_spi_encoder_init(void)
{
    return BSP_OK;
}


