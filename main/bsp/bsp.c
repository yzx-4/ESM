#include "bsp/bsp.h"

#include "bsp/board/board.h"
#include "bsp/clock/bsp_clock.h"
#include "bsp/gpio/bsp_gpio.h"
#include "bsp/nvs/bsp_nvs.h"
#include "bsp/spi_encoder/bsp_spi_encoder.h"
#include "bsp/timer/bsp_timer.h"
#include "bsp/uart/bsp_uart.h"

bsp_status_t esm_bsp_init(void)
{
    if (esm_bsp_board_init() != BSP_OK) {
        return BSP_ERR_FAIL;
    }
    if (esm_bsp_clock_init() != BSP_OK) {
        return BSP_ERR_FAIL;
    }
    if (esm_bsp_gpio_init() != BSP_OK) {
        return BSP_ERR_FAIL;
    }
    if (esm_bsp_timer_init() != BSP_OK) {
        return BSP_ERR_FAIL;
    }
    if (esm_bsp_uart_init() != BSP_OK) {
        return BSP_ERR_FAIL;
    }
    if (esm_bsp_nvs_init() != BSP_OK) {
        return BSP_ERR_FAIL;
    }
    if (esm_bsp_spi_encoder_init() != BSP_OK) {
        return BSP_ERR_FAIL;
    }
    return BSP_OK;
}



