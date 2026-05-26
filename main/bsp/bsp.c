#include "bsp/bsp.h"

#include "bsp/board/board.h"
#include "bsp/bsp_hal.h"
#include "bsp/encoder/bsp_encoder.h"

bsp_status_t esm_bsp_init(void)
{
    if (esm_bsp_board_init() != BSP_OK) {
        return BSP_ERR_FAIL;
    }
    if (esm_bsp_pwm_init() != ESP_OK) {
        return BSP_ERR_FAIL;
    }
    if (esm_bsp_current_sense_init() != ESP_OK) {
        return BSP_ERR_FAIL;
    }
    if (esm_bsp_encoder_init() != ESP_OK) {
        return BSP_ERR_FAIL;
    }
    return BSP_OK;
}



