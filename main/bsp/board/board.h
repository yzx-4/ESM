#pragma once

#include <stdint.h>

#include "bsp/bsp.h"
#include "bsp/bsp_hal.h"


bsp_status_t esm_bsp_board_init(void);

const esm_bsp_pwm_cfg_t *esm_bsp_board_get_pwm_cfg(void);
const esm_bsp_current_sense_cfg_t *esm_bsp_board_get_current_sense_cfg(void);
const esm_bsp_encoder_cfg_t *esm_bsp_board_get_encoder_cfg(void);


