#pragma once

#include <stdint.h>

#include "bsp/bsp_interface.h"
#include "bsp/bsp_status.h"

bsp_status_t esm_bsp_spi_encoder_init(const esm_bsp_encoder_cfg_t *cfg);
bsp_status_t esm_bsp_spi_encoder_xfer16(uint16_t tx_word, uint16_t *rx_word);
#pragma once

#include "bsp/bsp_status.h"

bsp_status_t esm_bsp_spi_encoder_init(void);


