#pragma once

#include "bsp/current_sensor/bsp_current_sensor.h"

typedef enum {
	BSP_OK = 0,
	BSP_ERR_FAIL = -1,
	BSP_ERR_INVALID_ARG = -2
} bsp_status_t;

bsp_status_t esm_bsp_init(void);


