/* MCPWM driver interface: PWM output apply, period callback, and ISR stats access. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/mcpwm_types.h"

typedef void (*esm_drv_mcpwm_period_cb_t)(void *user_ctx);
typedef void (*esm_drv_mcpwm_adc_trigger_cb_t)(void *user_ctx);

typedef struct {
	uint32_t isr_count;
	uint32_t period_cb_count;   // 周期性callback成功调用的次数
	uint32_t period_cb_drop_count;
} esm_drv_mcpwm_isr_stats_t;

esp_err_t esm_drv_mcpwm_init(void);
esp_err_t esm_drv_mcpwm_apply_duty(float duty_a, float duty_b, float duty_c);
esp_err_t esm_drv_mcpwm_register_period_callback(esm_drv_mcpwm_period_cb_t cb, void *user_ctx);
/* ADC 触发固定为 PWM 中心点（timer full）事件。 */
esp_err_t esm_drv_mcpwm_register_adc_center_trigger_callback(esm_drv_mcpwm_adc_trigger_cb_t cb, void *user_ctx);
void esm_drv_mcpwm_get_isr_stats(esm_drv_mcpwm_isr_stats_t *stats);// 获取ISR统计数据
void esm_drv_mcpwm_reset_isr_stats(void);// 重置ISR统计数据

/* 获取指定相位的比较器句柄，用于创建 ETM 事件或其他底层操作 */
esp_err_t esm_drv_mcpwm_get_comparator_handle(uint8_t phase, mcpwm_cmpr_handle_t *out_cmpr);

