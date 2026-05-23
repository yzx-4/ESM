/* BSP abstraction runtime storage: register and provide cfg/ops instances. */
#include "bsp/bsp_interface.h"

#include <stdbool.h>
#include <string.h>

typedef struct {
    bool is_registered;
    esm_bsp_pwm_cfg_t cfg;
    esm_bsp_pwm_ops_t ops;
} esm_bsp_pwm_slot_t;

typedef struct {
    bool is_registered;
    esm_bsp_current_sense_cfg_t cfg;
    esm_bsp_current_sense_ops_t ops;
} esm_bsp_current_sense_slot_t;

typedef struct {
    bool is_registered;
    esm_bsp_encoder_cfg_t cfg;
    esm_bsp_encoder_ops_t ops;
} esm_bsp_encoder_slot_t;

static esm_bsp_pwm_slot_t s_pwm_slots[ESM_BSP_PWM_MAX_INSTANCE];//PWM实例槽位数组，存储每个PWM实例的注册状态、配置和操作函数
static esm_bsp_current_sense_slot_t s_current_sense_slots[ESM_BSP_CURRENT_SENSE_MAX_INSTANCE];//电流传感器实例槽位数组，存储每个电流传感器实例的注册状态、配置和操作函数
static esm_bsp_encoder_slot_t s_encoder_slots[ESM_BSP_ENCODER_MAX_INSTANCE];//编码器实例槽位数组，存储每个编码器实例的注册状态、配置和操作函数

bsp_status_t esm_bsp_pwm_register(uint8_t instance_id, const esm_bsp_pwm_ops_t *ops, const esm_bsp_pwm_cfg_t *cfg)
{
    if (instance_id >= ESM_BSP_PWM_MAX_INSTANCE || ops == NULL || cfg == NULL) {
        return BSP_ERR_INVALID_ARG;
    }
    s_pwm_slots[instance_id].is_registered = true;
    s_pwm_slots[instance_id].cfg = *cfg;
    memcpy(&s_pwm_slots[instance_id].ops, ops, sizeof(*ops));
    return BSP_OK;
}

const esm_bsp_pwm_ops_t *esm_bsp_pwm_get_ops(uint8_t instance_id)
{
    if (instance_id >= ESM_BSP_PWM_MAX_INSTANCE || !s_pwm_slots[instance_id].is_registered) {
        return NULL;
    }
    return &s_pwm_slots[instance_id].ops;
}

const esm_bsp_pwm_cfg_t *esm_bsp_pwm_get_cfg(uint8_t instance_id)
{
    if (instance_id >= ESM_BSP_PWM_MAX_INSTANCE || !s_pwm_slots[instance_id].is_registered) {
        return NULL;
    }
    return &s_pwm_slots[instance_id].cfg;
}

bsp_status_t esm_bsp_current_sense_register(uint8_t instance_id, const esm_bsp_current_sense_ops_t *ops, const esm_bsp_current_sense_cfg_t *cfg)//注册电流传感器实例的操作函数和配置结构体到对应槽位
{
    if (instance_id >= ESM_BSP_CURRENT_SENSE_MAX_INSTANCE || ops == NULL || cfg == NULL) {
        return BSP_ERR_INVALID_ARG;
    }
    s_current_sense_slots[instance_id].is_registered = true;
    s_current_sense_slots[instance_id].cfg = *cfg;
    memcpy(&s_current_sense_slots[instance_id].ops, ops, sizeof(*ops));
    return BSP_OK;
}

const esm_bsp_current_sense_ops_t *esm_bsp_current_sense_get_ops(uint8_t instance_id)
{
    if (instance_id >= ESM_BSP_CURRENT_SENSE_MAX_INSTANCE || !s_current_sense_slots[instance_id].is_registered) {
        return NULL;
    }
    return &s_current_sense_slots[instance_id].ops;
}

const esm_bsp_current_sense_cfg_t *esm_bsp_current_sense_get_cfg(uint8_t instance_id)
{
    if (instance_id >= ESM_BSP_CURRENT_SENSE_MAX_INSTANCE || !s_current_sense_slots[instance_id].is_registered) {
        return NULL;
    }
    return &s_current_sense_slots[instance_id].cfg;
}

bsp_status_t esm_bsp_encoder_register(uint8_t instance_id, const esm_bsp_encoder_ops_t *ops, const esm_bsp_encoder_cfg_t *cfg)
{
    if (instance_id >= ESM_BSP_ENCODER_MAX_INSTANCE || ops == NULL || cfg == NULL) {
        return BSP_ERR_INVALID_ARG;
    }
    s_encoder_slots[instance_id].is_registered = true;
    s_encoder_slots[instance_id].cfg = *cfg;
    memcpy(&s_encoder_slots[instance_id].ops, ops, sizeof(*ops));
    return BSP_OK;
}

const esm_bsp_encoder_ops_t *esm_bsp_encoder_get_ops(uint8_t instance_id)
{
    if (instance_id >= ESM_BSP_ENCODER_MAX_INSTANCE || !s_encoder_slots[instance_id].is_registered) {
        return NULL;
    }
    return &s_encoder_slots[instance_id].ops;
}

const esm_bsp_encoder_cfg_t *esm_bsp_encoder_get_cfg(uint8_t instance_id)
{
    if (instance_id >= ESM_BSP_ENCODER_MAX_INSTANCE || !s_encoder_slots[instance_id].is_registered) {
        return NULL;
    }
    return &s_encoder_slots[instance_id].cfg;
}
