#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef enum {
	ESM_CONTROL_MODE_NONE = 0,
	ESM_CONTROL_MODE_TORQUE,
	ESM_CONTROL_MODE_VELOCITY,
	ESM_CONTROL_MODE_POSITION,
} control_mode_t;

typedef struct {
	control_mode_t mode;
	uint8_t u_bus;
	float target_torque;
	float target_velocity;
	float target_position;
	float torque_limit;
} control_command_t;

typedef struct {
	float actual_torque;
	float actual_velocity;
	float actual_position;
	float iq_measured;
	float vbus_voltage;
	uint8_t fault_code;
} motor_feedback_t;

esp_err_t esm_task_comm_start(void);
esp_err_t esm_task_comm_set_control_command(const control_command_t *cmd);
esp_err_t esm_task_comm_get_control_command(control_command_t *cmd);
esp_err_t esm_task_comm_publish_motor_feedback(const motor_feedback_t *fb);
esp_err_t esm_task_comm_get_latest_motor_feedback(motor_feedback_t *fb);

