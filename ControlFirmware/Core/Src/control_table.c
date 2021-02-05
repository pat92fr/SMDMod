/*
 * control_table.c
 *
 *  Created on: 27 sept. 2020
 *      Author: patrick
 */

#include "control_table.h"
#include "binary_tool.h"
#include "eeprom.h"

#include "stm32g4xx_hal.h"

uint8_t regs[REG_MAX];

void factory_reset_eeprom_regs()
{
	regs[REG_MODEL_NUMBER_L] = LOW_BYTE(REG_MODEL_NUMBER_VALUE);
	regs[REG_MODEL_NUMBER_H] = HIGH_BYTE(REG_MODEL_NUMBER_VALUE);
	regs[REG_VERSION] = REG_VERSION_VALUE;
	regs[REG_ID] = REG_ID_VALUE;
	regs[REG_BAUD_RATE] = REG_BAUD_RATE_VALUE;
	regs[REG_RETURN_DELAY] = REG_RETURN_DELAY_VALUE;

	regs[REG_MAX_VELOCITY_DPS_L] = LOW_BYTE(REG_MAX_VELOCITY_DPS_VALUE);
	regs[REG_MAX_VELOCITY_DPS_H] = HIGH_BYTE(REG_MAX_VELOCITY_DPS_VALUE);
	regs[REG_MAX_ACCELERATION_DPSS_L] = LOW_BYTE(REG_MAX_ACCELERATION_DPSS_VALUE);
	regs[REG_MAX_ACCELERATION_DPSS_H] = HIGH_BYTE(REG_MAX_ACCELERATION_DPSS_VALUE);
	regs[REG_MAX_CURRENT_MA_L] = LOW_BYTE(REG_MAX_CURRENT_MA_VALUE);
	regs[REG_MAX_CURRENT_MA_H] = HIGH_BYTE(REG_MAX_CURRENT_MA_VALUE);
	regs[REG_MAX_PWM_100_L] = LOW_BYTE(REG_MAX_PWM_100_VALUE);
	regs[REG_MAX_PWM_100_H] = HIGH_BYTE(REG_MAX_PWM_100_VALUE);
	regs[REG_TEMPERATURE_LIMIT] = REG_TEMPERATURE_LIMIT_VALUE;
	regs[REG_LOW_VOLTAGE_LIMIT] = REG_LOW_VOLTAGE_LIMIT_VALUE;
	regs[REG_HIGH_VOLTAGE_LIMIT] = REG_HIGH_VOLTAGE_LIMIT_VALUE;

	regs[REG_MOVING_THRESHOLD_DPS] = REG_MOVING_THRESHOLD_DPS_VALUE;
	regs[REG_STATUS_RETURN_LVL] = REG_STATUS_RETURN_LVL_VALUE;
	regs[REG_ALARM_LED] = REG_ALARM_LED_VALUE;
	regs[REG_ALARM_SHUTDOWN] = REG_ALARM_SHUTDOWN_VALUE;

	regs[REG_CPR_L]=LOW_BYTE(REG_CPR_VALUE);
	regs[REG_CPR_H]=HIGH_BYTE(REG_CPR_VALUE);
	regs[REG_REDUCER_L]=LOW_BYTE(REG_REDUCER_VALUE);
	regs[REG_REDUCER_H]=HIGH_BYTE(REG_REDUCER_VALUE);
	regs[REG_INV_ROTATION_MOTOR] = REG_INV_ROTATION_MOTOR_VALUE;
	regs[REG_INV_ROTATION_SENSOR] = REG_INV_ROTATION_SENSOR_VALUE;

	regs[REG_PID_VELOCITY_KP_L] = LOW_BYTE(REG_PID_VELOCITY_KP_VALUE);
	regs[REG_PID_VELOCITY_KP_H] = HIGH_BYTE(REG_PID_VELOCITY_KP_VALUE);
	regs[REG_PID_VELOCITY_KI_L] = LOW_BYTE(REG_PID_VELOCITY_KI_VALUE);
	regs[REG_PID_VELOCITY_KI_H] = HIGH_BYTE(REG_PID_VELOCITY_KI_VALUE);
	regs[REG_PID_VELOCITY_KD_L] = LOW_BYTE(REG_PID_VELOCITY_KD_VALUE);
	regs[REG_PID_VELOCITY_KD_H] = HIGH_BYTE(REG_PID_VELOCITY_KD_VALUE);
	regs[REG_PID_VELOCITY_KFF_L] = LOW_BYTE(REG_PID_VELOCITY_KFF_VALUE);
	regs[REG_PID_VELOCITY_KFF_H] = HIGH_BYTE(REG_PID_VELOCITY_KFF_VALUE);
	regs[REG_PID_ACCELERATION_KFF_L] = LOW_BYTE(REG_PID_ACCELERATION_KFF_VALUE);
	regs[REG_PID_ACCELERATION_KFF_H] = HIGH_BYTE(REG_PID_ACCELERATION_KFF_VALUE);

	regs[REG_PID_CURRENT_KP_L] = LOW_BYTE(REG_PID_CURRENT_KP_VALUE);
	regs[REG_PID_CURRENT_KP_H] = HIGH_BYTE(REG_PID_CURRENT_KP_VALUE);
	regs[REG_PID_CURRENT_KI_L] = LOW_BYTE(REG_PID_CURRENT_KI_VALUE);
	regs[REG_PID_CURRENT_KI_H] = HIGH_BYTE(REG_PID_CURRENT_KI_VALUE);
	regs[REG_PID_CURRENT_KFF_L] = LOW_BYTE(REG_PID_CURRENT_KFF_VALUE);
	regs[REG_PID_CURRENT_KFF_H] = HIGH_BYTE(REG_PID_CURRENT_KFF_VALUE);

	regs[REG_CAL_CURRENT_SENSE_A_L] = LOW_BYTE(REG_CAL_CURRENT_SENSE_A_VALUE);
	regs[REG_CAL_CURRENT_SENSE_A_H] = HIGH_BYTE(REG_CAL_CURRENT_SENSE_A_VALUE);
	regs[REG_CAL_VOLTAGE_SENSE_L] = LOW_BYTE(REG_CAL_VOLTAGE_SENSE_VALUE);
	regs[REG_CAL_VOLTAGE_SENSE_H] = HIGH_BYTE(REG_CAL_VOLTAGE_SENSE_VALUE);


	eeprom_store(regs, REG_TORQUE_ENABLE); // REG_TORQUE_ENABLE must be 64 bits aligned
}


void load_eeprom_regs()
{
	eeprom_restore(regs, REG_TORQUE_ENABLE); // REG_TORQUE_ENABLE must be 64 bits aligned
}

void store_eeprom_regs()
{
	eeprom_store(regs, REG_TORQUE_ENABLE); // REG_TORQUE_ENABLE must be 64 bits aligned
}

void reset_ram_regs()
{

	regs[REG_TORQUE_ENABLE] = 0; 	// OFF
	regs[REG_LED] = 0;				// OFF
	regs[REG_CONTROL_MODE] = REG_CONTROL_MODE_PWM;

	regs[REG_GOAL_ACCELERATION_DPSS_L] = 0;
	regs[REG_GOAL_ACCELERATION_DPSS_H] = 0;
	regs[REG_GOAL_VELOCITY_DPS_L] = 0;
	regs[REG_GOAL_VELOCITY_DPS_H] = 0;
	regs[REG_GOAL_CURRENT_MA_L] = 0;
	regs[REG_GOAL_CURRENT_MA_H] = 0;
	regs[REG_GOAL_PWM_100_L] = 0;
	regs[REG_GOAL_PWM_100_H] = 0;

	regs[REG_PRESENT_ACCELERATION_DPSS_L] = 0;
	regs[REG_PRESENT_ACCELERATION_DPSS_H] = 0;
	regs[REG_PRESENT_VELOCITY_DPS_L] = 0;
	regs[REG_PRESENT_VELOCITY_DPS_H] = 0;
	regs[REG_PRESENT_CURRENT_MA_L] = 0;
	regs[REG_PRESENT_CURRENT_MA_H] = 0;
	regs[REG_PRESENT_VOLTAGE] = 0;
	regs[REG_PRESENT_TEMPERATURE] = 0;
	regs[REG_MOVING] = 0;

	regs[REG_SETPOINT_ACCELERATION_DPSS_L] = 0;
	regs[REG_SETPOINT_ACCELERATION_DPSS_H] = 0;
	regs[REG_SETPOINT_VELOCITY_DPS_L] = 0;
	regs[REG_SETPOINT_VELOCITY_DPS_H] = 0;
	regs[REG_SETPOINT_CURRENT_MA_L] = 0;
	regs[REG_SETPOINT_CURRENT_MA_H] = 0;
	regs[REG_SETPOINT_PWM_100_L] = 0;
	regs[REG_SETPOINT_PWM_100_H] = 0;
	regs[REG_MOTOR_CURRENT_INPUT_ADC_L] = 0;
	regs[REG_MOTOR_CURRENT_INPUT_ADC_H] = 0;
	regs[REG_ENCODER_ABSOLUTE_L] = 0;
	regs[REG_ENCODER_ABSOLUTE_H] = 0;
	regs[REG_VOLTAGE_INPUT_ADC_L] = 0;
	regs[REG_VOLTAGE_INPUT_ADC_H] = 0;

	regs[REG_PROTOCOL_CRC_FAIL] = 0;
	regs[REG_HARDWARE_ERROR_STATUS] = 0;

}

