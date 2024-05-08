/*
 * battery.c
 *
 *  Created on: 12.04.2024
 *      Author: jansc
 */

#include "battery.h"

static uint8_t error_counter = 2;
BatterySystemTypeDef battery_values;

void init_Battery_values(){
	battery_values.totalVoltage = 0;
	battery_values.highestCellVoltage = 0;
	battery_values.lowestCellVoltage = 0;
	battery_values.meanCellVoltage = 0;

	battery_values.highestCellTemp = 0;
	battery_values.lowestCellTemp = 0;
	battery_values.meanCellTemp = 0;

	battery_values.actualCurrent = 0;
	battery_values.status = BATTERY_ERROR;
}

BatterySystemTypeDef* calc_Battery_values(uint8_t *volt_buffer, uint8_t *temp_buffer){
	uint16_t *volt_data = (uint16_t*)(volt_buffer);
	uint16_t *temp_data = (uint16_t*)(temp_buffer);

	// get total, mean, min, max
	uint32_t total = 0;
	uint16_t min = 50000;
	uint16_t max = 0;
	for(uint16_t i = 0; i<(18*num_of_clients); i++){
		total += volt_data[i];
		if(volt_data[i] < min){
			min = volt_data[i];
		}
		if(volt_data[i] > max){
			max = volt_data[i];
		}
	}
	battery_values.meanCellVoltage = (uint16_t)(total / (18*num_of_clients));
	battery_values.totalVoltage = (uint16_t)(total /= 1000); 		// total voltage in 0.1V/bit
	battery_values.lowestCellVoltage = min;
	battery_values.highestCellVoltage = max;

	total = 0;
	min = 50000;
	max = 0;
	for(uint16_t i = 0; i<(8*num_of_clients); i++){
		total += temp_data[i];
		if(temp_data[i] < min){
			min = temp_data[i];
		}
		if(temp_data[i] > max){
			max = temp_data[i];
		}
	}
	battery_values.meanCellTemp = (uint16_t)(total / (8*num_of_clients));
	battery_values.lowestCellTemp = min;
	battery_values.highestCellTemp = max;
	return &battery_values;
}

Battery_StatusTypeDef refresh_SDC(Battery_StatusTypeDef status){
	if(SDC_IN_GPIO_Port->IDR & SDC_IN_Pin){
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
		return BATTERY_ERROR;
	}
	if (status == BATTERY_OK){
		// SDC OK
		// reset tim7 timeout counter
		TIM7->CNT = 0;
		error_counter = 0;
	}else{
		// SDC error
		error_counter++;
		if(error_counter >= 3){
			SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
			return status;
		}
	}
	return BATTERY_OK;
}

Battery_StatusTypeDef SDC_reset(){
	if(SDC_IN_GPIO_Port->IDR & SDC_IN_Pin){
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
		return BATTERY_ERROR;
	}
	error_counter = 2;
	uint8_t volt_buffer[36*num_of_clients];
	uint8_t temp_buffer[20*num_of_clients];
	HAL_StatusTypeDef status_hw = ADBMS_HW_Init();
	status_hw |= Read_Voltages(volt_buffer);
	status_hw |= Read_Temp(temp_buffer);
	status_hw |= check_battery(volt_buffer, temp_buffer);
	// SDC on / off
	if(status_hw == HAL_OK){
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin;	// SDC high
		// reset tim7 timeout counter
		TIM7->CNT = 0;
		return BATTERY_OK;
	}else{
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
		return BATTERY_ERROR;
	}
}

Battery_StatusTypeDef check_battery(uint8_t *volt_buffer, uint8_t *temp_buffer){
	calc_Battery_values(volt_buffer, temp_buffer);
	// check limits
	Battery_StatusTypeDef status = BATTERY_OK;
	if((battery_values.highestCellVoltage > MAX_VOLT) || (battery_values.lowestCellVoltage < MIN_VOLT)){
		status |= BATTERY_VOLT_ERROR;
	}
	if((battery_values.highestCellTemp > MAX_TEMP) || (battery_values.lowestCellTemp < MIN_TEMP)){
		status |= BATTERY_TEMP_ERROR;
	}
	battery_values.status = refresh_SDC(status);
	return battery_values.status;
}

void set_relays(uint8_t CAN_Data){
	static uint64_t last_value = 0;
	if(last_value != CAN_Data){
		if(CAN_Data & AIR_positive){
			Drive_AIR_positive_GPIO_Port->BSRR = Drive_AIR_positive_Pin;	// high
		}else{
			Drive_AIR_positive_GPIO_Port->BSRR = Drive_AIR_positive_Pin<<16;	// low
		}
		if(CAN_Data & AIR_negative){
			Drive_AIR_negative_GPIO_Port->BSRR = Drive_AIR_negative_Pin;	// high
		}else{
			Drive_AIR_negative_GPIO_Port->BSRR = Drive_AIR_negative_Pin<<16;	// low
		}
		if(CAN_Data & Precharge_Relay){
			Drive_Precharge_Relay_GPIO_Port->BSRR = Drive_Precharge_Relay_Pin;	// high
		}else{
			Drive_Precharge_Relay_GPIO_Port->BSRR = Drive_Precharge_Relay_Pin<<16;	// low
		}
	}
	last_value = CAN_Data;
}

uint8_t balancing(){
	// do some balancing
	return 1;

}
