/*
 * battery.c
 *
 *  Created on: 12.04.2024
 *      Author: jansc
 */

#include "battery.h"

static uint8_t error_counter = 2;

Battery_StatusTypeDef refresh_SDC(Battery_StatusTypeDef status){
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
			return BATTERY_ERROR;
		}
	}
	return BATTERY_OK;
}

Battery_StatusTypeDef SDC_reset(){
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
	// check limits
	uint16_t *volt_data = (uint16_t*)(volt_buffer);
	uint16_t *temp_data = (uint16_t*)(temp_buffer);
	Battery_StatusTypeDef status = BATTERY_OK;

	for(uint16_t i = 0; i<(36*num_of_clients)>>1; i++){
		// check over-, undervoltage
		if(volt_data[i] < MIN_VOLT || volt_data[i] > MAX_VOLT){
			status |= BATTERY_VOLT_ERROR;
		}
	}
	for(uint16_t i = 0; i<(20*num_of_clients)>>1; i++){
		if(temp_data[i] < MIN_TEMP || temp_data[i] > MAX_TEMP){
			//status |= BATTERY_TEMP_ERROR;
		}
	}
	return refresh_SDC(status);
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
