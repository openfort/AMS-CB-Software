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
	battery_values.CurrentCounter = 0;

	battery_values.status = 0;
	battery_values.error = 0;
}

uint8_t get_battery_status_code(uint16_t GPIO_Input){
	uint8_t status_code = 0;
	status_code |= (battery_values.error&0x1F)==0 ? STATUS_BATTERY_OK : 0;
	status_code |= (GPIO_Input&Charger_Con_Pin)==Charger_Con_Pin ? STATUS_CHARGING : 0;
	// status MB temp
	status_code |= (GPIO_Input&V_FB_AIR_positive_Pin)==V_FB_AIR_positive_Pin ? STATUS_AIR_POSITIVE : 0;
	status_code |= (GPIO_Input&V_FB_AIR_negative_Pin)==V_FB_AIR_negative_Pin ? STATUS_AIR_NEGATIVE : 0;
	status_code |= (GPIO_Input&V_FB_PC_Relay_Pin)==V_FB_PC_Relay_Pin ? STATUS_PRECHARGE : 0;
	battery_values.status |= status_code;
	return status_code;
}

void battery_reset_error_flags(){
	battery_values.error = 0;
}

uint8_t get_battery_error_code(){
	return battery_values.error;
}

void set_battery_error_flag(uint8_t mask){
	battery_values.error |= mask;
}

void set_reset_battery_status_flag(uint8_t set, uint8_t mask){
	if(set){
		battery_values.status |= mask;
	}else{
		battery_values.status &= ~mask;
	}
}

BatterySystemTypeDef* calc_Battery_values(uint8_t *volt_buffer, uint8_t *temp_buffer){
	uint16_t *volt_data = (uint16_t*)(volt_buffer);
	uint16_t *temp_data = (uint16_t*)(temp_buffer);

	// get total, mean, min, max
	uint32_t total = 0;
	uint16_t min = 50000;
	uint16_t max = 0;
	for(uint16_t i = 0; i<(18*NUM_OF_CLIENTS); i++){
		total += volt_data[i];
		if(volt_data[i] < min){
			min = volt_data[i];
		}
		if(volt_data[i] > max){
			max = volt_data[i];
		}
	}
	battery_values.meanCellVoltage = (uint16_t)(total / (18*NUM_OF_CLIENTS));
	battery_values.totalVoltage = (uint16_t)(total /= 1000); 		// total voltage in 0.1V/bit
	battery_values.lowestCellVoltage = min;
	battery_values.highestCellVoltage = max;

	total = 0;
	min = 50000;
	max = 0;
	for(uint16_t i = 0; i<(8*NUM_OF_CLIENTS); i++){
		if(i != 1){		// temp sensor 1 defekt
			total += temp_data[i];
			if(temp_data[i] < min){
				min = temp_data[i];
			}
			if(temp_data[i] > max){
				max = temp_data[i];
			}
		}
	}
	battery_values.meanCellTemp = (uint16_t)(total / (8*NUM_OF_CLIENTS-1));		// 1 sensor defekt
	battery_values.highestCellTemp = min;
	battery_values.lowestCellTemp = max;
	return &battery_values;
}

uint8_t volt2celsius(uint16_t volt_100uV){		// convert volt to celsius with polynom
	if(volt_100uV > 23000){
		return 0;
	}else if(volt_100uV < 2000){
		return 100;
	}
	// Coefficients of the polynomial: a0, a1, ..., a10
    double coefficients[11] = {1.65728946e+02, -5.76649020e-02, 1.80075051e-05, -3.95278974e-09, 5.86752736e-13, -5.93033515e-17, 4.07565006e-21, -1.87118391e-25, 5.48516319e-30, -9.27411410e-35, 6.87565181e-40};

    // Calculate the polynomial value
    double result = coefficients[10];
    for (int8_t i = 9; i >= 0; i--) {
        result = result * volt_100uV + coefficients[i];
    }
    return (uint16_t)(result);		// in °C
}

Battery_StatusTypeDef refresh_SDC(){
	/*
	if(SDC_IN_GPIO_Port->IDR & SDC_IN_Pin){
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
		set_battery_error_flag(ERROR_SDC);
		return BATTERY_ERROR;
	*/
	if ((battery_values.error&0x1F) == 0){
		// SDC OK
		// reset tim7 timeout counter
		TIM7->CNT = 0;
		error_counter = 0;
	}else{
		// SDC error
		error_counter++;
		if(error_counter >= 3){
			SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
			set_battery_error_flag(ERROR_SDC);
			set_relays(0);								// open AIR relais
			return BATTERY_ERROR;
		}
	}
	return BATTERY_OK;
}

Battery_StatusTypeDef SDC_reset(){
	/*
	if(SDC_IN_GPIO_Port->IDR & SDC_IN_Pin){
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
		return BATTERY_ERROR;
	}
	*/
	error_counter = 2;
	uint8_t volt_buffer[36*NUM_OF_CLIENTS];
	uint8_t temp_buffer[20*NUM_OF_CLIENTS];
	HAL_StatusTypeDef status_hw;
	status_hw = ADBMS_HW_Init();
	status_hw |= Read_Voltages(volt_buffer);
	status_hw |= Read_Temp(temp_buffer);
	status_hw |= check_battery(volt_buffer, temp_buffer);
	// SDC on / off
	if(status_hw == HAL_OK){
		TIM7->CNT = 0;
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin;	// SDC high
		// reset tim7 timeout counter
		return BATTERY_OK;
	}else{
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
		return BATTERY_ERROR;
	}
}

Battery_StatusTypeDef check_battery(){
	HAL_StatusTypeDef status = Read_Voltages(battery_values.volt_buffer);
	status |= Read_Temp(battery_values.temp_buffer);

	if(status){
		set_battery_error_flag(ERROR_SPI|ERROR_BATTERY);
	}else{
		calc_Battery_values(battery_values.volt_buffer, battery_values.temp_buffer);
		// check limits
		if((battery_values.highestCellVoltage > MAX_VOLT) || (battery_values.lowestCellVoltage < MIN_VOLT)){
			set_battery_error_flag(ERROR_VOLT|ERROR_BATTERY);
		}
		if((battery_values.highestCellTemp < MAX_TEMP) || (battery_values.lowestCellTemp > MIN_TEMP)){
			set_battery_error_flag(ERROR_TEMP|ERROR_BATTERY);
		}
	}
	return refresh_SDC();;
}

void set_relays(uint8_t CAN_Data){
	static uint64_t last_value = 0;
	if(last_value != CAN_Data){
		if(CAN_Data & AIR_POSITIVE){
			Drive_AIR_positive_GPIO_Port->BSRR = Drive_AIR_positive_Pin;	// high
		}else{
			Drive_AIR_positive_GPIO_Port->BSRR = Drive_AIR_positive_Pin<<16;	// low
		}
		if(CAN_Data & AIR_NEGATIVE){
			Drive_AIR_negative_GPIO_Port->BSRR = Drive_AIR_negative_Pin;	// high
		}else{
			Drive_AIR_negative_GPIO_Port->BSRR = Drive_AIR_negative_Pin<<16;	// low
		}
		if(CAN_Data & PRECHARGE_RELAY){
			Drive_Precharge_Relay_GPIO_Port->BSRR = Drive_Precharge_Relay_Pin;	// high
		}else{
			Drive_Precharge_Relay_GPIO_Port->BSRR = Drive_Precharge_Relay_Pin<<16;	// low
		}
	}
	last_value = CAN_Data;
}

uint8_t balancing(uint16_t *volt_data){		// retrun if charger should be active
	// do some balancing
	uint32_t balance_cells[NUM_OF_CLIENTS];
	if(battery_values.highestCellVoltage >= 33000){		// start balancing at 4.15 V
		for(uint16_t i=0; i<NUM_OF_CLIENTS; i++){
			balance_cells[i] = 0;
			for(uint8_t j=0; j<18; j++){
				// get difference per cell to lowest cell
				if((volt_data[i*18 + j]-battery_values.lowestCellVoltage) > 100){
					balance_cells[i] |= 1<<j;

				}
			}
			//balance_cells[i] &= 0x0003C000;
		}
		set_DCCx(balance_cells);
		//write_balancing_PWM(1, balance_value);

		if(battery_values.highestCellVoltage >= 41900){		// stop charging at 4.19 V
			return 0;
		}else{
			return 1;
		}
	}else{
		for(uint8_t i=0; i<NUM_OF_CLIENTS; i++){
			balance_cells[i] = 0;
		}
		set_DCCx(balance_cells);
		return 1;
	}
}
