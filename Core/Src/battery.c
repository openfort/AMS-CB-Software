/*
 * battery.c
 *
 *  Created on: 12.04.2024
 *      Author: jansc
 */

#include "battery.h"

static uint8_t error_counter = 2;
BatterySystemTypeDef battery_values;

/*
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
*/

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

uint8_t get_battery_status_code(uint16_t GPIO_Input){
	set_reset_battery_status_flag((battery_values.error&0x1F)==0 ? 1 : 0, STATUS_BATTERY_OK);
	set_reset_battery_status_flag(battery_values.adbms_itemp<=85 ? 1 : 0, STATUS_MB_TEMP_OK);
	set_reset_battery_status_flag((GPIO_Input&V_FB_AIR_positive_Pin)==V_FB_AIR_positive_Pin ? 1 : 0, STATUS_AIR_POSITIVE);
	set_reset_battery_status_flag((GPIO_Input&V_FB_AIR_negative_Pin)==V_FB_AIR_negative_Pin ? 1 : 0, STATUS_AIR_NEGATIVE);
	set_reset_battery_status_flag((GPIO_Input&V_FB_PC_Relay_Pin)==V_FB_PC_Relay_Pin ? 1 : 0, STATUS_PRECHARGE);
	return battery_values.status;
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

BatterySystemTypeDef* calc_Battery_values(uint16_t *volt_data, uint16_t *temp_data){
	// get total, mean, min, max
	uint32_t total = 0;
	uint16_t min = 50000;
	uint16_t max = 0;
	for(uint16_t i = 0; i<(18*NUM_OF_CLIENTS); i++){
		if((i != 142) && (i != 143)){		// 2 zellen nicht bestückt
			total += volt_data[i];
			if(volt_data[i] < min){
				min = volt_data[i];
			}
			if(volt_data[i] > max){
				max = volt_data[i];
			}
		}
	}
	battery_values.meanCellVoltage = (uint16_t)(total / (18*NUM_OF_CLIENTS-2));		// 2 zellen nicht bestückt
	battery_values.totalVoltage = (uint16_t)(total /= 1000); 		// total voltage in 0.1V/bit
	battery_values.lowestCellVoltage = min;
	battery_values.highestCellVoltage = max;

	total = 0;
	min = 50000;
	max = 0;
	for(uint16_t i = 0; i<(8*NUM_OF_CLIENTS); i++){
		if((i != 1) && (i != 26)){		// 2 sensoren defekt
			total += temp_data[i];
			if(temp_data[i] < min){
				min = temp_data[i];
			}
			if(temp_data[i] > max){
				max = temp_data[i];
			}
		}
	}
	battery_values.meanCellTemp = (uint16_t)(total / (8*NUM_OF_CLIENTS-2));		// 2 sensoren defekt
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
	if(TIM16->CNT > 500){
		set_battery_error_flag(ERROR_IVT);
	}
	if ((battery_values.error&0x1F) == 0){
		// SDC OK
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin;	// SDC high
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

Battery_StatusTypeDef check_battery(){
	HAL_StatusTypeDef status = Read_Voltages(battery_values.volt_buffer);
	status |= Read_Temp(battery_values.temp_buffer);

	if(status){
		set_battery_error_flag(ERROR_SPI|ERROR_BATTERY);
	}else{
		User_LED_GPIO_Port->ODR ^= User_LED_Pin; // Toggle user LED if communication works
		calc_Battery_values(battery_values.volt_buffer, battery_values.temp_buffer);
		// check limits
		if((battery_values.highestCellVoltage > MAX_VOLT) || (battery_values.lowestCellVoltage < MIN_VOLT)){
			set_battery_error_flag(ERROR_VOLT|ERROR_BATTERY);
		}
		if((battery_values.highestCellTemp < MAX_TEMP) || (battery_values.lowestCellTemp > MIN_TEMP)){
			set_battery_error_flag(ERROR_TEMP|ERROR_BATTERY);
		}
	}
	return refresh_SDC();
}

Battery_StatusTypeDef SDC_reset(){
	error_counter = 2;
	HAL_StatusTypeDef status_hw;
	status_hw = ADBMS_HW_Init();
	status_hw |= check_battery();
	if(TIM16->CNT > 500){
		status_hw |= HAL_ERROR;
	}
	// SDC on / off
	if(status_hw == HAL_OK){
		TIM7->CNT = 0;		// reset tim7 timeout counter
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin;	// SDC high
		return BATTERY_OK;
	}else{
		SDC_Out_GPIO_Port->BSRR = SDC_Out_Pin<<16;	// SDC low
		return BATTERY_ERROR;
	}
}

void stop_balancing(){
	for(uint8_t i=0; i<NUM_OF_CLIENTS; i++){
		battery_values.balance_cells[i] = 0;
	}
	set_DCCx(battery_values.balance_cells);		// stop balancing
}

void balancing(){				// retrun if charger should be active
	uint16_t itemp = read_ADBMS_Temp();
	battery_values.adbms_itemp = itemp;
	if(itemp>=83){
		stop_balancing();
	}else{
		// do some balancing
		if(battery_values.highestCellVoltage >= 41000){		// start balancing at 4.10 V
			for(uint16_t i=0; i<NUM_OF_CLIENTS; i++){
				battery_values.balance_cells[i] = 0;
				for(uint8_t j=0; j<18; j++){
					// get difference per cell to lowest cell
					if((battery_values.volt_buffer[i*18 + j]-battery_values.lowestCellVoltage) > 100){
						battery_values.balance_cells[i] |= 1<<j;
					}
				}
			}
			set_DCCx(battery_values.balance_cells);		// actuall balancing command

		}else{
			stop_balancing();
		}
	}
}

void precharge_logic(){
	static uint32_t GPIOB_old = 0;
	static uint32_t cnt_100ms = 0;
	uint32_t GPIOB_Input = Precharge_EN_GPIO_Port->IDR;
	if ((GPIOB_Input & Precharge_EN_Pin)>(GPIOB_old & Precharge_EN_Pin)){ 			// rising edge pc_en
		set_relays(AIR_NEGATIVE | PRECHARGE_RELAY);
		cnt_100ms = 0;
	}else if(GPIOB_Input & Precharge_EN_Pin){										// pc_en high
		if(cnt_100ms > 55){
			set_relays(AIR_NEGATIVE | AIR_POSITIVE);
		}else if(cnt_100ms > 50){
			set_relays(AIR_NEGATIVE | AIR_POSITIVE | PRECHARGE_RELAY);
			cnt_100ms++;		// increment every 100ms
		}else{
			cnt_100ms++;		// increment every 100ms
		}
	}else if((GPIOB_Input & Precharge_EN_Pin)<(GPIOB_old & Precharge_EN_Pin)){		// falling edge pc_en
		set_relays(0);
	}
	GPIOB_old = GPIOB_Input;
}

void charging(uint32_t input_data){
	// precharge logic
	precharge_logic();

	// charging logic
   	if(!(input_data & Charger_Con_Pin)){		// charger connected
		if((battery_values.status&STATUS_CHARGING) == 0){
			set_reset_battery_status_flag(1, STATUS_CHARGING);
		}else{
			balancing();
		}
	}else{
		if((battery_values.status&STATUS_CHARGING) == STATUS_CHARGING){		// charger disconnected
			set_reset_battery_status_flag(0, STATUS_CHARGING);
			battery_values.adbms_itemp = 0;
			stop_balancing();
		}
	}
}

void set_time_per_measurement(uint16_t time_ms){
	battery_values.time_per_measurement = time_ms;
}
