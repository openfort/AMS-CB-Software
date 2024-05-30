/*
 * battery.h
 *
 *  Created on: 12.04.2024
 *      Author: jansc
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#include "main.h"
#include "SPI_MB.h"
#include "CAN_Bus.h"

/**
  * @brief  Battery Status
  */

#define MAX_VOLT 42000
#define MIN_VOLT 30000
#define MIN_TEMP 16725		// min temperature at 20°
#define MAX_TEMP 6115		// max temperature at 60°


typedef enum
{
  BATTERY_OK       		= 0x00,
  BATTERY_ERROR    		= 0x01,
  BATTERY_TEMP_ERROR    = 0x03,
  BATTERY_VOLT_ERROR    = 0x05,
  BATTERY_VT_ERROR    	= 0x07,
} Battery_StatusTypeDef;


// struct for the battery
typedef struct {
    uint16_t totalVoltage;          // Total voltage of the accumulator system
    uint16_t highestCellVoltage;    // Highest voltage among individual cells
    uint16_t lowestCellVoltage;     // Lowest voltage among individual cells
    uint16_t meanCellVoltage;       // Mean voltage of all cells

    uint16_t highestCellTemp;
    uint16_t lowestCellTemp;
    uint16_t meanCellTemp;

    uint8_t status;     // Current status of the accumulator
    uint8_t error;		// current error flags of the accumulator

    int32_t actualCurrent;
    int32_t CurrentCounter;

	uint8_t volt_buffer[36*NUM_OF_CLIENTS];	//uint16_t volt_buffer[18*NUM_OF_CLIENTS];
	uint8_t temp_buffer[20*NUM_OF_CLIENTS]; //uint16_t temp_buffer[10*NUM_OF_CLIENTS];
} BatterySystemTypeDef;

// extern variables
extern BatterySystemTypeDef battery_values;

// external Function prototypes
uint8_t get_battery_status_code(uint16_t GPIO_Input);
void battery_reset_error_flags();
uint8_t get_battery_error_code();
void set_battery_error_flag(uint8_t mask);
//void set_reset_battery_status_flag(uint8_t set, uint8_t mask);
uint8_t volt2celsius(uint16_t volt_100uV);
Battery_StatusTypeDef SDC_reset();
Battery_StatusTypeDef check_battery();
void set_relays(uint8_t CAN_Data);
void charging(uint16_t input_data);
#endif /* INC_BATTERY_H_ */
