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
#define MAX_TEMP 16725		// min temperature at 20°
#define MIN_TEMP 6115		// max temperature at 60°

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

    uint16_t actualCurrent;
    Battery_StatusTypeDef status;     // Current status of the accumulator
} BatterySystemTypeDef;

// extern variables
extern BatterySystemTypeDef battery_values;

// Function prototypes
Battery_StatusTypeDef SDC_reset();
Battery_StatusTypeDef check_battery();
void set_relays(uint8_t CAN_Data);
uint8_t balancing();
#endif /* INC_BATTERY_H_ */
