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

/**
  * @brief  Battery Status
  */
typedef enum
{
  BATTERY_OK       		= 0x00,
  BATTERY_ERROR    		= 0x01,
  BATTERY_TEMP_ERROR    = 0x03,
  BATTERY_VOLT_ERROR    = 0x05,
  BATTERY_VT_ERROR    	= 0x07,
} Battery_StatusTypeDef;

//CAN Flags in 64 bit format
#define AIR_positive 		(1<<0)
#define AIR_negative 		(1<<1)
#define Precharge_Relay 	(1<<2)
#define Battery_SW_reset	(1<<3)

// extern variables


// Function prototypes
Battery_StatusTypeDef SDC_reset();
Battery_StatusTypeDef check_battery();
void set_relays(uint64_t CAN_Data);
void balancing(void);
#endif /* INC_BATTERY_H_ */
