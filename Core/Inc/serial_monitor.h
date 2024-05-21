/*
 * serial_monitor.h
 *
 *  Created on: 12.04.2024
 *      Author: jansc
 */

#ifndef INC_SERIAL_MONITOR_H_
#define INC_SERIAL_MONITOR_H_

#include "main.h"

// serial command type
typedef enum
{
  volt		= 0xA0,
  temp		= 0xA1,
  can		= 0xA2,
  all_values= 0xA3,
} Serial_Commmand_type;

// HAL Handle
extern UART_HandleTypeDef huart2;

// Function prototypes
void SerialMonitor(Serial_Commmand_type command, uint8_t* data, uint16_t size);

#endif /* INC_SERIAL_MONITOR_H_ */
